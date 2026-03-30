"""
=============================================================================
  Isaac Sim 5.1 — Person + Forklift Simulation
=============================================================================

  WHAT YOU WILL SEE
  -----------------
  One person walks randomly using IRA's NavMesh (guaranteed obstacle-free).
  Three forklifts patrol the same warehouse using the same NavMesh:
    - Waypoints are sampled from the NavMesh (only walkable positions)
    - Paths between waypoints are computed via NavMesh shortest-path
      (automatically routes around shelves and walls)
    - Z position is snapped to the NavMesh surface every frame
      (forklifts follow the actual floor, never float)
    - Ackermann / bicycle kinematic model (rear-wheel-steered)
    - Acceleration, braking, speed reduction in tight turns
    - Fork mast animation: lower to pick up, raise for travel

  HOW TO RUN
  ----------
  Isaac Sim  →  Window  →  Script Editor  →  Open this file  →  Ctrl+Enter

  HOW IT WORKS
  ------------
  1. IRA loads the warehouse and bakes the NavMesh.
  2. After bake: the person's NavMesh waypoints are generated, and forklifts
     are spawned at NavMesh-snapped positions.
  3. Each forklift gets a random NavMesh destination and a shortest-path
     route to it.  A physics-step callback moves them along the route every
     frame using the Ackermann kinematic model.

=============================================================================
"""

import asyncio
import carb
import math
import os
import random

import omni.anim.navigation.core as nav_ext
import omni.physx
import omni.timeline
import yaml

from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import get_current_stage
from pxr import Gf, Sdf, Usd, UsdGeom

# ─────────────────────────────────────────────────────────────────────────────
#  SETTINGS
# ─────────────────────────────────────────────────────────────────────────────

ASSETS_ROOT       = "/home/ubuntu/isaacsim_assets/Assets/Isaac/5.1"
SCENE_USD         = f"{ASSETS_ROOT}/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
CHARACTERS_FOLDER = f"{ASSETS_ROOT}/Isaac/People/Characters/"
FORKLIFT_USD      = f"{ASSETS_ROOT}/Isaac/Props/Forklift/forklift.usd"

NUM_FORKLIFTS  = 3      # forklifts to spawn
COMMAND_FRAMES = 600    # person waypoint frames (30 fps, loops forever)

# ─────────────────────────────────────────────────────────────────────────────
#  STEP 1 — Enable IRA (drives the person)
# ─────────────────────────────────────────────────────────────────────────────
enable_extension("isaacsim.replicator.agent.core")

from isaacsim.replicator.agent.core.simulation import SimulationManager

# ─────────────────────────────────────────────────────────────────────────────
#  STEP 2 — Write IRA support files
# ─────────────────────────────────────────────────────────────────────────────
HERE           = os.path.dirname(os.path.abspath(__file__))
CMD_PATH       = os.path.join(HERE, "_person_commands.txt")
ROBOT_CMD_PATH = os.path.join(HERE, "_robot_commands.txt")
CFG_PATH       = os.path.join(HERE, "_ira_config.yaml")

with open(CMD_PATH,       "w") as f: f.write("# filled with NavMesh waypoints at startup\n")
with open(ROBOT_CMD_PATH, "w") as f: f.write("# no robots\n")

# ─────────────────────────────────────────────────────────────────────────────
#  STEP 3 — Write IRA config
# ─────────────────────────────────────────────────────────────────────────────
ira_config = {
    "isaacsim.replicator.agent": {
        "version": "0.7.0",
        "global": {"seed": 42, "simulation_length": COMMAND_FRAMES},
        "scene":  {"asset_path": SCENE_USD},
        "character": {
            "asset_path":      CHARACTERS_FOLDER,
            "command_file":    CMD_PATH,
            "filters":         [],
            "navigation_area": [],   # [] = entire NavMesh
            "spawn_area":      [],   # [] = anywhere on NavMesh
            "num": 1,
        },
        "robot": {
            "command_file":    ROBOT_CMD_PATH,
            "nova_carter_num": 0,
            "transporter_num": 0,
            "iw_hub_num":      0,
            "spawn_area":      [],
            "navigation_area": [],
            "write_data":      False,
        },
        "write_data": False,
    }
}
with open(CFG_PATH, "w") as f:
    yaml.dump(ira_config, f)

# ─────────────────────────────────────────────────────────────────────────────
#  FORKLIFT — kinematic constants
# ─────────────────────────────────────────────────────────────────────────────

# Rear-wheel-steered bicycle model — same physics as a real counterbalance forklift
FL_WHEELBASE      = 2.4    # m   — axle-to-axle distance
FL_MAX_SPEED      = 3.0    # m/s — peak travel speed (~11 km/h)
FL_MIN_SPEED      = 0.4    # m/s — minimum creep speed
FL_ACCEL          = 1.5    # m/s² — max acceleration
FL_BRAKE          = 2.5    # m/s² — max braking deceleration
FL_MAX_STEER      = 65.0   # deg — rear-axle steer limit
FL_STEER_RATE     = 100.0  # deg/s — steering angle rate of change
FL_HEADING_OFF    = 90.0   # deg — USD model's forward axis is +Y, not +X
FL_WHEEL_R        = 0.35   # m   — wheel radius for spin animation
FL_ARRIVE         = 2.0    # m   — arrival radius for the final waypoint
FL_PATH_ARRIVE    = 1.0    # m   — arrival radius for intermediate path points

# Fork mast animation
FORK_RAISE_SPEED  = 0.25   # m/s — vertical speed of forks
FORK_TRAVEL_H     = 0.45   # m   — raised during travel (OSHA safe height)
FORK_GROUND_H     = 0.0    # m   — lowered at pickup / drop-off
IDLE_DURATION     = 3.5    # s   — pause at each waypoint

STATE_DRIVE = "drive"
STATE_IDLE  = "idle"

# Fallback floor bounds if NavMesh is unavailable
NAV_X_MIN, NAV_X_MAX = -24.5,  3.7
NAV_Y_MIN, NAV_Y_MAX = -21.6, 28.8

# Module-level state (filled in run() after IRA setup)
_navmesh  = None   # INavMesh interface — available after NavMesh bake
forklifts = []     # list of forklift state dicts
_physx_sub = None  # kept alive at module level so the callback is never GC'd

# ─────────────────────────────────────────────────────────────────────────────
#  NAVMESH HELPERS
#
#  All three functions use the baked NavMesh that IRA creates at startup.
#  They are safe to call only AFTER setup_done has fired.
# ─────────────────────────────────────────────────────────────────────────────

def _rand_navmesh_pt():
    """Return a random (x, y, z) point on the walkable NavMesh surface."""
    if _navmesh is None:
        return (random.uniform(NAV_X_MIN, NAV_X_MAX),
                random.uniform(NAV_Y_MIN, NAV_Y_MAX), 0.0)
    pt = _navmesh.query_random_point()
    if pt is None:
        return (random.uniform(NAV_X_MIN, NAV_X_MAX),
                random.uniform(NAV_Y_MIN, NAV_Y_MAX), 0.0)
    return (pt[0], pt[1], pt[2])


def _navmesh_path(from_xyz, to_xyz):
    """Compute a NavMesh shortest-path from from_xyz to to_xyz.

    Returns a list of (x, y, z) waypoints that routes around all obstacles.
    Falls back to a straight two-point list if the NavMesh is unavailable or
    no path exists.
    """
    if _navmesh is None:
        return [to_xyz]
    path_obj = _navmesh.query_shortest_path(
        carb.Float3(from_xyz[0], from_xyz[1], from_xyz[2]),
        carb.Float3(to_xyz[0],   to_xyz[1],   to_xyz[2]),
        agent_radius=0.0,   # use 0 so we can fit through all baked aisles
    )
    if path_obj is None:
        return [to_xyz]
    raw_pts = path_obj.get_points()
    if not raw_pts:
        return [to_xyz]
    # Convert to plain Python tuples; skip the first point (= start position)
    pts = [(p[0], p[1], p[2]) for p in raw_pts]
    return pts[1:] if len(pts) > 1 else pts


def _floor_z(x, y):
    """Return the NavMesh floor Z at (x, y) by projecting downward from above.

    This is how forklifts always rest on the warehouse floor rather than
    floating at the Z=0 plane.
    """
    if _navmesh is None:
        return 0.0
    # Project from 3 m above — finds the walkable surface below
    result = _navmesh.query_closest_point(carb.Float3(x, y, 3.0), 0.0)
    if result is None:
        return 0.0
    return result[0][2]   # result[0] is carb.Float3; [2] is Z

# ─────────────────────────────────────────────────────────────────────────────
#  FORKLIFT — USD prim helpers
# ─────────────────────────────────────────────────────────────────────────────

def _collect_wheel_prims(root_prim):
    found = []
    for child in root_prim.GetAllChildren():
        if "wheel" in child.GetName().lower():
            found.append(child)
        found.extend(_collect_wheel_prims(child))
    return found

def _get_or_add_rotate_x(xform_obj):
    for op in xform_obj.GetOrderedXformOps():
        if "rotateX" in op.GetOpName():
            return op
    return xform_obj.AddRotateXOp()

def _collect_fork_prims(root_prim):
    found = []
    for child in Usd.PrimRange(root_prim):
        name = child.GetName().lower()
        if any(k in name for k in ("fork", "mast", "lift", "carriage", "tine")):
            if child.IsA(UsdGeom.Xformable):
                found.append(child)
    return found

def _update_fork_height(fl, prim, step_dt):
    """Smoothly move the forks toward their target height."""
    diff = fl["fork_target"] - fl["fork_height"]
    step = FORK_RAISE_SPEED * step_dt
    fl["fork_height"] = (fl["fork_target"] if abs(diff) <= step
                         else fl["fork_height"] + math.copysign(step, diff))

    if fl["fork_prims"] is None:
        fl["fork_prims"] = _collect_fork_prims(prim)
    for fp in fl["fork_prims"]:
        for op in UsdGeom.Xformable(fp).GetOrderedXformOps():
            if "translate" in op.GetOpName():
                t = op.Get()
                if t is not None:
                    op.Set(Gf.Vec3d(t[0], t[1], fl["fork_height"]))
                break

# ─────────────────────────────────────────────────────────────────────────────
#  FORKLIFT — per-frame movement
#
#  The kinematic model:
#    heading_rate = speed × tan(steer_angle) / wheelbase
#  This is the Ackermann bicycle equation for a rear-wheel-steered vehicle.
#  It produces natural curved arcs — tight at full lock, wide when straight.
# ─────────────────────────────────────────────────────────────────────────────

def _steer_speed_limit(steer_deg):
    """Speed limit based on current steer angle — forklifts slow in tight turns."""
    ratio = abs(steer_deg) / FL_MAX_STEER
    return FL_MAX_SPEED * (1.0 - 0.75 * ratio)   # 3 m/s → 0.75 m/s at full lock

def _braking_distance(speed):
    return (speed * speed) / (2.0 * FL_BRAKE)


def _move_forklift(fl, step_dt):
    stage = get_current_stage()
    prim  = stage.GetPrimAtPath(Sdf.Path(fl["path"]))
    if not prim.IsValid():
        return

    # ── STATE: IDLE — stopped at waypoint, simulating pick / drop ───────────
    if fl["state"] == STATE_IDLE:
        fl["idle_timer"] -= step_dt
        # Lower forks for the first half of the pause, then raise them
        fl["fork_target"] = (FORK_GROUND_H if fl["idle_timer"] > IDLE_DURATION * 0.5
                             else FORK_TRAVEL_H)
        _update_fork_height(fl, prim, step_dt)
        if fl["idle_timer"] <= 0.0:
            # Pick a new random NavMesh destination and compute the path to it
            new_dest = _rand_navmesh_pt()
            cur      = fl["pos"]
            fl["path_pts"] = _navmesh_path((cur[0], cur[1], cur[2]), new_dest)
            fl["path_idx"] = 0
            fl["state"]    = STATE_DRIVE
        return

    # ── STATE: DRIVE — following NavMesh path points ─────────────────────────

    # If path is empty or exhausted, generate a new one
    if not fl["path_pts"] or fl["path_idx"] >= len(fl["path_pts"]):
        new_dest = _rand_navmesh_pt()
        cur = fl["pos"]
        fl["path_pts"] = _navmesh_path((cur[0], cur[1], cur[2]), new_dest)
        fl["path_idx"] = 0
        return

    # Current target: next path point
    target     = fl["path_pts"][fl["path_idx"]]
    fx, fy, fz = fl["pos"]
    dx, dy     = target[0] - fx, target[1] - fy
    dist       = math.hypot(dx, dy)

    # Decide whether this is an intermediate point or the final destination
    is_last = (fl["path_idx"] == len(fl["path_pts"]) - 1)
    arrive_r = FL_ARRIVE if is_last else FL_PATH_ARRIVE

    # Advance path on arrival
    if dist < arrive_r:
        if is_last:
            # Reached the end — stop and idle
            fl["speed"]       = 0.0
            fl["state"]       = STATE_IDLE
            fl["idle_timer"]  = IDLE_DURATION
            fl["fork_target"] = FORK_GROUND_H
            return
        else:
            fl["path_idx"] += 1
            return

    # ── Steering controller (proportional, rate-limited) ────────────────────
    desired_heading = math.degrees(math.atan2(dy, dx)) + FL_HEADING_OFF
    heading_err     = (desired_heading - fl["heading"] + 180.0) % 360.0 - 180.0

    steer_target = max(-FL_MAX_STEER, min(FL_MAX_STEER, heading_err * 0.8))
    steer_diff   = steer_target - fl["steer_angle"]
    fl["steer_angle"] += max(-FL_STEER_RATE * step_dt,
                              min( FL_STEER_RATE * step_dt, steer_diff))

    # ── Speed control: accelerate + look-ahead braking ───────────────────────
    max_speed  = _steer_speed_limit(fl["steer_angle"])
    brake_dist = _braking_distance(fl["speed"])

    if is_last and dist < brake_dist + 0.5:
        # Decelerate to arrive gently at the final waypoint
        target_speed = max(FL_MIN_SPEED,
                           math.sqrt(max(0.0, 2.0 * FL_BRAKE * (dist - 0.3))))
    else:
        target_speed = max_speed

    speed_diff = target_speed - fl["speed"]
    if speed_diff > 0:
        fl["speed"] = min(fl["speed"] + FL_ACCEL * step_dt, target_speed)
    else:
        fl["speed"] = max(fl["speed"] - FL_BRAKE * step_dt, target_speed)
    fl["speed"] = max(0.0, fl["speed"])

    # ── Bicycle / Ackermann kinematic update ─────────────────────────────────
    #   heading_rate = speed × tan(steer_angle) / wheelbase
    steer_rad    = math.radians(fl["steer_angle"])
    heading_rate = fl["speed"] * math.tan(steer_rad) / FL_WHEELBASE
    fl["heading"] += math.degrees(heading_rate) * step_dt

    move_rad = math.radians(fl["heading"] - FL_HEADING_OFF)
    nx = fx + fl["speed"] * step_dt * math.cos(move_rad)
    ny = fy + fl["speed"] * step_dt * math.sin(move_rad)

    # ── Snap to NavMesh floor surface ────────────────────────────────────────
    # query_closest_point projects from above and returns the floor Z at (nx, ny).
    # This keeps the forklift on the actual floor and stops it entering obstacles.
    nz = _floor_z(nx, ny)

    # ── Forklift-to-forklift separation (~2 m safe gap) ──────────────────────
    SEP = 2.0
    for other in forklifts:
        if other is fl:
            continue
        ox, oy = other["pos"][0], other["pos"][1]
        d = math.hypot(nx - ox, ny - oy)
        if 0.001 < d < SEP:
            push = SEP - d
            nx += (nx - ox) / d * push
            ny += (ny - oy) / d * push
            fl["speed"] *= 0.7

    fl["pos"] = [nx, ny, nz]
    fl["fork_target"] = FORK_TRAVEL_H   # forks raised while driving

    # ── Apply USD transform ──────────────────────────────────────────────────
    xform = UsdGeom.Xformable(prim)
    ops   = {op.GetOpName(): op for op in xform.GetOrderedXformOps()}
    t_op  = ops.get("xformOp:translate")
    if t_op:
        t_op.Set(Gf.Vec3d(nx, ny, nz))
    for op in xform.GetOrderedXformOps():
        if "rotateZ" in op.GetOpName():
            op.Set(fl["heading"])
            break

    # ── Wheel spin — proportional to actual speed ────────────────────────────
    if fl["wheel_prims"] is None:
        fl["wheel_prims"] = _collect_wheel_prims(prim)
    if fl["wheel_prims"]:
        fl["wheel_angle"] += math.degrees(fl["speed"] * step_dt / FL_WHEEL_R)
        for wp_prim in fl["wheel_prims"]:
            _get_or_add_rotate_x(UsdGeom.Xformable(wp_prim)).Set(fl["wheel_angle"])

    _update_fork_height(fl, prim, step_dt)

# ─────────────────────────────────────────────────────────────────────────────
#  PHYSICS STEP CALLBACK — called every simulation frame
# ─────────────────────────────────────────────────────────────────────────────

def on_physics_step(step_dt):
    for fl in forklifts:
        _move_forklift(fl, step_dt)

# ─────────────────────────────────────────────────────────────────────────────
#  STEP 4 — Run the simulation
# ─────────────────────────────────────────────────────────────────────────────

async def run():
    global _navmesh, _physx_sub

    sim = SimulationManager()

    ok = sim.load_config_file(CFG_PATH)
    if not ok:
        print("[NavExample] ERROR: Config failed to load — check ASSETS_ROOT.")
        return

    # Wait for IRA to finish loading the scene and baking the NavMesh
    setup_done = asyncio.Event()
    _sub = sim.register_set_up_simulation_done_callback(lambda _: setup_done.set())
    print("[NavExample] Loading scene and baking NavMesh …")
    sim.set_up_simulation_from_config_file()
    await setup_done.wait()
    _sub = None

    # ── Acquire the NavMesh interface ─────────────────────────────────────────
    # IRA bakes the NavMesh before calling the setup_done callback, so the
    # interface is ready immediately after await.
    iface    = nav_ext.acquire_interface()
    _navmesh = iface.get_navmesh() if iface else None
    if _navmesh is None:
        print("[NavExample] WARNING: NavMesh interface unavailable — "
              "forklifts will use fallback wall-bounds navigation.")
    else:
        print("[NavExample] NavMesh ready.")

    # ── Spawn forklifts ───────────────────────────────────────────────────────
    print(f"[NavExample] Spawning {NUM_FORKLIFTS} forklifts …")
    stage = get_current_stage()
    cx    = -10.435   # warehouse X centre: (−26.33 + 5.46) / 2
    random.seed(42)

    for fi, x_offset in enumerate([-7.0, 0.0, 7.0][:NUM_FORKLIFTS]):
        # Pick a walkable spawn point near the desired X position
        spawn_pt = _rand_navmesh_pt()          # guaranteed on the floor
        sx       = cx + x_offset
        sy       = spawn_pt[1]                 # use NavMesh Y (walkable)
        sz       = _floor_z(sx, spawn_pt[1])  # exact floor Z at that position
        path     = f"/World/Forklifts/forklift_{fi}"

        # Create an Xform prim, set pose, attach the forklift USD reference
        xform = UsdGeom.Xform.Define(stage, path)
        xform.AddTranslateOp().Set(Gf.Vec3d(sx, sy, sz))
        xform.AddRotateZOp().Set(90.0)         # face along +Y
        stage.GetPrimAtPath(Sdf.Path(path)).GetReferences().AddReference(FORKLIFT_USD)

        # Compute an initial NavMesh path to a random destination
        first_dest = _rand_navmesh_pt()
        path_pts   = _navmesh_path((sx, sy, sz), first_dest)

        forklifts.append({
            "path":        path,
            "pos":         [sx, sy, sz],   # 3-D: [x, y, floor_z]
            "heading":     90.0,           # degrees — matches initial rotateZ
            "speed":       0.0,
            "steer_angle": 0.0,
            "state":       STATE_DRIVE,
            "idle_timer":  0.0,
            "path_pts":    path_pts,       # NavMesh path to current destination
            "path_idx":    fi,             # stagger: each forklift starts mid-path
            "fork_height": 0.0,
            "fork_target": FORK_TRAVEL_H,
            "wheel_angle": 0.0,
            "wheel_prims": None,           # collected lazily on first physics step
            "fork_prims":  None,
        })

    # ── Subscribe physics callback — drives all forklifts every frame ─────────
    # Stored at module level so it is never garbage-collected.
    _physx_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(on_physics_step)

    # ── Generate person's random NavMesh waypoints ────────────────────────────
    print("[NavExample] Generating random NavMesh waypoints for person …")
    random_commands = await sim.generate_random_commands()
    if random_commands:
        print(f"[NavExample] Generated {len(random_commands)} person waypoints.")
    else:
        print("[NavExample] WARNING: No person commands — NavMesh may be empty.")
    sim.save_commands(random_commands)

    # ── Play ──────────────────────────────────────────────────────────────────
    print("[NavExample] Simulation running!")
    print("[NavExample]   Person walks random NavMesh routes.")
    print(f"[NavExample]   {NUM_FORKLIFTS} forklifts follow NavMesh paths on the floor.")
    print("[NavExample]   Stop the timeline to pause.\n")
    omni.timeline.get_timeline_interface().play()


asyncio.ensure_future(run())
