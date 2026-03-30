"""
=============================================================================
  Isaac Sim 5.1 — Person + Forklift Simulation
=============================================================================

  WHAT YOU WILL SEE
  -----------------
  One person walks randomly around the entire warehouse floor using IRA's
  NavMesh system (guaranteed obstacle-free paths).

  Three forklifts patrol the same floor using a physics-based kinematic
  model that simulates real counterbalance forklift behaviour:
    - Rear-wheel-steered Ackermann/bicycle model (heading_rate = v·tan(δ)/L)
    - Smooth acceleration (1.5 m/s²) and braking (2.5 m/s²)
    - Speed automatically reduced in tight turns (3 m/s → 0.75 m/s at full lock)
    - Look-ahead braking: starts decelerating early enough to stop at waypoints
    - Fork mast animation: forks lower to pick up, raise to travel height
    - Shelf-aisle lane discipline: forklifts stay in aisles inside the rack area
    - Forklift-to-forklift separation to prevent overlapping

  HOW TO RUN
  ----------
  Isaac Sim  →  Window  →  Script Editor  →  Open this file  →  Ctrl+Enter

  HOW IT WORKS
  ------------
  1. IRA loads the warehouse scene and bakes the NavMesh.
  2. generate_random_commands() fills the person's command file with random
     NavMesh waypoints; the person walks them on loop.
  3. Forklifts are spawned as USD references after scene load.
  4. A physics step callback runs every frame and moves each forklift using
     the Ackermann kinematic model — no PhysX rigid bodies needed.

=============================================================================
"""

import asyncio
import math
import os
import random

import carb.eventdispatcher
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

NUM_FORKLIFTS  = 3      # how many forklifts to spawn
COMMAND_FRAMES = 600    # frames of person waypoints (30 fps, loops forever)

# ─────────────────────────────────────────────────────────────────────────────
#  STEP 1 — Enable IRA (Isaac Replicator Agent — drives the person)
# ─────────────────────────────────────────────────────────────────────────────
enable_extension("isaacsim.replicator.agent.core")

from isaacsim.replicator.agent.core.simulation import SimulationManager

# ─────────────────────────────────────────────────────────────────────────────
#  STEP 2 — Write IRA support files
#
#  CMD_PATH starts empty; it is filled with random NavMesh waypoints after
#  the NavMesh is baked (step 4).  The file must stay on disk for the whole
#  simulation because CharacterBehavior re-reads it on every animation loop.
# ─────────────────────────────────────────────────────────────────────────────
HERE           = os.path.dirname(os.path.abspath(__file__))
CMD_PATH       = os.path.join(HERE, "_person_commands.txt")
ROBOT_CMD_PATH = os.path.join(HERE, "_robot_commands.txt")
CFG_PATH       = os.path.join(HERE, "_ira_config.yaml")

with open(CMD_PATH,       "w") as f: f.write("# random commands will be written here\n")
with open(ROBOT_CMD_PATH, "w") as f: f.write("# no robots\n")

# ─────────────────────────────────────────────────────────────────────────────
#  STEP 3 — Write IRA config
# ─────────────────────────────────────────────────────────────────────────────
ira_config = {
    "isaacsim.replicator.agent": {
        "version": "0.7.0",
        "global": {
            "seed": 42,
            # simulation_length = how many frames of random commands to produce.
            # Commands loop automatically, so the person walks forever.
            "simulation_length": COMMAND_FRAMES,
        },
        "scene": {
            "asset_path": SCENE_USD,
        },
        "character": {
            "asset_path":      CHARACTERS_FOLDER,
            "command_file":    CMD_PATH,   # filled with NavMesh waypoints in step 4
            "filters":         [],
            "navigation_area": [],         # [] = use the entire NavMesh
            "spawn_area":      [],         # [] = spawn anywhere on the NavMesh
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
#  FORKLIFT — warehouse floor bounds
#  (measured corners of full_warehouse.usd)
# ─────────────────────────────────────────────────────────────────────────────

WALL_X_MIN, WALL_X_MAX = -26.33,  5.46
WALL_Y_MIN, WALL_Y_MAX = -23.40, 30.60
_MARGIN    = 1.8                          # keep forklifts this far from walls
NAV_X_MIN  = WALL_X_MIN + _MARGIN        # ~-24.5
NAV_X_MAX  = WALL_X_MAX - _MARGIN        # ~  3.7
NAV_Y_MIN  = WALL_Y_MIN + _MARGIN        # ~-21.6
NAV_Y_MAX  = WALL_Y_MAX - _MARGIN        # ~ 28.8

# ─────────────────────────────────────────────────────────────────────────────
#  FORKLIFT — physics / kinematic constants
# ─────────────────────────────────────────────────────────────────────────────

# Rear-wheel-steered bicycle model (same as a real counterbalance forklift)
FL_WHEELBASE   = 2.4    # m   — axle-to-axle distance
FL_MAX_SPEED   = 3.0    # m/s — peak travel speed (~11 km/h, typical warehouse)
FL_MIN_SPEED   = 0.4    # m/s — minimum creep speed
FL_ACCEL       = 1.5    # m/s² — maximum acceleration  (0 → 3 m/s in ~2 s)
FL_BRAKE       = 2.5    # m/s² — maximum braking decel (stronger than accel)
FL_MAX_STEER   = 65.0   # deg — rear-axle steer limit (forklifts steer very sharply)
FL_STEER_RATE  = 100.0  # deg/s — max rate of steering angle change
FL_HEADING_OFF = 90.0   # deg — USD model's forward axis is +Y, not +X
FL_WHEEL_R     = 0.35   # m   — wheel radius for spin animation
FL_ARRIVE      = 2.0    # m   — waypoint arrival radius
FL_BODY        = 0.9    # m   — half-width used for obstacle / shelf clearance
AISLE_SNAP     = 0.8    # m   — lateral tolerance before snapping to aisle centre

# Fork mast animation
FORK_RAISE_SPEED = 0.25  # m/s — vertical speed of the forks
FORK_TRAVEL_H    = 0.45  # m   — raised during travel (OSHA safe height)
FORK_GROUND_H    = 0.0   # m   — lowered at pickup / drop-off
IDLE_DURATION    = 3.5   # s   — pause at each waypoint (simulate pick/drop)

# State machine labels
STATE_DRIVE = "drive"
STATE_IDLE  = "idle"

# ─────────────────────────────────────────────────────────────────────────────
#  FORKLIFT — shelf geometry (populated lazily on first physics step)
#
#  The shelf scan runs once after the USD is fully resolved.  Until then
#  forklifts roam the open floor without aisle discipline — that is fine
#  because _SHELF_RECTS is empty and all shelf checks return False.
# ─────────────────────────────────────────────────────────────────────────────

_SHELF_RECTS    = []     # list of (min_x, max_x, min_y, max_y) tuples
_AISLE_XS       = []     # X centres of navigable aisle lanes
_SHELF_Y_MIN    = None
_SHELF_Y_MAX    = None
_shelves_ready  = False

_SHELF_KEYWORDS = {"rack", "shelf", "shelv", "pallet_rack", "shelving",
                   "storage", "fixture", "unit"}

# forklifts list — filled in run() after the scene loads
forklifts = []

# Keep the physx subscription alive for the entire simulation
_physx_sub = None

# ─────────────────────────────────────────────────────────────────────────────
#  FORKLIFT — navigation geometry helpers
# ─────────────────────────────────────────────────────────────────────────────

def _inside_shelf(x, y, margin=0.0):
    """Return True if (x, y) is inside any shelf bounding box (plus margin)."""
    for rx0, rx1, ry0, ry1 in _SHELF_RECTS:
        if rx0 - margin < x < rx1 + margin and ry0 - margin < y < ry1 + margin:
            return True
    return False

def _in_shelf_area(y):
    """Return True if y is within the Y extent of the shelving area."""
    return (_SHELF_Y_MIN is not None and
            _SHELF_Y_MIN - 1.0 < y < _SHELF_Y_MAX + 1.0)

def _nearest_aisle(x):
    """Return the X centre of the aisle lane closest to x."""
    return min(_AISLE_XS, key=lambda ax: abs(ax - x)) if _AISLE_XS else x

def _rand_floor_pt(prefer_aisle=False):
    """Random navigable waypoint on the warehouse floor."""
    if prefer_aisle and _AISLE_XS and _SHELF_Y_MIN is not None:
        # Pick a point inside a shelf aisle for full warehouse coverage
        return (random.choice(_AISLE_XS),
                random.uniform(_SHELF_Y_MIN + 1.0, _SHELF_Y_MAX - 1.0))
    for _ in range(30):
        x = random.uniform(NAV_X_MIN + 1.0, NAV_X_MAX - 1.0)
        y = random.uniform(NAV_Y_MIN + 1.0, NAV_Y_MAX - 1.0)
        if not _inside_shelf(x, y, margin=1.5):
            return (x, y)
    return (x, y)   # fallback: return last candidate even if inside a shelf

def _gen_patrol(n=8):
    """Generate n waypoints: mix of open-floor and shelf-aisle points."""
    return [_rand_floor_pt(prefer_aisle=(i % 3 == 0)) for i in range(n)]

# ─────────────────────────────────────────────────────────────────────────────
#  FORKLIFT — physics model helpers
# ─────────────────────────────────────────────────────────────────────────────

def _steer_speed_limit(steer_deg):
    """Max allowed speed for the current steer angle.

    Forklifts must slow down in tight turns to stay stable.
    At full lock (65°) the speed drops to 0.75 m/s.
    """
    ratio = abs(steer_deg) / FL_MAX_STEER   # 0.0 = straight, 1.0 = full lock
    return FL_MAX_SPEED * (1.0 - 0.75 * ratio)

def _braking_distance(speed):
    """Minimum distance needed to stop from the given speed under max braking."""
    return (speed * speed) / (2.0 * FL_BRAKE)

def _collect_wheel_prims(root_prim):
    """Recursively find all child prims whose name contains 'wheel'."""
    found = []
    for child in root_prim.GetAllChildren():
        if "wheel" in child.GetName().lower():
            found.append(child)
        found.extend(_collect_wheel_prims(child))
    return found

def _get_or_add_rotate_x(xform_obj):
    """Return the existing rotateX op, or add a new one."""
    for op in xform_obj.GetOrderedXformOps():
        if "rotateX" in op.GetOpName():
            return op
    return xform_obj.AddRotateXOp()

def _collect_fork_prims(root_prim):
    """Find fork/mast/carriage prims by name for height animation."""
    found = []
    for child in Usd.PrimRange(root_prim):
        name = child.GetName().lower()
        if any(k in name for k in ("fork", "mast", "lift", "carriage", "tine")):
            if child.IsA(UsdGeom.Xformable):
                found.append(child)
    return found

def _update_fork_height(fl, prim, step_dt):
    """Move forks smoothly toward their target height at FORK_RAISE_SPEED."""
    diff = fl["fork_target"] - fl["fork_height"]
    step = FORK_RAISE_SPEED * step_dt
    if abs(diff) <= step:
        fl["fork_height"] = fl["fork_target"]
    else:
        fl["fork_height"] += math.copysign(step, diff)

    # Lazy-collect fork prims the first time we animate
    if fl["fork_prims"] is None:
        fl["fork_prims"] = _collect_fork_prims(prim)

    for fp in fl["fork_prims"]:
        xf = UsdGeom.Xformable(fp)
        for op in xf.GetOrderedXformOps():
            if "translate" in op.GetOpName():
                t = op.Get()
                if t is not None:
                    op.Set(Gf.Vec3d(t[0], t[1], fl["fork_height"]))
                break

# ─────────────────────────────────────────────────────────────────────────────
#  FORKLIFT — per-frame movement (Ackermann bicycle kinematic model)
# ─────────────────────────────────────────────────────────────────────────────

def _move_forklift(fl, step_dt):
    stage = get_current_stage()
    prim  = stage.GetPrimAtPath(Sdf.Path(fl["path"]))
    if not prim.IsValid():
        return

    # ── STATE: IDLE — parked at waypoint, simulating pick / drop ────────────
    if fl["state"] == STATE_IDLE:
        fl["idle_timer"] -= step_dt
        # Lower forks for the first half of the pause, raise them for the second
        if fl["idle_timer"] > IDLE_DURATION * 0.5:
            fl["fork_target"] = FORK_GROUND_H   # lower to pallet
        else:
            fl["fork_target"] = FORK_TRAVEL_H   # raise for departure
        _update_fork_height(fl, prim, step_dt)
        if fl["idle_timer"] <= 0.0:
            # Resume driving — advance to next waypoint
            fl["state"]  = STATE_DRIVE
            fl["wp_idx"] = (fl["wp_idx"] + 1) % len(fl["waypoints"])
            if fl["wp_idx"] == 0:
                fl["waypoints"] = _gen_patrol()   # fresh route each lap
        return  # no movement while idling

    # ── STATE: DRIVE — heading toward the next waypoint ─────────────────────
    fx, fy = fl["pos"]
    wp = fl["waypoints"][fl["wp_idx"]]

    # Skip waypoints that land inside a shelf (unreachable)
    if _inside_shelf(wp[0], wp[1], margin=1.5):
        fl["wp_idx"] = (fl["wp_idx"] + 1) % len(fl["waypoints"])
        wp = fl["waypoints"][fl["wp_idx"]]

    dx, dy     = wp[0] - fx, wp[1] - fy
    dist_to_wp = math.hypot(dx, dy)

    # ── Arrival: switch to IDLE ──────────────────────────────────────────────
    if dist_to_wp < FL_ARRIVE:
        fl["speed"]      = 0.0
        fl["state"]      = STATE_IDLE
        fl["idle_timer"] = IDLE_DURATION
        fl["fork_target"] = FORK_GROUND_H
        return

    # ── Lane discipline in the shelving area ────────────────────────────────
    # Phase 1: steer toward the nearest aisle X first.
    # Phase 2: once in the lane, drive along it (Y toward waypoint, small X trim).
    if _in_shelf_area(fy) and _AISLE_XS:
        ax = _nearest_aisle(fx)
        if abs(fx - ax) > AISLE_SNAP:
            dx, dy = ax - fx, 0.0
        else:
            dx, dy = ax - fx, wp[1] - fy

    # ── Steering controller (proportional, rate-limited) ────────────────────
    desired_heading = math.degrees(math.atan2(dy, dx)) + FL_HEADING_OFF
    heading_err     = (desired_heading - fl["heading"] + 180.0) % 360.0 - 180.0

    # Proportional gain 0.8: snappy but not oscillating
    steer_target = max(-FL_MAX_STEER, min(FL_MAX_STEER, heading_err * 0.8))
    steer_diff   = steer_target - fl["steer_angle"]
    fl["steer_angle"] += max(-FL_STEER_RATE * step_dt,
                              min( FL_STEER_RATE * step_dt, steer_diff))

    # ── Speed control: acceleration + look-ahead braking ────────────────────
    max_speed  = _steer_speed_limit(fl["steer_angle"])
    brake_dist = _braking_distance(fl["speed"])

    if dist_to_wp < brake_dist + 0.5:
        # Start braking: target a speed that lets us stop in the remaining distance
        target_speed = max(FL_MIN_SPEED,
                           math.sqrt(max(0.0, 2.0 * FL_BRAKE * (dist_to_wp - 0.3))))
    else:
        target_speed = max_speed

    speed_diff = target_speed - fl["speed"]
    if speed_diff > 0:
        fl["speed"] = min(fl["speed"] + FL_ACCEL * step_dt, target_speed)
    else:
        fl["speed"] = max(fl["speed"] - FL_BRAKE * step_dt, target_speed)
    fl["speed"] = max(0.0, fl["speed"])

    # ── Bicycle / Ackermann kinematic update ────────────────────────────────
    # Real formula for a rear-wheel-steered vehicle:
    #   heading_rate = speed * tan(steer_angle) / wheelbase
    # This produces natural curved arcs — tight turns at low speed, wide
    # arcs when nearly straight, just like a real forklift.
    steer_rad    = math.radians(fl["steer_angle"])
    heading_rate = fl["speed"] * math.tan(steer_rad) / FL_WHEELBASE   # rad/s
    fl["heading"] += math.degrees(heading_rate) * step_dt

    move_rad = math.radians(fl["heading"] - FL_HEADING_OFF)
    nx = fx + fl["speed"] * step_dt * math.cos(move_rad)
    ny = fy + fl["speed"] * step_dt * math.sin(move_rad)

    # ── Clamp to navigable area ──────────────────────────────────────────────
    nx = max(NAV_X_MIN, min(NAV_X_MAX, nx))
    ny = max(NAV_Y_MIN, min(NAV_Y_MAX, ny))

    # ── Shelf push-out: hard-eject if the forklift clips a shelf rect ────────
    for rx0, rx1, ry0, ry1 in _SHELF_RECTS:
        ex0, ex1 = rx0 - FL_BODY, rx1 + FL_BODY
        ey0, ey1 = ry0 - FL_BODY, ry1 + FL_BODY
        if ex0 < nx < ex1 and ey0 < ny < ey1:
            dl, dr = nx - ex0, ex1 - nx
            db, dt = ny - ey0, ey1 - ny
            d_min  = min(dl, dr, db, dt)
            if   d_min == dl: nx = ex0
            elif d_min == dr: nx = ex1
            elif d_min == db: ny = ey0
            else:             ny = ey1
            fl["speed"] *= 0.4                                          # absorb impact
            fl["wp_idx"] = (fl["wp_idx"] + 1) % len(fl["waypoints"])  # skip blocked wp
            break

    # ── Forklift-to-forklift separation (~2 m safe gap) ─────────────────────
    SEP = FL_BODY * 2.2
    for other in forklifts:
        if other is fl:
            continue
        ox, oy = other["pos"]
        dist = math.hypot(nx - ox, ny - oy)
        if 0.001 < dist < SEP:
            push = SEP - dist
            nx += (nx - ox) / dist * push
            ny += (ny - oy) / dist * push
            nx = max(NAV_X_MIN, min(NAV_X_MAX, nx))
            ny = max(NAV_Y_MIN, min(NAV_Y_MAX, ny))
            fl["speed"] *= 0.7   # slow down when close to another forklift

    fl["pos"] = [nx, ny]
    fl["fork_target"] = FORK_TRAVEL_H   # keep forks raised while moving

    # ── Apply USD transform ──────────────────────────────────────────────────
    xform = UsdGeom.Xformable(prim)
    ops   = {op.GetOpName(): op for op in xform.GetOrderedXformOps()}
    t_op  = ops.get("xformOp:translate")
    if t_op:
        cur = t_op.Get()
        t_op.Set(Gf.Vec3d(nx, ny, cur[2]))
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

    # ── Fork height animation ────────────────────────────────────────────────
    _update_fork_height(fl, prim, step_dt)

# ─────────────────────────────────────────────────────────────────────────────
#  SHELF DETECTION — runs once on the first physics step
#
#  By the first physics step the USD reference is fully resolved, so all
#  shelf prims are visible.  We scan their bounding boxes once, compute the
#  aisle X centres, then set _shelves_ready = True to skip this forever after.
# ─────────────────────────────────────────────────────────────────────────────

def _init_shelf_rects():
    global _shelves_ready, _SHELF_Y_MIN, _SHELF_Y_MAX

    stage   = get_current_stage()
    wh_prim = stage.GetPrimAtPath(Sdf.Path("/World/Warehouse"))
    if not wh_prim.IsValid():
        _shelves_ready = True
        return

    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    seen = []   # (cx, cy) of already-registered rects — avoids duplicates

    for prim in Usd.PrimRange(wh_prim):
        if not any(k in prim.GetName().lower() for k in _SHELF_KEYWORDS):
            continue
        if not prim.IsA(UsdGeom.Xformable):
            continue
        try:
            rng = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
            mn, mx = rng.GetMin(), rng.GetMax()
            w, d, h = mx[0]-mn[0], mx[1]-mn[1], mx[2]-mn[2]
            if w < 0.5 or d < 0.5 or h < 0.5:
                continue
            rcx, rcy = (mn[0]+mx[0])/2, (mn[1]+mx[1])/2
            if any(abs(rcx-s[0]) < 0.5 and abs(rcy-s[1]) < 0.5 for s in seen):
                continue
            seen.append((rcx, rcy))
            _SHELF_RECTS.append((mn[0], mx[0], mn[1], mx[1]))
        except Exception:
            pass

    if _SHELF_RECTS:
        _SHELF_Y_MIN = min(r[2] for r in _SHELF_RECTS)
        _SHELF_Y_MAX = max(r[3] for r in _SHELF_RECTS)

        # Merge shelf X intervals → find gaps between them → aisle centres
        intervals = sorted((r[0], r[1]) for r in _SHELF_RECTS)
        merged = []
        for a, b in intervals:
            if merged and a < merged[-1][1]:
                merged[-1] = (merged[-1][0], max(merged[-1][1], b))
            else:
                merged.append((a, b))
        for i in range(len(merged) - 1):
            gx0, gx1 = merged[i][1], merged[i+1][0]
            if gx1 - gx0 > 1.0:
                _AISLE_XS.append((gx0 + gx1) / 2.0)
        # Also include outer wall corridors if wide enough
        if merged:
            if merged[0][0]  - NAV_X_MIN > 2.0:
                _AISLE_XS.append((NAV_X_MIN + merged[0][0]) / 2.0)
            if NAV_X_MAX - merged[-1][1] > 2.0:
                _AISLE_XS.append((merged[-1][1] + NAV_X_MAX) / 2.0)
        _AISLE_XS.sort()

    print(f"[NavExample] Shelves: {len(_SHELF_RECTS)} rects | "
          f"Aisles X: {[round(x, 1) for x in _AISLE_XS]}")
    _shelves_ready = True

# ─────────────────────────────────────────────────────────────────────────────
#  PHYSICS STEP CALLBACK — called every simulation frame
# ─────────────────────────────────────────────────────────────────────────────

def on_physics_step(step_dt):
    # First frame: detect shelves so forklifts know aisle positions
    if not _shelves_ready:
        _init_shelf_rects()
    # Move each forklift
    for fl in forklifts:
        _move_forklift(fl, step_dt)

# ─────────────────────────────────────────────────────────────────────────────
#  STEP 4 — Run the simulation
# ─────────────────────────────────────────────────────────────────────────────

async def run():
    global _physx_sub

    sim = SimulationManager()

    ok = sim.load_config_file(CFG_PATH)
    if not ok:
        print("[NavExample] ERROR: Config failed to load — check ASSETS_ROOT.")
        return

    # Wait for IRA to finish loading the scene + baking the NavMesh
    setup_done = asyncio.Event()
    _sub = sim.register_set_up_simulation_done_callback(lambda _: setup_done.set())

    print("[NavExample] Loading scene and baking NavMesh …")
    sim.set_up_simulation_from_config_file()
    await setup_done.wait()
    _sub = None

    # ── Spawn forklifts ──────────────────────────────────────────────────────
    print(f"[NavExample] Spawning {NUM_FORKLIFTS} forklifts …")
    stage = get_current_stage()
    cx    = (WALL_X_MIN + WALL_X_MAX) / 2.0   # warehouse X centre (~-10.4 m)
    random.seed(42)
    for fi, x_offset in enumerate([-7.0, 0.0, 7.0][:NUM_FORKLIFTS]):
        sx   = cx + x_offset
        sy   = NAV_Y_MIN + 2.0 + fi * 4.0     # stagger Y so they don't overlap
        path = f"/World/Forklifts/forklift_{fi}"

        # Create an Xform prim, position it, then attach the forklift USD
        xform = UsdGeom.Xform.Define(stage, path)
        xform.AddTranslateOp().Set(Gf.Vec3d(sx, sy, 0.0))
        xform.AddRotateZOp().Set(90.0)         # face along +Y (forward direction)
        stage.GetPrimAtPath(Sdf.Path(path)).GetReferences().AddReference(FORKLIFT_USD)

        forklifts.append({
            "path":        path,
            "pos":         [sx, sy],
            "heading":     90.0,           # degrees, matches initial rotateZ
            "speed":       0.0,            # m/s — starts at rest
            "steer_angle": 0.0,            # deg — starts pointing straight
            "waypoints":   _gen_patrol(),  # random route across the warehouse
            "wp_idx":      fi * 2,         # stagger starting waypoint per forklift
            "state":       STATE_DRIVE,
            "idle_timer":  0.0,
            "fork_height": 0.0,            # m — forks start on the ground
            "fork_target": FORK_TRAVEL_H,  # raise immediately on first move
            "wheel_angle": 0.0,
            "wheel_prims": None,           # found lazily on first physics step
            "fork_prims":  None,           # found lazily on first physics step
        })

    # ── Subscribe to physics — this drives the forklifts every frame ─────────
    # _physx_sub is module-level so it stays alive for the whole simulation.
    _physx_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(on_physics_step)

    # ── Generate random person waypoints from the live NavMesh ───────────────
    print("[NavExample] Generating random NavMesh waypoints for person …")
    random_commands = await sim.generate_random_commands()

    if not random_commands:
        print("[NavExample] WARNING: No random commands — NavMesh may be empty.")
        print("[NavExample]   The person will still spawn but won't move.")
    else:
        print(f"[NavExample] Generated {len(random_commands)} person waypoints.")

    sim.save_commands(random_commands)

    # ── Play ──────────────────────────────────────────────────────────────────
    print("[NavExample] Simulation running!")
    print("[NavExample]   Person walks random NavMesh routes.")
    print(f"[NavExample]   {NUM_FORKLIFTS} forklifts patrol with physics-based movement.")
    print("[NavExample]   Stop the timeline to pause.\n")
    omni.timeline.get_timeline_interface().play()


asyncio.ensure_future(run())
