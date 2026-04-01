"""
=============================================================================
  Forklift Pallet Shuttle Demo — Isaac Sim 5.1
=============================================================================

  One ForkliftC robot moves a single pallet:
    Loading Zone (yellow)  →  Staging Area (blue)  →  back  →  loop

  HOW TO RUN
  ----------
  Isaac Sim  →  Window  →  Script Editor  →  Open this file  →  Ctrl+Enter

=============================================================================
"""

import asyncio
import math

import omni.graph.core as og
import omni.kit.app
import omni.timeline
import omni.usd
import usdrt.Sdf
from isaacsim.core.api import World
from isaacsim.core.utils.prims import delete_prim, get_prim_at_path
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import Gf, Sdf, UsdGeom, UsdPhysics

# ─────────────────────────────────────────────────────────────────────────────
#  Asset paths
# ─────────────────────────────────────────────────────────────────────────────
ASSETS       = "/home/ubuntu/isaacsim_assets/Assets/Isaac/5.1"
FORKLIFT_USD = f"{ASSETS}/Isaac/Robots/IsaacSim/ForkliftC/forklift_c.usd"
PALLET_USD   = f"{ASSETS}/Isaac/Props/Pallet/pallet.usd"
GROUND_USD   = f"{ASSETS}/Isaac/Environments/Grid/default_environment.usd"

GRAPH = "/ForkliftGraph"

# ─────────────────────────────────────────────────────────────────────────────
#  Layout — both zones at the same X, separated in Y
#
#  Loading Zone (yellow):  centre (8, 3),  pallet at (8, 3)
#  Staging Area (blue):    centre (8, -4), pallet goes to (8, -4)
#  Forklift starts at origin (0, 0) facing +X
# ─────────────────────────────────────────────────────────────────────────────
LOADING_POS  = Gf.Vec3d(8.0, 3.0, 0.0)
STAGING_POS  = Gf.Vec3d(8.0, -4.0, 0.0)


async def main():
    # ── Clean up any previous run ───────────────────────────────────────────
    for path in ["/World", GRAPH]:
        if get_prim_at_path(path):
            delete_prim(path)
    await omni.kit.app.get_app().next_update_async()

    # ── Create World ────────────────────────────────────────────────────────
    world = World(stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()

    # ── Ground plane ────────────────────────────────────────────────────────
    add_reference_to_stage(GROUND_USD, "/World/Ground")

    # ── Zone marker bands ───────────────────────────────────────────────────
    def create_zone(name, cx, cy, w, d, color):
        STRIP_W, STRIP_H = 0.15, 0.012
        strips = [
            ("North", cx,       cy + d/2, w/2,      STRIP_W/2),
            ("South", cx,       cy - d/2, w/2,      STRIP_W/2),
            ("East",  cx + w/2, cy,       STRIP_W/2, d/2),
            ("West",  cx - w/2, cy,       STRIP_W/2, d/2),
        ]
        for sname, tx, ty, sx, sy in strips:
            cube = UsdGeom.Cube.Define(stage, Sdf.Path(f"/World/{name}/Strip_{sname}"))
            xf   = UsdGeom.Xformable(cube.GetPrim())
            xf.AddTranslateOp().Set(Gf.Vec3d(tx, ty, STRIP_H))
            xf.AddScaleOp().Set(Gf.Vec3f(sx, sy, STRIP_H))
            cube.GetDisplayColorAttr().Set(color)

    YELLOW = [Gf.Vec3f(1.0, 0.85, 0.0)]
    BLUE   = [Gf.Vec3f(0.3, 0.8,  1.0)]
    create_zone("LoadingZone", 8.0,  3.0, 5.0, 4.0, YELLOW)
    create_zone("StagingArea", 8.0, -4.0, 5.0, 4.0, BLUE)

    # ── Forklift at origin ─────────────────────────────────────────────────
    add_reference_to_stage(FORKLIFT_USD, "/World/Forklift")

    # ── One pallet in the Loading Zone ─────────────────────────────────────
    add_reference_to_stage(PALLET_USD, "/World/Pallet")
    pallet_prim = stage.GetPrimAtPath(Sdf.Path("/World/Pallet"))
    pxf = UsdGeom.Xformable(pallet_prim)
    pxf.AddTranslateOp().Set(Gf.Vec3d(LOADING_POS[0], LOADING_POS[1], 0.15))
    pxf.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, 90.0))
    UsdPhysics.RigidBodyAPI.Apply(pallet_prim)
    UsdPhysics.CollisionAPI.Apply(pallet_prim)
    UsdPhysics.MeshCollisionAPI.Apply(pallet_prim).CreateApproximationAttr().Set("convexHull")
    UsdPhysics.MassAPI.Apply(pallet_prim).CreateMassAttr(20.0)

    await omni.kit.app.get_app().next_update_async()

    # ── Initialize world FIRST (resets stage context) ──────────────────────
    omni.timeline.get_timeline_interface().set_time_codes_per_second(60)
    await world.initialize_simulation_context_async()
    await omni.kit.app.get_app().next_update_async()

    # ── OmniGraph controller (AFTER world init) ───────────────────────────
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": GRAPH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("Tick",        "omni.graph.action.OnPlaybackTick"),
                ("Articulation","isaacsim.core.nodes.IsaacArticulationController"),
                ("WriteSteerL", "omni.graph.nodes.WritePrimAttribute"),
                ("WriteSteerR", "omni.graph.nodes.WritePrimAttribute"),
                ("WriteLift",   "omni.graph.nodes.WritePrimAttribute"),
                ("Steer",       "omni.graph.nodes.ConstantDouble"),
                ("Lift",        "omni.graph.nodes.ConstantDouble"),
            ],
            keys.CONNECT: [
                ("Tick.outputs:tick", "Articulation.inputs:execIn"),
                ("Tick.outputs:tick", "WriteSteerL.inputs:execIn"),
                ("Tick.outputs:tick", "WriteSteerR.inputs:execIn"),
                ("Tick.outputs:tick", "WriteLift.inputs:execIn"),
                ("Steer.inputs:value", "WriteSteerL.inputs:value"),
                ("Steer.inputs:value", "WriteSteerR.inputs:value"),
                ("Lift.inputs:value",  "WriteLift.inputs:value"),
            ],
            keys.SET_VALUES: [
                ("Steer.inputs:value", 0.0),
                ("Lift.inputs:value",  0.0),
                ("WriteSteerL.inputs:prim",
                    [usdrt.Sdf.Path("/World/Forklift/left_rotator_joint")]),
                ("WriteSteerL.inputs:name",
                    "drive:angular:physics:targetPosition"),
                ("WriteSteerR.inputs:prim",
                    [usdrt.Sdf.Path("/World/Forklift/right_rotator_joint")]),
                ("WriteSteerR.inputs:name",
                    "drive:angular:physics:targetPosition"),
                ("WriteLift.inputs:prim",
                    [usdrt.Sdf.Path("/World/Forklift/lift_joint")]),
                ("WriteLift.inputs:name",
                    "drive:linear:physics:targetPosition"),
                ("Articulation.inputs:robotPath",
                    "/World/Forklift"),
                ("Articulation.inputs:velocityCommand",
                    [0.0, 0.0]),
                ("Articulation.inputs:jointNames",
                    ["left_back_wheel_joint", "right_back_wheel_joint"]),
            ],
        },
    )
    await omni.kit.app.get_app().next_update_async()

    # ── Control helpers ──────────────────────────────────────────────────────
    def wheels(speed):
        og.Controller.attribute(f"{GRAPH}/Articulation.inputs:velocityCommand").set([speed, speed])

    def steer(angle_deg):
        og.Controller.attribute(f"{GRAPH}/Steer.inputs:value").set(angle_deg)

    def lift(height):
        og.Controller.attribute(f"{GRAPH}/Lift.inputs:value").set(height)

    async def wait(frames):
        for _ in range(frames):
            await omni.kit.app.get_app().next_update_async()

    # ── Position utilities ───────────────────────────────────────────────────
    forklift_prim = stage.GetPrimAtPath(Sdf.Path("/World/Forklift"))

    def get_forklift_pos():
        mat = UsdGeom.XformCache().GetLocalToWorldTransform(forklift_prim)
        t   = mat.ExtractTranslation()
        return Gf.Vec3d(t[0], t[1], 0.0)

    def _current_heading():
        mat = UsdGeom.XformCache().GetLocalToWorldTransform(forklift_prim)
        fwd = mat.GetRow3(0)
        return math.degrees(math.atan2(fwd[1], fwd[0]))

    def set_prim_position(prim, pos):
        for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
            if "translate" in op.GetOpName():
                op.Set(pos)
                return

    # ── Constants ────────────────────────────────────────────────────────────
    CARRY_OFFSET_X  = 1.5    # fork-tip distance ahead of body
    CARRY_OFFSET_Z  = 0.30   # pallet height during carry
    MAX_STEER       = 22.0
    KP_STEER        = 0.5
    MIN_TURN_SPEED  = 0.5    # MUST be > 0 — ForkliftC cannot turn in place!
    WAYPOINT_DIST   = 1.0    # how close to target before "arrived"

    # ── Navigation ───────────────────────────────────────────────────────────
    async def drive_to(target, cargo=None):
        """
        Drive to XY target. Forklift always moves forward (MIN_TURN_SPEED)
        even during sharp turns, because the ForkliftC physically cannot
        rotate without wheel movement.
        """
        while True:
            pos  = get_forklift_pos()
            dx   = target[0] - pos[0]
            dy   = target[1] - pos[1]
            dist = math.sqrt(dx*dx + dy*dy)
            if dist < WAYPOINT_DIST:
                break

            desired = math.degrees(math.atan2(dy, dx))
            err     = (desired - _current_heading() + 180) % 360 - 180
            sa      = max(-MAX_STEER, min(MAX_STEER, KP_STEER * err))
            abs_err = abs(err)

            # Always keep minimum speed so steering actually works
            if abs_err > 90:
                speed = MIN_TURN_SPEED
            elif abs_err > 45:
                speed = 1.5
            elif abs_err > 15:
                speed = 2.5
            else:
                speed = 4.0

            wheels(speed)
            steer(sa)

            if cargo:
                _update_carried_pallet(cargo)
            await omni.kit.app.get_app().next_update_async()

        wheels(0.0)
        steer(0.0)

    def _update_carried_pallet(cargo_prim):
        mat = UsdGeom.XformCache().GetLocalToWorldTransform(forklift_prim)
        fwd = mat.GetRow3(0)
        t   = mat.ExtractTranslation()
        set_prim_position(cargo_prim, Gf.Vec3d(
            t[0] + fwd[0] * CARRY_OFFSET_X,
            t[1] + fwd[1] * CARRY_OFFSET_X,
            CARRY_OFFSET_Z,
        ))

    # ── Pickup: drive to pallet zone, creep under pallet, lift ───────────────
    async def pickup_at(zone_pos):
        """Drive to a zone, slide forks under the pallet, and lift it."""
        # Approach: stop 3 m west of pallet so we're lined up
        approach = Gf.Vec3d(zone_pos[0] - 3.0, zone_pos[1], 0.0)
        print(f"[Shuttle]   driving to approach ({approach[0]:.1f}, {approach[1]:.1f})")
        await drive_to(approach)

        # Lower forks before insertion
        print("[Shuttle]   lowering forks")
        lift(0.0)
        steer(0.0)
        await wait(60)

        # Creep east until fork tips reach the pallet (body at zone_x - offset)
        target_body_x = zone_pos[0] - CARRY_OFFSET_X
        print(f"[Shuttle]   creeping east to body X={target_body_x:.1f}")
        while True:
            pos = get_forklift_pos()
            if pos[0] >= target_body_x:
                break
            wheels(1.5)
            steer(0.0)
            await omni.kit.app.get_app().next_update_async()
        wheels(0.0)
        await wait(30)

        # Lift
        print("[Shuttle]   lifting pallet")
        lift(0.15)
        await wait(120)

        # Make pallet kinematic so it follows the forklift
        UsdPhysics.RigidBodyAPI(pallet_prim).GetKinematicEnabledAttr().Set(True)
        print("[Shuttle]   pallet attached (kinematic)")

    # ── Drop: drive to zone, creep in, lower pallet, release, reverse ────────
    async def drop_at(zone_pos):
        """Carry the pallet to a zone, lower it, and back away."""
        # Approach: stop 3 m west of drop zone
        approach = Gf.Vec3d(zone_pos[0] - 3.0, zone_pos[1], 0.0)
        print(f"[Shuttle]   carrying to approach ({approach[0]:.1f}, {approach[1]:.1f})")
        await drive_to(approach, cargo=pallet_prim)

        # Creep east to deposit position
        target_body_x = zone_pos[0] - CARRY_OFFSET_X
        print(f"[Shuttle]   creeping east to body X={target_body_x:.1f}")
        while True:
            pos = get_forklift_pos()
            if pos[0] >= target_body_x:
                break
            wheels(1.5)
            steer(0.0)
            _update_carried_pallet(pallet_prim)
            await omni.kit.app.get_app().next_update_async()
        wheels(0.0)
        await wait(30)

        # Lower pallet
        print("[Shuttle]   lowering pallet")
        lift(0.0)
        await wait(120)

        # Snap pallet to zone position and release physics
        set_prim_position(pallet_prim, Gf.Vec3d(zone_pos[0], zone_pos[1], 0.15))
        UsdPhysics.RigidBodyAPI(pallet_prim).GetKinematicEnabledAttr().Set(False)
        print("[Shuttle]   pallet released")
        await wait(60)

        # Reverse away (3 m west)
        print("[Shuttle]   reversing away")
        wheels(-2.0)
        steer(0.0)
        await wait(120)
        wheels(0.0)
        await wait(30)

    # ── Start simulation ──────────────────────────────────────────────────────
    world.play()
    await wait(60)   # let physics settle pallets onto floor

    # ── Shuttle loop ──────────────────────────────────────────────────────────
    print("[Shuttle] === Starting: 1 pallet, Loading ↔ Staging ===")

    while True:
        # Loading → Staging
        print("\n[Shuttle] ── Loading Zone → Staging Area ──")
        await pickup_at(LOADING_POS)
        await drop_at(STAGING_POS)

        # Staging → Loading
        print("\n[Shuttle] ── Staging Area → Loading Zone ──")
        await pickup_at(STAGING_POS)
        await drop_at(LOADING_POS)


asyncio.ensure_future(main())
