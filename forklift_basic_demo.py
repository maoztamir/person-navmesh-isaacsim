"""
=============================================================================
  Forklift Pallet Shuttle Demo — Isaac Sim 5.1
=============================================================================

  A physics-based ForkliftC robot shuttles 4 pallets between two zones:

    Loading Zone  (yellow markers)  ←→  Staging Area  (blue markers)

  Sequence:
    1. Forklift picks up each pallet from the Loading Zone one at a time
    2. Carries it to the Staging Area and sets it down
    3. Once all pallets are in Staging, reverses direction back to Loading
    4. Loops forever

  Pallet carry uses kinematic attachment:
    - Pallet RigidBody is set kinematic while being transported
    - Position is updated every frame to follow the forklift's fork tips
    - On drop, kinematic is released and physics takes over

  HOW TO RUN
  ----------
  Isaac Sim  →  Window  →  Script Editor  →  Open this file  →  Ctrl+Enter

=============================================================================
"""

import asyncio
import math
import random

import carb
import omni.anim.navigation.core as nav
import omni.graph.core as og
import omni.kit.app
import omni.kit.commands
import omni.timeline
import omni.usd
import usdrt.Sdf
import NavSchema
from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.prims import delete_prim, get_prim_at_path
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import Gf, Sdf, UsdGeom, UsdPhysics

# Enable NavMesh extension
enable_extension("omni.anim.navigation.core")

# ─────────────────────────────────────────────────────────────────────────────
#  Asset paths
# ─────────────────────────────────────────────────────────────────────────────
ASSETS       = "/home/ubuntu/isaacsim_assets/Assets/Isaac/5.1"
FORKLIFT_USD = f"{ASSETS}/Isaac/Robots/IsaacSim/ForkliftC/forklift_c.usd"
PALLET_USD   = f"{ASSETS}/Isaac/Props/Pallet/pallet.usd"
GROUND_USD   = f"{ASSETS}/Isaac/Environments/Grid/default_environment.usd"

GRAPH = "/ForkliftGraph"


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

    # ── NavMesh volume (20×20 m, centred at (5, 0)) ─────────────────────────
    nav_vol = NavSchema.NavMeshVolume.Define(stage, Sdf.Path("/World/NavMeshVolume"))
    nav_vol.GetNavVolumeTypeAttr().Set("Include")
    UsdGeom.Boundable(nav_vol.GetPrim()).GetExtentAttr().Set(
        [Gf.Vec3f(-10.0, -10.0, -0.1), Gf.Vec3f(10.0, 10.0, 4.0)]
    )
    UsdGeom.Xformable(nav_vol.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(5.0, 0.0, 0.0))

    # ── Zone marker bands ───────────────────────────────────────────────────
    YELLOW = [Gf.Vec3f(1.0, 0.85, 0.0)]
    BLUE   = [Gf.Vec3f(0.3, 0.8,  1.0)]

    def create_zone(name, cx, cy, w, d, color):
        """Draw a rectangular marker band on the floor (visual only, no collision)."""
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

    create_zone("LoadingZone", 8.0,  3.0, 5.0, 4.0, YELLOW)
    create_zone("StagingArea", 8.0, -4.0, 5.0, 4.0, BLUE)

    # ── Forklift ────────────────────────────────────────────────────────────
    add_reference_to_stage(FORKLIFT_USD, "/World/Forklift")

    # ── Spawn 4 pallets in the Loading Zone ─────────────────────────────────
    LOADING_SLOTS = [Gf.Vec3d(7.5, y, 0.0) for y in [1.5, 2.5, 3.5, 4.5]]
    STAGING_SLOTS = [Gf.Vec3d(7.5, y, 0.0) for y in [-5.5, -4.5, -3.5, -2.5]]

    def spawn_pallets(slots):
        prims = []
        for i, slot in enumerate(slots):
            path = Sdf.Path(f"/World/Pallet_{i}")
            add_reference_to_stage(PALLET_USD, str(path))
            prim = stage.GetPrimAtPath(path)
            xf   = UsdGeom.Xformable(prim)
            xf.AddTranslateOp().Set(Gf.Vec3d(slot[0], slot[1], 0.20))
            xf.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, 90.0))
            UsdPhysics.RigidBodyAPI.Apply(prim)
            UsdPhysics.CollisionAPI.Apply(prim)
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr().Set("convexHull")
            UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(20.0)
            prims.append(prim)
        return prims

    pallet_prims = spawn_pallets(LOADING_SLOTS)

    await omni.kit.app.get_app().next_update_async()

    # ── OmniGraph controller ────────────────────────────────────────────────
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

    # ── Initialize simulation ───────────────────────────────────────────────
    omni.timeline.get_timeline_interface().set_time_codes_per_second(60)
    await world.initialize_simulation_context_async()

    # ── Bake NavMesh ────────────────────────────────────────────────────────
    print("[Shuttle] Baking NavMesh …")
    inav = nav.acquire_interface()
    inav.start_navmesh_baking()
    for _ in range(1800):
        if inav.get_navmesh() is not None:
            print("[Shuttle] NavMesh ready. (Window → Navigation → NavMesh → Show NavMesh to visualise)")
            break
        await omni.kit.app.get_app().next_update_async()
    else:
        print("[Shuttle] WARNING: NavMesh bake timed out.")

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

    def get_prim_position(prim):
        mat = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
        t   = mat.ExtractTranslation()
        return Gf.Vec3d(t[0], t[1], t[2])

    def set_prim_position(prim, pos):
        xf  = UsdGeom.Xformable(prim)
        for op in xf.GetOrderedXformOps():
            if "translate" in op.GetOpName():
                op.Set(pos)
                return

    # ── Shuttle constants ────────────────────────────────────────────────────
    SLOT_X         = 7.5
    APPROACH_X     = 5.0     # park here, face +X, then creep east to SLOT_X
    REVERSE_X      = 4.0     # back up to here after depositing
    FORK_LOW       = 0.0
    FORK_CARRY     = 0.15    # just clears the floor
    CREEP_SPEED    = 2.0     # m/s for fine legs
    CARRY_SPEED    = 3.0     # m/s while carrying
    CARRY_OFFSET_X = 1.5     # metres ahead of forklift body (fork tips)
    CARRY_OFFSET_Z = 0.30    # metres up during transport
    FINE_STOP_DIST = 0.25    # tight arrival threshold for straight-line legs
    WAYPOINT_DIST  = 2.0     # coarse arrival threshold for NavMesh legs
    MAX_STEER      = 22.0
    KP_STEER       = 0.4

    # ── Carry attachment ─────────────────────────────────────────────────────
    def _update_carried_pallet(pallet_prim):
        """Move the kinematic pallet to the forklift's fork-tip position."""
        mat = UsdGeom.XformCache().GetLocalToWorldTransform(forklift_prim)
        fwd = mat.GetRow3(0)
        t   = mat.ExtractTranslation()
        set_prim_position(pallet_prim, Gf.Vec3d(
            t[0] + fwd[0] * CARRY_OFFSET_X,
            t[1] + fwd[1] * CARRY_OFFSET_X,
            CARRY_OFFSET_Z,
        ))

    # ── Heading helpers ──────────────────────────────────────────────────────
    def _current_heading():
        mat = UsdGeom.XformCache().GetLocalToWorldTransform(forklift_prim)
        fwd = mat.GetRow3(0)
        return math.degrees(math.atan2(fwd[1], fwd[0]))

    async def align_to_x_axis(tolerance_deg=5.0):
        """Steer in place until forklift faces +X (heading ≈ 0°)."""
        while True:
            err = (-_current_heading() + 180) % 360 - 180
            if abs(err) < tolerance_deg:
                wheels(0.0)
                steer(0.0)
                break
            steer(max(-MAX_STEER, min(MAX_STEER, KP_STEER * err)))
            wheels(0.0)
            await omni.kit.app.get_app().next_update_async()

    # ── Navigation helpers ───────────────────────────────────────────────────
    async def drive_to_waypoint(target, cargo=None):
        """
        Drive to an XY target using proportional heading control.
        If cargo prim is supplied, its kinematic position is updated each frame.
        """
        while True:
            pos = get_forklift_pos()
            dx  = target[0] - pos[0]
            dy  = target[1] - pos[1]
            dist = math.sqrt(dx*dx + dy*dy)
            if dist < WAYPOINT_DIST:
                break

            desired  = math.degrees(math.atan2(dy, dx))
            err      = (desired - _current_heading() + 180) % 360 - 180
            sa       = max(-MAX_STEER, min(MAX_STEER, KP_STEER * err))
            abs_err  = abs(err)
            speed    = (0.0 if abs_err > 120 else
                        1.5 if abs_err > 60  else
                        2.5 if abs_err > 30  else
                        CARRY_SPEED if cargo else 4.0)
            wheels(speed)
            steer(sa)

            if cargo:
                _update_carried_pallet(cargo)
            await omni.kit.app.get_app().next_update_async()

        wheels(0.0)
        steer(0.0)

    # ── Pickup / drop sequences ──────────────────────────────────────────────
    async def approach_and_pickup(pallet_prim, slot_pos):
        """Navigate to slot, slide forks under pallet, lift, go kinematic."""
        slot_y = slot_pos[1]

        print(f"[Shuttle]   → approach ({APPROACH_X:.1f}, {slot_y:.1f})")
        await drive_to_waypoint(Gf.Vec3d(APPROACH_X, slot_y, 0.0))

        print("[Shuttle]   → aligning to +X")
        await align_to_x_axis(tolerance_deg=3.0)

        print("[Shuttle]   → lowering forks")
        lift(FORK_LOW)
        await wait(60)

        print("[Shuttle]   → creeping under pallet")
        while True:
            pos = get_forklift_pos()
            if abs(pos[0] - SLOT_X) < FINE_STOP_DIST:
                break
            wheels(CREEP_SPEED)
            steer(0.0)
            await omni.kit.app.get_app().next_update_async()
        wheels(0.0)
        await wait(30)

        print("[Shuttle]   → lifting")
        lift(FORK_CARRY)
        await wait(90)

        UsdPhysics.RigidBodyAPI(pallet_prim).GetKinematicEnabledAttr().Set(True)

    async def carry_and_drop(pallet_prim, dest_slot_pos):
        """Drive to destination carrying the pallet, deposit, and reverse."""
        dest_y = dest_slot_pos[1]

        print(f"[Shuttle]   → carrying to approach ({APPROACH_X:.1f}, {dest_y:.1f})")
        await drive_to_waypoint(Gf.Vec3d(APPROACH_X, dest_y, 0.0), cargo=pallet_prim)

        print("[Shuttle]   → aligning to +X at destination")
        await align_to_x_axis(tolerance_deg=3.0)

        print("[Shuttle]   → creeping to deposit position")
        while True:
            pos = get_forklift_pos()
            if abs(pos[0] - SLOT_X) < FINE_STOP_DIST:
                break
            wheels(CREEP_SPEED)
            steer(0.0)
            _update_carried_pallet(pallet_prim)
            await omni.kit.app.get_app().next_update_async()
        wheels(0.0)
        await wait(30)

        print("[Shuttle]   → lowering pallet")
        lift(FORK_LOW)
        await wait(90)

        # Snap pallet to slot, release physics
        set_prim_position(pallet_prim,
            Gf.Vec3d(dest_slot_pos[0], dest_slot_pos[1], 0.20))
        UsdPhysics.RigidBodyAPI(pallet_prim).GetKinematicEnabledAttr().Set(False)
        await wait(60)

        print("[Shuttle]   → reversing away")
        while True:
            pos = get_forklift_pos()
            if pos[0] < REVERSE_X:
                break
            wheels(-CREEP_SPEED)
            steer(0.0)
            await omni.kit.app.get_app().next_update_async()
        wheels(0.0)
        await wait(30)

    # ── Start ────────────────────────────────────────────────────────────────
    world.play()
    await wait(30)   # let physics settle

    # ── Shuttle state machine ────────────────────────────────────────────────
    loading_pallets = list(range(4))   # pallet indices currently in Loading Zone
    staging_pallets = []               # pallet indices currently in Staging Area

    print("[Shuttle] Starting pallet shuttle. Loading Zone → Staging Area.")

    while True:
        if loading_pallets:
            idx       = loading_pallets.pop(0)
            src_slot  = LOADING_SLOTS[idx]
            dest_slot = STAGING_SLOTS[idx]
            print(f"[Shuttle] Pallet {idx}: Loading({src_slot[1]:.1f}) → Staging({dest_slot[1]:.1f})")
            await approach_and_pickup(pallet_prims[idx], src_slot)
            await carry_and_drop(pallet_prims[idx], dest_slot)
            staging_pallets.append(idx)

        elif staging_pallets:
            print("[Shuttle] All pallets in Staging — reversing direction.")
            idx       = staging_pallets.pop(0)
            src_slot  = STAGING_SLOTS[idx]
            dest_slot = LOADING_SLOTS[idx]
            print(f"[Shuttle] Pallet {idx}: Staging({src_slot[1]:.1f}) → Loading({dest_slot[1]:.1f})")
            await approach_and_pickup(pallet_prims[idx], src_slot)
            await carry_and_drop(pallet_prims[idx], dest_slot)
            loading_pallets.append(idx)

        else:
            # Both lists empty simultaneously — shouldn't happen, but guard anyway
            await wait(60)


asyncio.ensure_future(main())
