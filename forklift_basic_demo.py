"""
=============================================================================
  Forklift Pallet Demo — Isaac Sim 5.1
=============================================================================

  A physics-based ForkliftC robot:
    1. Drives forward to a pallet                  (wheel joints)
    2. Slides forks underneath and lifts the pallet (lift joint)
    3. Drives the pallet to a new location          (wheel joints)
    4. Lowers and places the pallet on the ground   (lift joint)
    5. Reverses away                                (wheel joints)

  Uses the articulated ForkliftC robot with real physics joints:
    - left/right_back_wheel_joint  → drive velocity
    - left/right_rotator_joint     → steering angle
    - lift_joint                   → fork height

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
#  Asset paths — ForkliftC is the articulated robot version
# ─────────────────────────────────────────────────────────────────────────────
ASSETS       = "/home/ubuntu/isaacsim_assets/Assets/Isaac/5.1"
FORKLIFT_USD = f"{ASSETS}/Isaac/Robots/IsaacSim/ForkliftC/forklift_c.usd"
PALLET_USD   = f"{ASSETS}/Isaac/Props/Pallet/pallet.usd"
GROUND_USD   = f"{ASSETS}/Isaac/Environments/Grid/default_environment.usd"

GRAPH = "/ForkliftGraph"   # OmniGraph controller path


async def main():
    # ── Clean up any previous run ───────────────────────────────────────────
    for path in ["/World", GRAPH]:
        if get_prim_at_path(path):
            delete_prim(path)
    await omni.kit.app.get_app().next_update_async()

    # ── Create World ────────────────────────────────────────────────────────
    world = World(stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()

    # ── Load assets ─────────────────────────────────────────────────────────
    # Ground plane — needed for physics (gravity, floor collision)
    add_reference_to_stage(GROUND_USD, "/World/Ground")

    # ── NavMesh volume ───────────────────────────────────────────────────────
    # Define a 20×20 m navigable area centred at the origin (Z up).
    # NavSchema.NavMeshVolume marks the region the baker will treat as walkable.
    nav_vol = NavSchema.NavMeshVolume.Define(stage, Sdf.Path("/World/NavMeshVolume"))
    nav_vol.GetNavVolumeTypeAttr().Set("Include")
    nav_boundable = UsdGeom.Boundable(nav_vol.GetPrim())
    # Extent is in local units: half-extents for a 20×20×4 m box
    nav_boundable.GetExtentAttr().Set(
        [Gf.Vec3f(-10.0, -10.0, -0.1), Gf.Vec3f(10.0, 10.0, 4.0)]
    )
    UsdGeom.Xformable(nav_vol.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(5.0, 0.0, 0.0))

    # ── Loading area — yellow marker band on the floor ───────────────────────
    # 5 m × 4 m zone inside the NavMesh, centred at (8, 3).
    # Four thin flat strips form the border; no physics so forklifts can enter.
    ZONE_CX, ZONE_CY = 8.0, 3.0   # zone centre (world XY)
    ZONE_W,  ZONE_D  = 5.0, 4.0   # zone width (X) × depth (Y)
    STRIP_W          = 0.15        # band width in metres
    STRIP_H          = 0.012       # band thickness (sits on floor)
    YELLOW           = [Gf.Vec3f(1.0, 0.85, 0.0)]

    _zone_strips = [
        # name       centre-X                    centre-Y                    scale-X         scale-Y
        ("North", ZONE_CX,                   ZONE_CY + ZONE_D / 2,   ZONE_W / 2,     STRIP_W / 2),
        ("South", ZONE_CX,                   ZONE_CY - ZONE_D / 2,   ZONE_W / 2,     STRIP_W / 2),
        ("East",  ZONE_CX + ZONE_W / 2,      ZONE_CY,                 STRIP_W / 2,    ZONE_D / 2),
        ("West",  ZONE_CX - ZONE_W / 2,      ZONE_CY,                 STRIP_W / 2,    ZONE_D / 2),
    ]
    for name, tx, ty, sx, sy in _zone_strips:
        cube = UsdGeom.Cube.Define(stage, Sdf.Path(f"/World/LoadingZone/Strip_{name}"))
        xf = UsdGeom.Xformable(cube.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d(tx, ty, STRIP_H))
        xf.AddScaleOp().Set(Gf.Vec3f(sx, sy, STRIP_H))   # Cube is unit [-1,1] so scale = half-size
        cube.GetDisplayColorAttr().Set(YELLOW)

    # Articulated forklift — spawns at the origin, facing +X
    add_reference_to_stage(FORKLIFT_USD, "/World/Forklift")

    # Pallet — placed 4 m ahead of the forklift in the +X direction
    add_reference_to_stage(PALLET_USD, "/World/Pallet")
    pallet_prim = stage.GetPrimAtPath(Sdf.Path("/World/Pallet"))
    xformable = UsdGeom.Xformable(pallet_prim)
    xformable.AddTranslateOp().Set(Gf.Vec3d(4.0, 0.0, 0.2))
    # Rotate 90° around Z so the pallet's long side faces the forklift (+X)
    xformable.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, 90.0))

    # Give the pallet rigid-body physics so the forks can push it up
    UsdPhysics.RigidBodyAPI.Apply(pallet_prim)
    UsdPhysics.CollisionAPI.Apply(pallet_prim)
    # Explicitly set convex hull so PhysX doesn't complain about triangle mesh on a dynamic body
    UsdPhysics.MeshCollisionAPI.Apply(pallet_prim).CreateApproximationAttr().Set("convexHull")
    mass = UsdPhysics.MassAPI.Apply(pallet_prim)
    mass.CreateMassAttr(20.0)   # 20 kg

    await omni.kit.app.get_app().next_update_async()

    # ── OmniGraph controller ────────────────────────────────────────────────
    # This is the same pattern NVIDIA uses in their official forklift test.
    # The graph runs every simulation tick and writes joint commands:
    #   - Wheel velocity via ArticulationController
    #   - Steering angle via WritePrimAttribute on rotator joints
    #   - Lift height  via WritePrimAttribute on lift joint
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
                # Steering joints — control the turning angle
                ("WriteSteerL.inputs:prim",
                    [usdrt.Sdf.Path("/World/Forklift/left_rotator_joint")]),
                ("WriteSteerL.inputs:name",
                    "drive:angular:physics:targetPosition"),
                ("WriteSteerR.inputs:prim",
                    [usdrt.Sdf.Path("/World/Forklift/right_rotator_joint")]),
                ("WriteSteerR.inputs:name",
                    "drive:angular:physics:targetPosition"),
                # Lift joint — control the fork height
                ("WriteLift.inputs:prim",
                    [usdrt.Sdf.Path("/World/Forklift/lift_joint")]),
                ("WriteLift.inputs:name",
                    "drive:linear:physics:targetPosition"),
                # Wheel joints — control the drive velocity
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

    # ── Bake NavMesh now so it's visible before the demo starts ─────────────
    print("[Forklift] Baking NavMesh …")
    inav = nav.acquire_interface()
    inav.start_navmesh_baking()
    for _ in range(1800):          # poll up to 30 s
        if inav.get_navmesh() is not None:
            print("[Forklift] NavMesh ready — enable overlay: Window → Navigation → NavMesh → Show NavMesh")
            break
        await omni.kit.app.get_app().next_update_async()
    else:
        print("[Forklift] WARNING: NavMesh bake timed out.")

    # ── Shorthand helpers ───────────────────────────────────────────────────
    def wheels(speed):
        """Set both rear wheel velocities. Positive = forward (+X)."""
        og.Controller.attribute(
            f"{GRAPH}/Articulation.inputs:velocityCommand").set([speed, speed])

    def steer(angle_deg):
        """Set the steering angle on both rotator joints."""
        og.Controller.attribute(
            f"{GRAPH}/Steer.inputs:value").set(angle_deg)

    def lift(height):
        """Set the lift joint target position. 0 = ground, 1.0 = raised."""
        og.Controller.attribute(
            f"{GRAPH}/Lift.inputs:value").set(height)

    async def wait(frames):
        """Advance the simulation by N frames."""
        for _ in range(frames):
            await omni.kit.app.get_app().next_update_async()

    # ── Start ───────────────────────────────────────────────────────────────
    world.play()
    await wait(10)   # let physics settle

    # ── Demo sequence ───────────────────────────────────────────────────────

    print("[Forklift] 1/7  Lowering forks to ground level …")
    lift(0.0)
    steer(0.0)
    await wait(60)

    print("[Forklift] 2/7  Driving forward — sliding forks under pallet …")
    wheels(3.0)          # slower approach for precision
    await wait(360)      # drive longer to get 90% under the pallet

    print("[Forklift] 3/7  Stopping at pallet — lifting forks …")
    wheels(0.0)
    await wait(60)
    lift(0.00)           # very low lift — barel off the ground
    await wait(120)

    print("[Forklift] 4/7  Transporting pallet to destination …")
    wheels(5.0)
    await wait(240)

    print("[Forklift] 5/7  Stopping — lowering pallet …")
    wheels(0.0)
    await wait(60)
    lift(0.0)
    await wait(120)

    print("[Forklift] 6/7  Reversing away …")
    wheels(-5.0)
    await wait(180)

    print("[Forklift] 7/7  Stopping — done!")
    wheels(0.0)
    steer(0.0)
    await wait(30)

    print("[Forklift] Demo complete — starting NavMesh roam …")

    navmesh = inav.get_navmesh()   # already baked above
    if navmesh is None:
        print("[Forklift] NavMesh not available — roam will use random XY fallback.")

    print("[Forklift] NavMesh ready — starting waypoint roam …")

    # ── Forklift state for NavMesh roam ─────────────────────────────────────
    # We track heading and position manually since the articulated robot's
    # transform is driven by physics — read it from the prim each frame.
    DRIVE_SPEED   = 4.0   # m/s forward
    MAX_STEER     = 22.0  # max steering angle (degrees) — physical limit of ForkliftC
    KP_STEER      = 0.4   # proportional gain: steer_angle = KP * heading_error
    WAYPOINT_DIST = 2.0   # m — switch to next waypoint when this close
    AGENT_RADIUS  = 0.6   # m

    forklift_prim = stage.GetPrimAtPath(Sdf.Path("/World/Forklift"))
    forklift_xform = UsdGeom.Xformable(forklift_prim)

    def get_forklift_pos():
        """Return current world XY position of the forklift."""
        xform_cache = UsdGeom.XformCache()
        mat = xform_cache.GetLocalToWorldTransform(forklift_prim)
        t = mat.ExtractTranslation()
        return Gf.Vec3d(t[0], t[1], 0.0)

    async def drive_to_waypoint(target: Gf.Vec3d):
        """Steer and drive toward a single XY waypoint, stop when close."""
        while True:
            pos = get_forklift_pos()
            dx = target[0] - pos[0]
            dy = target[1] - pos[1]
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < WAYPOINT_DIST:
                break

            # Desired heading in degrees (Isaac: X forward = 0°, Y left = 90°)
            desired_heading = math.degrees(math.atan2(dy, dx))

            # Read current forklift heading from transform
            xform_cache = UsdGeom.XformCache()
            mat = xform_cache.GetLocalToWorldTransform(forklift_prim)
            fwd = mat.GetRow3(0)   # local X axis = forward direction of forklift
            current_heading = math.degrees(math.atan2(fwd[1], fwd[0]))

            heading_err = desired_heading - current_heading
            # Normalise to [-180, 180]
            heading_err = (heading_err + 180) % 360 - 180

            # Proportional steering — small error = small steer angle
            steer_angle = max(-MAX_STEER, min(MAX_STEER, KP_STEER * heading_err))

            # Slow down proportionally to heading error so it can turn in place
            abs_err = abs(heading_err)
            if abs_err > 120:
                speed = 0.0   # stop and steer to face destination first
            elif abs_err > 60:
                speed = 1.5
            elif abs_err > 30:
                speed = 2.5
            else:
                speed = DRIVE_SPEED

            wheels(speed)
            steer(steer_angle)
            await omni.kit.app.get_app().next_update_async()

        wheels(0.0)
        steer(0.0)

    # ── NavMesh roam loop ────────────────────────────────────────────────────
    while True:
        # Pick a random destination on the NavMesh
        if navmesh is not None:
            goal_pt = navmesh.query_random_point()
            if goal_pt is None:
                # Fallback: random point in the nav area
                goal_pt = carb.Float3(
                    random.uniform(-5.0, 15.0),
                    random.uniform(-8.0, 8.0),
                    0.0,
                )
        else:
            goal_pt = carb.Float3(
                random.uniform(-5.0, 15.0),
                random.uniform(-8.0, 8.0),
                0.0,
            )

        goal = Gf.Vec3d(goal_pt[0], goal_pt[1], 0.0)
        print(f"[Forklift] Roam → ({goal[0]:.1f}, {goal[1]:.1f})")

        # Get a path along the NavMesh if available
        if navmesh is not None:
            pos = get_forklift_pos()
            start = carb.Float3(pos[0], pos[1], 0.0)
            path = navmesh.query_shortest_path(start, goal_pt, agent_radius=AGENT_RADIUS)
            waypoints = [Gf.Vec3d(p[0], p[1], 0.0) for p in path.get_points()] if path else [goal]
        else:
            waypoints = [goal]

        for wp in waypoints:
            await drive_to_waypoint(wp)

        # Brief pause at destination
        await wait(60)


asyncio.ensure_future(main())
