"""
=============================================================================
  Isaac Sim 5.1 — Simple Person Random NavMesh Navigation Example
=============================================================================

  WHAT YOU WILL SEE
  -----------------
  One person walks randomly around the ENTIRE warehouse floor forever.
  Every destination is a random point on the NavMesh, so the character
  automatically covers the whole walkable area and never repeats the
  same route twice.

  HOW TO RUN
  ----------
  Isaac Sim  →  Window  →  Script Editor  →  Open this file  →  Ctrl+Enter

  HOW IT WORKS
  ------------
  1. The warehouse scene is loaded (it already has a NavMesh volume).
  2. IRA bakes the NavMesh from the scene geometry at startup.
  3. generate_random_commands() queries the NavMesh for random walkable
     points and produces a list of GoTo / Idle / LookAround commands.
  4. These commands are saved to a file that CharacterBehavior reads.
  5. The timeline plays — the character walks the random route and loops.

=============================================================================
"""

import asyncio
import os

import carb.eventdispatcher
import omni.timeline
import yaml

# ─────────────────────────────────────────────────────────────────────────────
#  SETTINGS
# ─────────────────────────────────────────────────────────────────────────────

ASSETS_ROOT = "/home/ubuntu/isaacsim_assets/Assets/Isaac/5.1"

SCENE_USD         = f"{ASSETS_ROOT}/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
CHARACTERS_FOLDER = f"{ASSETS_ROOT}/Isaac/People/Characters/"

# How many frames of random commands to generate (30 fps).
# These commands loop automatically, so 600 frames (20 s) of waypoints
# is enough — the character will re-walk the random route on each loop.
COMMAND_FRAMES = 600

# ─────────────────────────────────────────────────────────────────────────────
#  STEP 1 — Enable IRA
# ─────────────────────────────────────────────────────────────────────────────
from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.replicator.agent.core")

from isaacsim.replicator.agent.core.simulation import SimulationManager

# ─────────────────────────────────────────────────────────────────────────────
#  STEP 2 — Write supporting files
#
#  The command file is initially empty — it will be filled with random
#  NavMesh waypoints AFTER the NavMesh is baked (step 4 below).
#  It must stay on disk for the entire simulation (CharacterBehavior
#  re-reads it on every loop).
# ─────────────────────────────────────────────────────────────────────────────
HERE           = os.path.dirname(os.path.abspath(__file__))
CMD_PATH       = os.path.join(HERE, "_person_commands.txt")
ROBOT_CMD_PATH = os.path.join(HERE, "_robot_commands.txt")
CFG_PATH       = os.path.join(HERE, "_ira_config.yaml")

# Placeholder — will be overwritten with real random commands after NavMesh bake
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
            # simulation_length controls how many frames of random commands
            # generate_random_commands() will produce. Commands loop after
            # this many frames, so the character keeps walking forever.
            "simulation_length": COMMAND_FRAMES,
        },
        "scene": {
            "asset_path": SCENE_USD,
        },
        "character": {
            "asset_path":      CHARACTERS_FOLDER,
            "command_file":    CMD_PATH,   # filled with random waypoints in step 4
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
#  STEP 4 — Run the simulation
# ─────────────────────────────────────────────────────────────────────────────
async def run():
    sim = SimulationManager()

    ok = sim.load_config_file(CFG_PATH)
    if not ok:
        print("[NavExample] ERROR: Config failed to load — check ASSETS_ROOT.")
        return

    # Wait for scene load + NavMesh bake + character spawn
    setup_done = asyncio.Event()
    _sub = sim.register_set_up_simulation_done_callback(lambda _: setup_done.set())

    print("[NavExample] Loading scene and baking NavMesh …")
    sim.set_up_simulation_from_config_file()
    await setup_done.wait()
    _sub = None

    # ── Generate random waypoints using the live NavMesh ────────────────────
    #  generate_random_commands() queries the NavMesh for random walkable
    #  points and returns a list of GoTo / Idle / LookAround command strings.
    #  Every point is guaranteed to be reachable — the NavMesh ensures that.
    #  The list covers the entire walkable area of the warehouse.
    print("[NavExample] Generating random waypoints from NavMesh …")
    random_commands = await sim.generate_random_commands()

    if not random_commands:
        print("[NavExample] WARNING: No random commands generated — NavMesh may be empty.")
        print("[NavExample]   The character will still spawn but won't move.")
    else:
        print(f"[NavExample] Generated {len(random_commands)} random waypoints.")

    # Save the random commands to the command file.
    # CharacterBehavior will read this file when the timeline starts
    # and loop through the waypoints forever.
    sim.save_commands(random_commands)

    # ── Start the simulation ─────────────────────────────────────────────────
    print("[NavExample] Starting simulation — person walking randomly!")
    print("[NavExample]   The route loops automatically — the person walks forever.")
    print("[NavExample]   Stop the timeline to pause.\n")
    omni.timeline.get_timeline_interface().play()


asyncio.ensure_future(run())
