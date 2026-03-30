"""
=============================================================================
  Isaac Sim 5.1 — Simple Person NavMesh Navigation Example
=============================================================================

  WHAT YOU WILL SEE
  -----------------
  One person walks a large loop that covers the ENTIRE warehouse floor,
  automatically following the Navigation Mesh (NavMesh) so they never
  walk through walls or shelves.

  HOW TO RUN
  ----------
  Isaac Sim  →  Window  →  Script Editor  →  Open this file  →  Ctrl+Enter

  KEY CONCEPTS (explained below with the code)
  ------------
  NavMesh     : A map of every spot in the scene that a person can walk on.
                IRA bakes it at startup from the scene geometry and uses it
                to find obstacle-free paths between any two positions.

  IRA         : isaacsim.replicator.agent — Isaac Sim's built-in system for
                placing and animating humanoid characters. Given a scene +
                command list it handles spawning, NavMesh pathfinding, and
                walk/idle animations automatically.

  Command file: A plain-text list of instructions per character.
                  GoTo  x  y  z  _   → walk to world position via NavMesh
                  Idle  t             → stand still for t seconds
                  LookAround  t       → look around naturally for t seconds
                The list loops automatically so the person walks forever.

=============================================================================
"""

import asyncio
import os

import carb.eventdispatcher
import omni.timeline
import yaml  # bundled with Isaac Sim

# ─────────────────────────────────────────────────────────────────────────────
#  SETTINGS — change these to match your environment
# ─────────────────────────────────────────────────────────────────────────────

# Root folder where Isaac Sim assets live on this machine.
ASSETS_ROOT = "/home/ubuntu/isaacsim_assets/Assets/Isaac/5.1"

# The warehouse USD — this file already has a NavMesh volume baked inside it.
SCENE_USD = f"{ASSETS_ROOT}/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"

# Folder containing humanoid character USD assets.
CHARACTERS_FOLDER = f"{ASSETS_ROOT}/Isaac/People/Characters/"

# ─────────────────────────────────────────────────────────────────────────────
#  STEP 1 — Enable IRA (the people-navigation extension)
# ─────────────────────────────────────────────────────────────────────────────
from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.replicator.agent.core")

# Must import AFTER enabling the extension
from isaacsim.replicator.agent.core.simulation import SimulationManager

# ─────────────────────────────────────────────────────────────────────────────
#  STEP 2 — Write the command files
#
#  IMPORTANT: IRA reads the command file during every loop of the simulation,
#  not just at startup. The files must stay on disk the entire time the
#  simulation is running. We write them next to this script so they persist.
# ─────────────────────────────────────────────────────────────────────────────

# Directory this script lives in — we write supporting files here.
HERE = os.path.dirname(os.path.abspath(__file__))

# ── Character movement commands ───────────────────────────────────────────────
#  Waypoints cover EVERY AREA of the warehouse:
#    east/west aisles, far north, far south, corners, centre.
#
#  IRA uses the NavMesh to find the walkable path between each GoTo point
#  automatically routing around shelves and walls.
#
#  The list loops forever — the person never stops walking.
# ─────────────────────────────────────────────────────────────────────────────
CHARACTER_COMMANDS = """\
# One person walks a large loop covering the entire warehouse floor.
# IRA routes around shelves automatically on each GoTo via the NavMesh.
# This command list loops forever — the person never stops.

Character LookAround 2.0
Character GoTo   1.24    5.79  0.0  _
Character Idle   1.5

Character GoTo   4.61   -5.82  0.0  _
Character LookAround 2.0

Character GoTo   2.55  -16.60  0.0  _
Character Idle   2.0

Character GoTo  -14.52  -22.38  0.0  _
Character LookAround 2.5

Character GoTo  -19.24  -12.59  0.0  _
Character Idle   1.5

Character GoTo  -23.32   -2.37  0.0  _
Character LookAround 2.0

Character GoTo  -16.76   -4.85  0.0  _
Character Idle   1.0

Character GoTo   -1.28    5.08  0.0  _
Character LookAround 2.0

Character GoTo  -11.16    7.25  0.0  _
Character Idle   1.5

Character GoTo   -3.04   24.90  0.0  _
Character LookAround 3.0

Character GoTo    1.41   24.04  0.0  _
Character Idle   2.0
"""

# ── Robot command file (empty — no robots in this example) ───────────────────
#  IRA always looks for a robot command file. Providing an empty one
#  avoids the "Unable to set up robot command file: None" error.
ROBOT_COMMANDS = "# No robots in this example\n"

# Write files to the script directory so they survive for the whole session
CMD_PATH       = os.path.join(HERE, "_person_commands.txt")
ROBOT_CMD_PATH = os.path.join(HERE, "_robot_commands.txt")
CFG_PATH       = os.path.join(HERE, "_ira_config.yaml")

with open(CMD_PATH,       "w") as f: f.write(CHARACTER_COMMANDS)
with open(ROBOT_CMD_PATH, "w") as f: f.write(ROBOT_COMMANDS)

# ─────────────────────────────────────────────────────────────────────────────
#  STEP 3 — Build and write the IRA configuration
# ─────────────────────────────────────────────────────────────────────────────
#  IRA is configured via a YAML file. These are the minimum fields needed
#  to spawn one person and have them walk around.
# ─────────────────────────────────────────────────────────────────────────────
ira_config = {
    "isaacsim.replicator.agent": {
        "version": "0.7.0",
        "global": {
            "seed": 42,
            "simulation_length": 99999,   # very large → runs until you stop it
        },
        "scene": {
            # Warehouse USD that already contains a NavMesh volume.
            # IRA rebakes the NavMesh at startup from this geometry,
            # then uses it to route the character around shelves/walls.
            "asset_path": SCENE_USD,
        },
        "character": {
            "asset_path":      CHARACTERS_FOLDER,  # IRA picks one character from here
            "command_file":    CMD_PATH,            # walk/idle instructions (step 2)
            "filters":         [],
            "navigation_area": [],                  # [] = use the full NavMesh
            "spawn_area":      [],                  # [] = spawn anywhere on NavMesh
            "num": 1,                              # spawn exactly 1 person
        },
        "robot": {
            # No robots — but IRA still needs a command_file entry (even empty).
            "command_file":    ROBOT_CMD_PATH,
            "nova_carter_num": 0,
            "transporter_num": 0,
            "iw_hub_num":      0,
            "spawn_area":      [],
            "navigation_area": [],
            "write_data":      False,
        },
        "write_data": False,   # set True to save RGB + bounding-box data to disk
    }
}

with open(CFG_PATH, "w") as f:
    yaml.dump(ira_config, f)

# ─────────────────────────────────────────────────────────────────────────────
#  STEP 4 — Set up and start the simulation
# ─────────────────────────────────────────────────────────────────────────────
#  IRA's setup is asynchronous (opening a large USD takes time), so we
#  schedule it with asyncio, which Isaac Sim's Script Editor supports.
# ─────────────────────────────────────────────────────────────────────────────
async def run():

    # Create the SimulationManager and load our config.
    # SimulationManager is IRA's core class — it orchestrates everything:
    #   • opens the scene USD
    #   • bakes the NavMesh from scene geometry
    #   • spawns the character at a random walkable position
    #   • loads our command file so the character knows where to walk
    sim = SimulationManager()
    ok = sim.load_config_file(CFG_PATH)
    if not ok:
        print("[NavExample] ERROR: Config failed to load. Check the ASSETS_ROOT path at the top of this file.")
        return

    # set_up_simulation_from_config_file() is non-blocking — it fires the
    # SET_UP_SIMULATION_DONE_EVENT when the scene + NavMesh + characters
    # are all ready. We use an asyncio.Event to wait for it.
    setup_done = asyncio.Event()

    def on_setup_done(_event):
        setup_done.set()

    _sub = sim.register_set_up_simulation_done_callback(on_setup_done)

    print("[NavExample] Loading scene and baking NavMesh …")
    sim.set_up_simulation_from_config_file()

    await setup_done.wait()   # wait for scene + NavMesh + character to be ready
    _sub = None               # release the event subscription

    # Play the timeline — this starts the physics + animation loop.
    # The CharacterBehavior script attached to the person will read
    # our command file and begin walking immediately.
    #
    # NOTE: The command files in this directory must NOT be deleted while
    # the simulation is running — CharacterBehavior re-reads them each loop.
    print("[NavExample] Setup complete — starting simulation!")
    print("[NavExample]   The person is now walking around the warehouse.")
    print("[NavExample]   Stop the timeline to pause, close Isaac Sim to exit.")
    print(f"[NavExample]   Command file: {CMD_PATH}\n")
    omni.timeline.get_timeline_interface().play()


# Schedule on Isaac Sim's already-running async event loop
asyncio.ensure_future(run())
