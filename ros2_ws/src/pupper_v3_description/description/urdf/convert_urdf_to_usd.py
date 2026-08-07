"""Convert pupper_v3.urdf into a USD asset for Isaac Sim 5.1.

Run headless with the Isaac Sim python interpreter:
    ~/isaacsim/python.sh convert_urdf_to_usd.py

Output: description/urdf/pupper_v3/pupper_v3.usd
"""

import os

from isaacsim import SimulationApp

kit = SimulationApp({"headless": True})

import omni.kit.commands
import omni.usd
from isaacsim.asset.importer.urdf import _urdf

HERE = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.join(HERE, "pupper_v3.urdf")
DEST_PATH = os.path.join(HERE, "pupper_v3", "pupper_v3.usd")

# Import configuration tuned for a floating-base quadruped locomotion robot.
status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
import_config.merge_fixed_joints = True          # collapse fixed joints into parent links
import_config.fix_base = False                    # floating base (free to move)
import_config.import_inertia_tensor = True        # use inertia values from the URDF
import_config.distance_scale = 1.0                # URDF is already in meters
import_config.convex_decomp = False               # convex-hull collision per mesh
import_config.self_collision = False              # disable self-collision by default
import_config.make_default_prim = True
import_config.create_physics_scene = True
import_config.parse_mimic = True
# Leave joint drives unconfigured; RL / controllers set gains downstream.
import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_NONE
import_config.default_position_drive_damping = -1

status, prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=URDF_PATH,
    import_config=import_config,
    dest_path=DEST_PATH,
    get_articulation_root=True,
)

print(f"Imported articulation at prim: {prim_path}")
print(f"Wrote USD: {DEST_PATH}")

kit.close()
