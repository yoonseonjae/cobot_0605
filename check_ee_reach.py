# Headless probe: where is the body (back) vs the arm EE (arm0_link_fngr)?
# We need this to place stowage cubes on the back at a spot the EE can actually
# reach for the Fake Grasp. Prints world positions at default pose + the body
# frame so we can compute a back-mounted offset.
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import numpy as np
import os
from pathlib import Path

from isaacsim.core.api import World
from isaacsim.core.utils.prims import define_prim
from isaacsim.core.prims import SingleArticulation, SingleXFormPrim

BASE_DIR = Path(__file__).resolve().parent
usd_path = os.path.join(BASE_DIR, "assets", "spot_arm.usd")

world = World()

# Add the robot USD as a reference under /World/Spot.
prim = define_prim("/World/Spot", "Xform")
prim.GetReferences().AddReference(usd_path)

robot = SingleArticulation(prim_path="/World/Spot", name="Spot", position=np.array([0.0, 0.0, 0.8]))
world.reset()
robot.initialize()

# Step a bit so transforms settle.
for _ in range(5):
    world.step(render=False)

print("\n========== SPOT ARM FRAME PROBE ==========")
print("DOF names:", robot.dof_names)

# Body (back) and EE world poses via XForm prims.
def world_pos(path):
    try:
        p = SingleXFormPrim(prim_path=path)
        pos, _ = p.get_world_pose()
        return np.asarray(pos)
    except Exception as e:
        return f"ERR {e}"

body = world_pos("/World/Spot/body")
ee   = world_pos("/World/Spot/arm0_link_fngr")
wr1  = world_pos("/World/Spot/arm0_link_wr1")

print(f"\nbody (back)       world pos: {body}")
print(f"arm0_link_wr1     world pos: {wr1}")
print(f"arm0_link_fngr(EE) world pos: {ee}")

if isinstance(body, np.ndarray) and isinstance(ee, np.ndarray):
    print(f"\nEE relative to body (EE - body): {ee - body}")
    print(f"  -> dx(forward)={ee[0]-body[0]:.3f}  dy(left)={ee[1]-body[1]:.3f}  dz(up)={ee[2]-body[2]:.3f}")
    print(f"  EE-to-body distance: {np.linalg.norm(ee - body):.3f} m")

print("==========================================\n")
simulation_app.close()
