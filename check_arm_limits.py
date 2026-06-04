# Headless probe: arm joint limits + default pose angles, so we can plan a
# "fold arm over the back" pose without guessing joint directions/ranges.
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import numpy as np
import os
from pathlib import Path

from isaacsim.core.api import World
from isaacsim.core.utils.prims import define_prim
from isaacsim.core.prims import SingleArticulation

BASE_DIR = Path(__file__).resolve().parent
usd_path = os.path.join(BASE_DIR, "assets", "spot_arm.usd")

world = World()
prim = define_prim("/World/Spot", "Xform")
prim.GetReferences().AddReference(usd_path)
robot = SingleArticulation(prim_path="/World/Spot", name="Spot", position=np.array([0.0, 0.0, 0.8]))
world.reset()
robot.initialize()
for _ in range(5):
    world.step(render=False)

names = robot.dof_names
default_pos = robot.get_joint_positions()

# Joint limits from the articulation view.
try:
    lower = robot.dof_properties["lower"]
    upper = robot.dof_properties["upper"]
except Exception:
    # Fallback API
    limits = robot.get_dof_limits()
    lower = np.asarray(limits)[:, 0]
    upper = np.asarray(limits)[:, 1]

print("\n========== ARM JOINT LIMITS / DEFAULT POSE ==========")
print(f"{'idx':>3}  {'name':<12} {'default(rad)':>12} {'lower':>9} {'upper':>9}")
for i, n in enumerate(names):
    tag = "  <-- ARM" if n.startswith("arm0") else ""
    print(f"{i:>3}  {n:<12} {default_pos[i]:>12.3f} {lower[i]:>9.3f} {upper[i]:>9.3f}{tag}")
print("=====================================================\n")
simulation_app.close()
