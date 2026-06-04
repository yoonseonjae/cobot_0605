from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import numpy as np
from spot_policy import SpotArmFlatTerrainPolicy
import os
from pathlib import Path
BASE_DIR = Path("/home/yoon/IsaacRobotics-main")
policy_path = os.path.join(BASE_DIR, "policies/spot_arm/models", "spot_arm_policy.pt")
policy_params_path = os.path.join(BASE_DIR, "policies/spot_arm/params", "env.yaml")
usd_path = os.path.join(BASE_DIR, "assets", "spot_arm.usd")
spot = SpotArmFlatTerrainPolicy(
    prim_path="/World/Spot", name="Spot", usd_path=usd_path,
    policy_path=policy_path, policy_params_path=policy_params_path,
    position=np.array([0.0, 0.0, 0.8]),
)
spot.initialize()
print(spot.robot.dof_names)
app.close()
