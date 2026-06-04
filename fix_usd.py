import omni
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
from pxr import Usd, UsdGeom, UsdPhysics

stage = Usd.Stage.Open("/home/rokey/Downloads/IsaacRobotics-main/assets/spot_arm_basket.usd")

for prim in stage.Traverse():
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        print(f"Found RigidBodyAPI on: {prim.GetPath()}")
        if "body" in str(prim.GetPath()) and prim.GetName() != "body":
            print(f"Removing RigidBodyAPI from: {prim.GetPath()}")
            prim.RemoveAPI(UsdPhysics.RigidBodyAPI)

stage.Export("/home/rokey/Downloads/IsaacRobotics-main/assets/spot_arm_basket.usd")
print("Saved fixed USD.")
simulation_app.close()
