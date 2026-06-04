from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import omni.usd
from pxr import UsdGeom
import os

usd_path = "/home/yoon/IsaacRobotics-main/assets/spot_arm.usd"
omni.usd.get_context().open_stage(usd_path)
stage = omni.usd.get_context().get_stage()
print("Links:", [p.GetPath().pathString for p in stage.Traverse() if "link" in p.GetName().lower() or "body" in p.GetName().lower()])
print("Joints:", [p.GetPath().pathString for p in stage.Traverse() if p.GetTypeName() == "PhysicsRevoluteJoint"])
app.close()
