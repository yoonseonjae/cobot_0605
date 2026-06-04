from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import omni.usd
from pxr import Usd

omni.usd.get_context().open_stage("/home/rokey/Downloads/IsaacRobotics-main/assets/spot_arm.usd")
app.update()

stage = omni.usd.get_context().get_stage()
print("--- PRIMS ---")
for p in Usd.PrimRange(stage.GetPseudoRoot()):
    if "Lidar" in p.GetName() or "LRS" in p.GetName() or "sick" in p.GetName().lower():
        print(f"FOUND: {p.GetPath().pathString} (Type: {p.GetTypeName()})")

app.close()
