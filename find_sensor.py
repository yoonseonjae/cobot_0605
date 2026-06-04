from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import omni.usd
from pxr import Usd

omni.usd.get_context().open_stage("/home/rokey/Downloads/IsaacRobotics-main/assets/spot_arm.usd")
app.update()

stage = omni.usd.get_context().get_stage()
print("--- SENSOR PRIMS ---")
for p in Usd.PrimRange(stage.GetPseudoRoot()):
    if p.HasAPI("IsaacRtxLidarSensorAPI") or p.GetTypeName() in ["Camera", "Lidar"]:
        print(f"SENSOR FOUND: {p.GetPath().pathString} (Type: {p.GetTypeName()})")

app.close()
