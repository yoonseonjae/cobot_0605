from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import omni.kit.commands
from pxr import UsdGeom
import omni.usd

success, sensor = omni.kit.commands.execute("IsaacSensorCreateLidar", path="/Lidar", parent="/World")
print(f"[TEST] Lidar creation: success={success}")

if not success:
    success2, sensor2 = omni.kit.commands.execute("IsaacSensorCreateRtxLidar", path="/RtxLidar", parent="/World", config="Sick_LMS111")
    print(f"[TEST] RTX Lidar creation: success={success2}")

app.close()
