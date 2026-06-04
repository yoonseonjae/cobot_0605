from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
import omni.kit.commands
import carb
success, sensor = omni.kit.commands.execute("IsaacSensorCreateLidar", path="/Lidar", parent="/World")
print(f"Lidar creation success: {success}, sensor: {sensor}")
simulation_app.close()
