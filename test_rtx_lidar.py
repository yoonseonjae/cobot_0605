from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.kit.commands
from omni.isaac.core.api import World
from omni.isaac.core.utils.extensions import enable_extension

enable_extension('isaacsim.ros2.bridge')

world = World()
world.reset()

import omni.replicator.core as rep

success, sensor = omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar",
    path="/Lidar",
    parent="/World",
    config="Sick_LMS111",
    translation=(0.0, 0.0, 0.2),
    orientation=(1.0, 0.0, 0.0, 0.0)
)
print(f"Lidar creation: success={success}, sensor={sensor}")

try:
    render_product = rep.create.render_product(sensor, [1, 1])
    writer = rep.writers.get("RtxLidar" + "ROS2PublishLaserScan")
    writer.initialize(topicName="/scan", frameId="Lidar")
    writer.attach([render_product])
    print("Replicator writer attached successfully.")
except Exception as e:
    print(f"Failed to attach writer: {e}")

simulation_app.update()
for _ in range(10):
    world.step(render=True)

print("Test complete.")
simulation_app.close()
