from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.kit.commands
import omni.graph.core as og
import omni.usd
from omni.isaac.core.utils.extensions import enable_extension

enable_extension("isaacsim.ros2.bridge")
enable_extension("omni.isaac.range_sensor")

stage = omni.usd.get_context().get_stage()

success, sensor = omni.kit.commands.execute(
    "IsaacSensorCreateLidar",
    path="/Lidar",
    parent="/World",
    min_range=0.4,
    max_range=25.0,
    draw_points=False,
    draw_lines=True,
    horizontal_fov=360.0,
    vertical_fov=1.0,
    horizontal_resolution=1.0,
    vertical_resolution=1.0,
    rotation_rate=0.0,
    direction_vector=(1.0, 0.0, 0.0),
    up_vector=(0.0, 0.0, 1.0),
    translation=(0.0, 0.0, 0.2)
)

print(f"Lidar creation success: {success}, sensor: {sensor}")

keys = og.Controller.Keys
(graph, nodes, _, _) = og.Controller.edit(
    {"graph_path": "/ROS2_Graph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadLidar", "omni.isaac.range_sensor.IsaacReadLidarBeams"),
            ("PublishLidar", "omni.isaac.ros2_bridge.ROS2PublishLaserScan"),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "ReadLidar.inputs:execIn"),
            ("ReadLidar.outputs:execOut", "PublishLidar.inputs:execIn"),
            
            ("ReadLidar.outputs:horizontalFov", "PublishLidar.inputs:horizontalFov"),
            ("ReadLidar.outputs:horizontalResolution", "PublishLidar.inputs:horizontalResolution"),
            ("ReadLidar.outputs:depthRange", "PublishLidar.inputs:depthRange"),
            ("ReadLidar.outputs:linearDepthData", "PublishLidar.inputs:linearDepthData"),
            ("ReadLidar.outputs:intensitiesData", "PublishLidar.inputs:intensitiesData"),
            ("ReadLidar.outputs:numRows", "PublishLidar.inputs:numRows"),
            ("ReadLidar.outputs:numCols", "PublishLidar.inputs:numCols"),
            ("ReadLidar.outputs:rotationRate", "PublishLidar.inputs:rotationRate"),
        ],
        keys.SET_VALUES: [
            ("PublishLidar.inputs:topicName", "/scan"),
            ("PublishLidar.inputs:frameId", "Lidar"),
        ]
    }
)

rel = stage.GetPrimAtPath("/ROS2_Graph/ReadLidar").GetRelationship("inputs:lidarPrim")
rel.SetTargets(["/World/Lidar"])

simulation_app.update()
import omni.timeline
omni.timeline.get_timeline_interface().play()
for _ in range(10):
    simulation_app.update()

print("Graph and Lidar tested.")
simulation_app.close()
