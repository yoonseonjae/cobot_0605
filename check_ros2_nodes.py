# Headless probe: which OmniGraph node type names actually WORK in this Isaac
# Sim 5.1 build. Instead of querying the registry (API differs across versions),
# we try to CREATE each node in a throwaway graph — creation succeeding is the
# ground truth that the type name is correct. Run via run_isaac.sh.
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.core.nodes")

# Give the extensions several frames to register their node types.
for _ in range(20):
    simulation_app.update()

import omni.graph.core as og

WANTED = {
    "Context": ["isaacsim.ros2.bridge.ROS2Context", "omni.isaac.ros2_bridge.ROS2Context"],
    "PublishClock": ["isaacsim.ros2.bridge.ROS2PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"],
    "PublishTF": ["isaacsim.ros2.bridge.ROS2PublishTransformTree", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"],
    "PublishLaserScan": ["isaacsim.ros2.bridge.ROS2PublishLaserScan", "omni.isaac.ros2_bridge.ROS2PublishLaserScan"],
    "ReadSimTime": ["isaacsim.core.nodes.IsaacReadSimulationTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"],
}

keys = og.Controller.Keys
print("\n========== ROS2 / SIM-TIME NODE TYPE RESOLUTION (create test) ==========")
probe_idx = 0
for label, candidates in WANTED.items():
    print(f"\n[{label}]")
    for c in candidates:
        # Unique graph path per attempt so a leftover node never collides.
        graph_path = f"/Probe_{probe_idx}"
        probe_idx += 1
        try:
            og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {keys.CREATE_NODES: [("probe", c)]},
            )
            print(f"   OK    {c}   <-- USE THIS")
        except Exception as e:
            msg = str(e).splitlines()[0][:90]
            print(f"   MISS  {c}   ({msg})")

print("========================================================================\n")
simulation_app.close()
