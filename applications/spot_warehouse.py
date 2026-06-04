from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import carb
import numpy as np
import os
from pathlib import Path
import omni.appwindow
import cv2

from isaacsim.core.api import World
from isaacsim.core.utils.prims import define_prim
from spot_policy import SpotFlatTerrainPolicy, SpotArmFlatTerrainPolicy
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.utils.extensions import enable_extension
enable_extension('isaacsim.ros2.bridge')

class SpotRunner(object):
    def __init__(self, physics_dt, render_dt) -> None:
        self._world = World(stage_units_in_meters=1.0, physics_dt=physics_dt, rendering_dt=render_dt)

        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")

        BASE_DIR = Path(__file__).resolve().parent.parent

        # Dome light so the scene is lit (custom USD map ships no lights -> renders black)
        from pxr import UsdLux, Sdf
        import omni.usd as _omni_usd
        _stage = _omni_usd.get_context().get_stage()
        _dome = UsdLux.DomeLight.Define(_stage, Sdf.Path("/World/DomeLight"))
        _dome.CreateIntensityAttr(1000.0)

        # Spawn warehouse (Default)
        # prim = define_prim("/World/Warehouse", "Xform")
        # asset_path = assets_root_path + "/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd"
        # prim.GetReferences().AddReference(asset_path)
        
        # Custom map
        prim = define_prim("/World/Map", "Xform")
        asset_path = os.path.join(BASE_DIR, "Collected_c_1_default_map", "c_1_default_map.usd")
        prim.GetReferences().AddReference(asset_path)

        # --- Add Collision to Custom Map ---
        from pxr import Usd, UsdGeom, UsdPhysics
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        map_prim = stage.GetPrimAtPath("/World/Map")
        if map_prim:
            for p in Usd.PrimRange(map_prim):
                if p.IsA(UsdGeom.Mesh):
                    UsdPhysics.CollisionAPI.Apply(p)
                    mesh_api = UsdPhysics.MeshCollisionAPI.Apply(p)
                    mesh_api.CreateApproximationAttr("none") # Use exact triangle mesh for collision
        # -----------------------------------
        policy_path = os.path.join(BASE_DIR, "policies/spot_arm/models", "spot_arm_policy.pt")
        policy_params_path = os.path.join(BASE_DIR, "policies/spot_arm/params", "env.yaml")
        usd_path = os.path.join(BASE_DIR, "assets", "spot_arm.usd")

        self._spot = SpotArmFlatTerrainPolicy(
            prim_path="/World/Spot",
            name="Spot",
            usd_path=usd_path,
            policy_path=policy_path,
            policy_params_path=policy_params_path,
            position=np.array([1, 0, 0.8]),
        )

        # ---------------------------------------------------------
        # The user has modified assets/spot_arm.usd directly,
        # so we no longer generate the dynamic basket here.
        # ---------------------------------------------------------

        self._base_command = np.zeros(3)
        self._input_keyboard_mapping = {
            "NUMPAD_8": [1.0, 0.0, 0.0], "UP": [1.0, 0.0, 0.0],
            "NUMPAD_2": [-1.0, 0.0, 0.0], "DOWN": [-1.0, 0.0, 0.0],
            "NUMPAD_6": [0.0, -1.0, 0.0], "RIGHT": [0.0, -1.0, 0.0],
            "NUMPAD_4": [0.0, 1.0, 0.0], "LEFT": [0.0, 1.0, 0.0],
            "NUMPAD_7": [0.0, 0.0, 1.0], "N": [0.0, 0.0, 1.0],
            "NUMPAD_9": [0.0, 0.0, -1.0], "M": [0.0, 0.0, -1.0],
        }

        self.needs_reset = False
        self.first_step = True

    def setup(self) -> None:
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(
            self._keyboard, self._sub_keyboard_event
        )

        self.setup_ros2_nav2_bridge()
        self._world.add_physics_callback("spot_forward", callback_fn=self.on_physics_step)

    def setup_ros2_nav2_bridge(self):
        import omni.graph.core as og
        import omni.replicator.core as rep
        import omni.usd
        import omni.kit.commands
        from isaacsim.core.utils.extensions import enable_extension

        # Isaac Sim 5.1 namespaces (verified via check_ros2_nodes.py)
        enable_extension("isaacsim.ros2.bridge")
        enable_extension("isaacsim.core.nodes")

        stage = omni.usd.get_context().get_stage()
        
        # 1. Setup Clock and TF Tree
        try:
            keys = og.Controller.Keys
            (graph, nodes, _, _) = og.Controller.edit(
                {"graph_path": "/ROS2_Graph", "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                        ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                        ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                        ("Context.outputs:context", "PublishClock.inputs:context"),

                        ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                        ("Context.outputs:context", "PublishTF.inputs:context"),
                    ],
                }
            )
            tf_node = stage.GetPrimAtPath("/ROS2_Graph/PublishTF")
            rel = tf_node.GetRelationship("inputs:targetPrims")
            if rel:
                rel.SetTargets(["/World"])
            print("[Nav2] ROS 2 Clock and TF Graph created.")
        except Exception as e:
            print(f"[Nav2] Error setting up Clock/TF Graph: {e}")

        # 2. Spawn a fully functional RTX Lidar via Code
        # This guarantees it works, avoiding the issue of visual-only props being attached in GUI.
        from pxr import Gf
        lidar_parent = "/World/Spot/body"
        lidar_path = f"{lidar_parent}/Functional_Lidar"

        # Isaac Sim 5.1: orientation must be a Gf.Quatd (w, x, y, z), not a tuple.
        success, sensor_prim_path = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path="Functional_Lidar",
            parent=lidar_parent,
            config="TIM781",  # SICK TiM 2D planar lidar (verified in 5.1 via check_lidar_configs.py)
            translation=Gf.Vec3d(0.0, 0.0, 0.25),
            orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
        )
        
        if success:
            try:
                render_product = rep.create.render_product(sensor_prim_path, [1, 1])
                writer = rep.writers.get("RtxLidar" + "ROS2PublishLaserScan")
                writer.initialize(topicName="/scan", frameId="Functional_Lidar") 
                writer.attach([render_product])
                print(f"[Nav2] Successfully attached Replicator writer to Lidar at: {sensor_prim_path}")
            except Exception as e:
                print(f"[Nav2] Failed to attach Lidar writer: {e}")
        else:
            print("[Nav2] Warning: Failed to spawn functional RTX Lidar!")

    def on_physics_step(self, step_size) -> None:
        if self.first_step:
            self._spot.initialize()
            self._spot.robot.set_joint_positions(self._spot.default_pos)
            self._spot.robot.set_joint_velocities(self._spot.default_vel)
            self.first_step = False
        elif self.needs_reset:
            self._world.reset(True)
            self.needs_reset = False
            self.first_step = True
        else:
            self._spot.forward(step_size, self._base_command)

    def run(self) -> None:
        while simulation_app.is_running():
            self._world.step(render=True)
            if self._world.is_stopped():
                self.needs_reset = True
        return

    def _sub_keyboard_event(self, event, *args, **kwargs) -> bool:
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name in self._input_keyboard_mapping:
                self._base_command += np.array(self._input_keyboard_mapping[event.input.name])
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            if event.input.name in self._input_keyboard_mapping:
                self._base_command -= np.array(self._input_keyboard_mapping[event.input.name])
        return True


def main():
    physics_dt = 1 / 200.0
    render_dt = 1 / 60.0

    runner = SpotRunner(physics_dt=physics_dt, render_dt=render_dt)
    simulation_app.update()
    runner._world.reset()
    simulation_app.update()
    runner.setup()
    simulation_app.update()
    runner.run()
    simulation_app.close()


if __name__ == "__main__":
    main()
