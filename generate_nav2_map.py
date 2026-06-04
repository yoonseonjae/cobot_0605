from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import numpy as np
import os
import cv2
import yaml
from pxr import Usd, UsdGeom, UsdPhysics
import omni.usd

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
usd_path = os.path.join(BASE_DIR, "Collected_c_1_default_map", "c_1_default_map.usd")

print(f"Loading stage: {usd_path}")
omni.usd.get_context().open_stage(usd_path)
simulation_app.update()

stage = omni.usd.get_context().get_stage()
default_prim = stage.GetDefaultPrim()
root_path = default_prim.GetPath().pathString if default_prim else "/World"

print("Applying physics colliders to meshes...")
for p in Usd.PrimRange(stage.GetPrimAtPath(root_path)):
    if p.IsA(UsdGeom.Mesh):
        UsdPhysics.CollisionAPI.Apply(p)
        mesh_api = UsdPhysics.MeshCollisionAPI.Apply(p)
        mesh_api.CreateApproximationAttr("none")

# Use World to ensure Physics is fully initialized and colliders are parsed
from isaacsim.core.api import World
world = World()
world.reset()

simulation_app.update()

print("Calculating bounding box for the map...")
bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
bbox = bbox_cache.ComputeWorldBound(stage.GetPrimAtPath(root_path)).ComputeAlignedBox()
min_pt = bbox.GetMin()
max_pt = bbox.GetMax()

print(f"Map Bounds -> Min: {min_pt}, Max: {max_pt}")

# Use Isaac Sim Occupancy Map Generator
print("Initializing Occupancy Map Generator...")
from omni.isaac.core.utils.extensions import enable_extension

try:
    enable_extension("omni.isaac.occupancy_map")
    from omni.isaac.occupancy_map import _occupancy_map
    physx = omni.physx.get_physx_interface()
    stage_id = omni.usd.get_context().get_stage_id()
    generator = _occupancy_map.Generator(physx, stage_id)
except Exception:
    enable_extension("isaacsim.asset.gen.omap")
    from isaacsim.asset.gen.omap.bindings import _omap
    import omni.physx
    physx = omni.physx.get_physx_interface()
    stage_id = omni.usd.get_context().get_stage_id()
    generator = _omap.Generator(physx, stage_id)

cell_size = 0.05
# Update settings: cell_size, occupied_val=4, free_val=5, unknown_val=6
generator.update_settings(cell_size, 4, 5, 6)

# Set bounds: Z from 0.1 to 1.0 meters to capture walls and ignore floor/ceiling
min_bound = (min_pt[0], min_pt[1], 0.1)
max_bound = (max_pt[0], max_pt[1], 1.0)
# Use Z=0.5 for origin to ensure it's not inside the floor mesh
generator.set_transform((0.0, 0.0, 0.5), min_bound, max_bound)

# Step physics so it loads collision geometries
world.step(render=False)

print(f"Generating 2D occupancy map with cell size {cell_size}m...")
try:
    generator.update()
except AttributeError:
    pass
generator.generate2d()

dims = generator.get_dimensions()
buffer = generator.get_buffer()

if dims[0] == 0 or dims[1] == 0 or not buffer:
    print("ERROR: Generated map buffer is empty! The origin might be inside an obstacle or Physics is not loaded.")
    simulation_app.close()
    exit(1)

print(f"Map dimensions: {dims[0]}x{dims[1]}")

# Convert buffer to numpy array
# buffer format is flattened array.
img = np.array(buffer).reshape((dims[1], dims[0])).astype(np.uint8)

# Convert to standard ROS map values
# 4 = Occupied -> 0 (Black)
# 5 = Free -> 255 (White)
# 6 = Unknown -> 205 (Grey)
nav2_img = np.full_like(img, 205)
nav2_img[img == 4] = 0
nav2_img[img == 5] = 255

map_name = "nav2_map"
png_path = os.path.join(BASE_DIR, f"{map_name}.png")
yaml_path = os.path.join(BASE_DIR, f"{map_name}.yaml")

print(f"Saving map image to {png_path}...")
cv2.imwrite(png_path, nav2_img)

print(f"Saving ROS Nav2 configuration to {yaml_path}...")
# Note: origin is [x, y, yaw] corresponding to bottom-left pixel
yaml_content = {
    "image": f"{map_name}.png",
    "resolution": cell_size,
    "origin": [float(min_bound[0]), float(min_bound[1]), 0.0],
    "negate": 0,
    "occupied_thresh": 0.65,
    "free_thresh": 0.25
}

with open(yaml_path, "w") as f:
    yaml.dump(yaml_content, f, default_flow_style=False)

print("Nav2 SLAM Map successfully generated!")
simulation_app.close()
