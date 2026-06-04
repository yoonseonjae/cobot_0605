# Headless probe: find which RTX Lidar config string actually CREATES a sensor
# in this 5.1 build. Ground truth = the IsaacSensorCreateRtxLidar command returns
# success. We want a 2D planar lidar for Nav2 (SICK TiM family is the classic pick).
# Run via run_isaac.sh.
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.sensors.rtx")
enable_extension("isaacsim.core.nodes")

from isaacsim.core.api import World
world = World()
world.reset()
for _ in range(10):
    simulation_app.update()

import omni.kit.commands
from pxr import Gf

# Candidate config strings, best-for-Nav2 (2D) first.
candidates = [
    "TIM781",          # SICK TiM 2D lidar (folder name)
    "tim781",          # file-name variant
    "SICK_TiM781",
    "Sick_Tim781",
    "picoScan150",     # SICK 2D-ish
    "microScan3",
    "Example_Rotary",  # generic fallback
]

print("\n========== RTX LIDAR CONFIG CREATE TEST ==========")
working = []
for i, cfg in enumerate(candidates):
    path = f"Probe_{i}"
    try:
        success, prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path=path,
            parent="/World",
            config=cfg,
            translation=Gf.Vec3d(0.0, 0.0, 0.2),
            orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
        )
        if success:
            print(f"   OK    config='{cfg}'   -> {prim}")
            working.append(cfg)
        else:
            print(f"   FAIL  config='{cfg}'   (command returned success=False)")
    except Exception as e:
        print(f"   ERR   config='{cfg}'   ({str(e).splitlines()[0][:70]})")

print("\n>>> WORKING CONFIGS:", working if working else "NONE")
print("==================================================\n")
simulation_app.close()
