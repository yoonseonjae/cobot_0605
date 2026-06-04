# Spot Arm Pick & Place in Isaac Sim (cobot_0605)

This project demonstrates a 19-DOF Spot quadruped robot with an attached robotic arm performing a pick-and-place sequence in NVIDIA Isaac Sim. 

## Key Features

1. **Dynamic Base Lock (Statue Mode)**
   - The robot's mobile base and 12 leg joints can be completely frozen in space during arm operations to prevent unwanted body rotations or physics instability caused by reaction torques.
   - Implemented using a runtime USD `FixedJoint` dynamically generated between the world and the exact body position, perfectly preventing "Physics Explosions".

2. **Smooth Simulation Reset**
   - Safely destroys runtime constraints and physics locks *before* executing the Isaac Sim `world.reset()` step.
   - Fixes the common bug where resetting the simulation while an active FixedJoint is holding the robot causes the PhysX solver to crash or crumple the robot.

3. **Fake Grasp & Lerp Sequences**
   - Uses linear interpolation (Lerp) to smoothly animate the 7-DOF arm.
   - Includes a visual "fake grasp" mechanism that dynamically reparents objects to the End-Effector upon arrival and disables their physics until released.

## Controls
Focus on the Isaac Sim viewport window to use the following hotkeys:

- `*` (Numpad Multiply / Asterisk): **Toggle Base Lock** (Freezes the robot in mid-air/standing position).
- `1`: Start Pick sequence for **Mask**
- `2`: Start Pick sequence for **Extinguisher**
- `3`: Start Place sequence for **Mask**
- `4`: Start Place sequence for **Extinguisher**

## How to Run

```bash
./run_isaac.sh applications/stow_cubes.py
```

## Technical Notes
- **FixedJoint Anchoring**: The Base Lock explicitly grabs the local-to-world transform of `/World/Spot/body` using `UsdGeom.Xformable` instead of the articulation root to prevent snapping.
- **Physics Callback Ordering**: The reset logic intercepts the simulation loop *before* `world.step()` to safely clean up constraints without violating PhysX solver limits.
