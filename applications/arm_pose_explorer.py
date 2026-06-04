# Interactive arm-pose explorer: walk policy keeps the legs/base alive while we
# tweak the 7 arm joints with the keyboard and watch the EE position vs the back
# (body) in real time. Goal: find a "fold arm over the back" pose so we know where
# to mount the stowage cubes. Run via run_isaac.sh (no ROS2 needed, but fine).
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import carb
import numpy as np
import os
from pathlib import Path
import omni.appwindow

from isaacsim.core.api import World
from isaacsim.core.utils.prims import define_prim
from isaacsim.core.prims import SingleXFormPrim
from spot_policy import SpotArmFlatTerrainPolicy

# Arm joint indices in the 19-DOF order (from check_arm_limits.py).
ARM = {
    "sh0": 1, "sh1": 0, "el0": 2, "el1": 7, "wr0": 12, "wr1": 17, "f1x": 18,
}
# Avoid Isaac Sim viewport hotkeys (W/E/R/F/T/Q/S/G... are camera/gizmo keys).
# Instead: select the active joint with NUMPAD_1..7, then nudge it with
# NUMPAD_PLUS / NUMPAD_MINUS. Numpad keys aren't bound by the viewport.
STEP = 0.1
# Move the arm toward its target slowly so the walk policy can rebalance and the
# robot doesn't tip over from a sudden shift in center of mass.
LERP_RATE = 0.02  # rad per physics step, per joint (max)
JOINT_ORDER = ["sh0", "sh1", "el0", "el1", "wr0", "wr1", "f1x"]
# NUMPAD_1..6 select an arm joint to nudge. NUMPAD_7 is a dedicated gripper
# OPEN/CLOSE toggle (f1x: 0 = closed, -1.571 = open).
SELECT_KEYS = {
    "NUMPAD_1": "sh0", "NUMPAD_2": "sh1", "NUMPAD_3": "el0", "NUMPAD_4": "el1",
    "NUMPAD_5": "wr0", "NUMPAD_6": "wr1",
}
GRIPPER_OPEN = -1.5   # offset that opens the gripper (near lower limit -1.571)
GRIPPER_CLOSED = 0.0
INC_KEYS = {"NUMPAD_ADD", "EQUAL"}        # numpad '+' or main-row '='
DEC_KEYS = {"NUMPAD_SUBTRACT", "MINUS"}   # numpad '-' or main-row '-'


class ArmExplorer(object):
    def __init__(self, physics_dt, render_dt):
        self._world = World(stage_units_in_meters=1.0, physics_dt=physics_dt, rendering_dt=render_dt)

        BASE_DIR = Path(__file__).resolve().parent.parent

        from pxr import UsdLux, Sdf
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/DomeLight"))
        dome.CreateIntensityAttr(1000.0)

        # Flat ground plane (added before the robot, same pattern as the examples).
        from isaacsim.core.api.objects.ground_plane import GroundPlane
        GroundPlane(prim_path="/World/GroundPlane", z_position=0.0)

        policy_path = os.path.join(BASE_DIR, "policies/spot_arm/models", "spot_arm_policy.pt")
        policy_params_path = os.path.join(BASE_DIR, "policies/spot_arm/params", "env.yaml")
        usd_path = os.path.join(BASE_DIR, "assets", "spot_arm.usd")

        self._spot = SpotArmFlatTerrainPolicy(
            prim_path="/World/Spot", name="Spot", usd_path=usd_path,
            policy_path=policy_path, policy_params_path=policy_params_path,
            position=np.array([0.0, 0.0, 0.8]),
        )

        # Show the two back cubes (same layout as stow_cubes) so we can find arm
        # poses that reach each one. Visual only.
        from pxr import UsdGeom, Gf
        self._stage = stage
        self._cubes = {"mask": [0.0, 0.12, 0.20], "extinguisher": [0.0, -0.12, 0.20]}
        colors = {"mask": (0.1, 0.3, 1.0), "extinguisher": (1.0, 0.1, 0.1)}
        for nm, pos in self._cubes.items():
            c = UsdGeom.Cube.Define(stage, f"/World/Spot/body/Cube_{nm}")
            c.CreateSizeAttr(0.08)
            c.CreateDisplayColorAttr([Gf.Vec3f(*colors[nm])])
            xf = UsdGeom.Xformable(c.GetPrim())
            xf.ClearXformOpOrder()
            xf.AddTranslateOp().Set(Gf.Vec3d(*pos))

        self._base_command = np.zeros(3)
        # Per-arm-joint target offsets the user dials in (added on top of default).
        self._arm_target = {k: 0.0 for k in ARM}
        # Current (lerped) offset actually applied — chases the target slowly.
        self._arm_current = {k: 0.0 for k in ARM}
        self._active = "sh0"   # currently selected joint
        self.first_step = True
        self.needs_reset = False
        self._base_locked = False     # freeze base in air so the arm can't tip it
        self._lock_pose = None        # (pos, quat) captured when locking
        self._print_counter = 0

    def setup(self):
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._sub = self._input.subscribe_to_keyboard_events(self._keyboard, self._on_key)
        self._world.add_physics_callback("step", callback_fn=self.on_physics_step)
        self._print_help()

    def _print_help(self):
        print("\n========== ARM POSE EXPLORER ==========")
        print("Select a joint with NUMPAD 1..7, then nudge it with + / - (numpad or main row).")
        print("   NUMPAD_1=sh0  2=sh1  3=el0  4=el1  5=wr0  6=wr1")
        print("   NUMPAD_7 : toggle GRIPPER open/close")
        print("   + / - : nudge the SELECTED joint by 0.1 rad")
        print("   P : print current arm angles + EE/back positions")
        print("   numpad_0 : reset all arm offsets to 0")
        print("   NUMPAD_MULTIPLY (*) : toggle BASE LOCK (freeze robot in air; arm can't tip it)")
        print(f"   [active joint = {self._active}]  [base_locked = {self._base_locked}]")
        print("=======================================\n")

    def on_physics_step(self, step_size):
        if self.first_step:
            self._spot.initialize()
            self._spot.robot.set_joint_positions(self._spot.default_pos)
            self._spot.robot.set_joint_velocities(self._spot.default_vel)
            self.first_step = False
            return
        elif self.needs_reset:
            self._world.reset(True)
            self.needs_reset = False
            self.first_step = True
            self._arm_current = {k: 0.0 for k in ARM}
            return

        if self._base_locked:
            # Freeze the base: pin world pose + zero velocities each step, so the
            # arm can move to any pose without tipping. Legs are held at default.
            if self._lock_pose is None:
                pos, quat = self._spot.robot.get_world_pose()
                self._lock_pose = (np.asarray(pos), np.asarray(quat))
            self._spot.robot.set_world_pose(self._lock_pose[0], self._lock_pose[1])
            self._spot.robot.set_linear_velocity(np.zeros(3))
            self._spot.robot.set_angular_velocity(np.zeros(3))
        else:
            # Walk policy computes leg targets internally and applies them; we then
            # add our arm overrides as a SEPARATE targeted action on just the arm DOFs.
            self._lock_pose = None
            self._spot.forward(step_size, self._base_command)

        # Lerp current offset toward the target so the COM shifts gradually.
        for j in ARM:
            delta = self._arm_target[j] - self._arm_current[j]
            step = max(-LERP_RATE, min(LERP_RATE, delta))
            self._arm_current[j] += step

        from isaacsim.core.utils.types import ArticulationAction
        arm_idx = list(ARM.values())
        arm_targets = np.array(
            [self._spot.default_pos[i] + self._arm_current[j] for j, i in ARM.items()]
        )
        self._spot.robot.apply_action(
            ArticulationAction(joint_positions=arm_targets, joint_indices=np.array(arm_idx))
        )

        self._print_counter += 1

    def _report(self):
        def wp(path):
            try:
                pos, _ = SingleXFormPrim(prim_path=path).get_world_pose()
                return np.asarray(pos)
            except Exception:
                return None
        ee = wp("/World/Spot/arm0_link_fngr")
        print("\n--- POSE REPORT ---")
        # Copy-paste-ready dict (rounded). Drop f1x if you only want the arm pose.
        offs = {k: round(self._arm_target[k], 2) for k in ARM}
        print("offsets =", offs)
        # Distance from EE to each back cube (<=0.10 m = good grasp candidate).
        if ee is not None:
            for nm in self._cubes:
                c = wp(f"/World/Spot/body/Cube_{nm}")
                if c is not None:
                    d = np.linalg.norm(ee - c)
                    mark = " <= OK (<=0.10)" if d <= 0.10 else ""
                    print(f"  EE-{nm:<13} dist={d:.3f}{mark}")
        print("-------------------\n")

    def _on_key(self, event, *args, **kwargs):
        if event.type != carb.input.KeyboardEventType.KEY_PRESS:
            return True
        name = event.input.name

        if name == "NUMPAD_7":
            # Toggle gripper open/closed.
            now_open = self._arm_target["f1x"] <= GRIPPER_OPEN / 2.0
            self._arm_target["f1x"] = GRIPPER_CLOSED if now_open else GRIPPER_OPEN
            print(f"[gripper] {'CLOSING' if now_open else 'OPENING'}")
        elif name in SELECT_KEYS:
            self._active = SELECT_KEYS[name]
            print(f"[select] active joint = {self._active}  (offset {self._arm_target[self._active]:+.2f})")
        elif name in INC_KEYS:
            self._arm_target[self._active] += STEP
            print(f"[{self._active}] offset = {self._arm_target[self._active]:+.2f}")
        elif name in DEC_KEYS:
            self._arm_target[self._active] -= STEP
            print(f"[{self._active}] offset = {self._arm_target[self._active]:+.2f}")
        elif name == "P":
            self._report()
        elif name == "NUMPAD_0":
            self._arm_target = {k: 0.0 for k in ARM}
            print("[reset] arm offsets cleared")
        elif name == "NUMPAD_MULTIPLY":
            self._base_locked = not self._base_locked
            self._lock_pose = None
            print(f"[base lock] {'ON (frozen in air)' if self._base_locked else 'OFF (walk policy)'}")
        return True

    def run(self):
        while simulation_app.is_running():
            self._world.step(render=True)
            if self._world.is_stopped():
                self.needs_reset = True


def main():
    runner = ArmExplorer(physics_dt=1/200.0, render_dt=1/60.0)
    simulation_app.update()
    runner._world.reset()
    simulation_app.update()
    runner.setup()
    simulation_app.update()
    runner.run()
    simulation_app.close()


if __name__ == "__main__":
    main()
