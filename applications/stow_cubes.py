# Fake Grasp Pick & Place with gripper-driven sequences.
# Two stowage cubes ride on Spot's back (mask=blue, extinguisher=red).
# Press 1/2 to PICK mask/extinguisher, 3/4 to PLACE (release) them.
# Each press runs an automatic sequence: open gripper -> move arm -> close gripper
# -> fake-grasp(reparent under EE)  /  move to place -> open gripper -> release(+physics).
# Run via run_isaac.sh.
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
from isaacsim.core.utils.types import ArticulationAction
from spot_policy import SpotArmFlatTerrainPolicy

# Arm joint indices in the 19-DOF order (from check_arm_limits.py).
ARM = {"sh0": 1, "sh1": 0, "el0": 2, "el1": 7, "wr0": 12, "wr1": 17, "f1x": 18}
ARM_NO_GRIP = [k for k in ARM if k != "f1x"]

GRIPPER_OPEN = -1.5     # f1x offset: open
GRIPPER_CLOSED = 0.0    # f1x offset: closed

# Per-target arm poses (offsets from default, rad), captured via arm_pose_explorer.
# Gripper (f1x) is handled separately by the sequence, so it's omitted here.
HOME_ARM = {k: 0.0 for k in ARM_NO_GRIP}
GRASP_ARM = {
    "mask":         {"sh0":  2.8, "sh1": 1.0, "el0": -1.0, "el1": 0.0, "wr0": 1.0, "wr1": 0.0},
    "extinguisher": {"sh0": -2.8, "sh1": 1.0, "el0": -1.0, "el1": 0.0, "wr0": 1.0, "wr1": 0.0},
}
PLACE_ARM = {
    "mask":         {"sh0": 0.0, "sh1": 1.0, "el0": -1.0, "el1": 0.0, "wr0": 1.0, "wr1": 0.0},
    "extinguisher": {"sh0": 0.0, "sh1": 2.0, "el0": -2.5, "el1": 0.0, "wr0": 1.0, "wr1": 0.0},
}

# Cubes on the back (body-local frame).
CUBE_SIZE = 0.08
CUBES = {
    "mask":         {"color": (0.1, 0.3, 1.0), "pos": [0.0,  0.12, 0.20]},  # blue
    "extinguisher": {"color": (1.0, 0.1, 0.1), "pos": [0.0, -0.12, 0.20]},  # red
}

LERP_RATE = 0.03          # rad/step the arm chases its target
POSE_TOL = 0.05           # |current-target| under this (all joints) = "arrived"
DWELL_STEPS = 60          # hold each pose this many physics steps before next stage


class StowRunner(object):
    def __init__(self, physics_dt, render_dt):
        self._world = World(stage_units_in_meters=1.0, physics_dt=physics_dt, rendering_dt=render_dt)
        BASE_DIR = Path(__file__).resolve().parent.parent

        from pxr import UsdLux, Sdf
        import omni.usd
        self._stage = omni.usd.get_context().get_stage()
        dome = UsdLux.DomeLight.Define(self._stage, Sdf.Path("/World/DomeLight"))
        dome.CreateIntensityAttr(1000.0)

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
        self._world.scene.add(self._spot.robot)
        self._make_cubes()

        self._base_command = np.zeros(3)
        # Current (lerped) offset applied to every arm joint incl. gripper.
        self._cur = {k: 0.0 for k in ARM}
        # Target the arm chases right now (set by the active sequence step).
        self._tgt = {k: 0.0 for k in ARM}
        self._cur["f1x"] = 0.0
        self._tgt["f1x"] = 0.0

        self._seq = []            # list of pending steps (dicts)
        self._step = None         # current step
        self._dwell = 0           # how long we've held the current arrived pose
        self._fired = False       # whether on_arrive already ran for this step
        self._held = {}           # cube_name -> True if grasped
        self._ee_path = "/World/Spot/arm0_link_fngr"

        self.first_step = True
        self.needs_reset = False
        self._base_locked = False
        self._lock_pose = None
        self._lock_legs = None    # captured leg joint angles while frozen
        self._leg_idx = None

    # ---------------- scene setup ----------------
    def _make_cubes(self):
        # Visual + collision only while carried (no RigidBody: nested rigid bodies
        # under /World/Spot/body are illegal). Physics is added on release.
        from pxr import UsdGeom, Gf
        for name, cfg in CUBES.items():
            path = f"/World/Spot/body/Cube_{name}"
            cube = UsdGeom.Cube.Define(self._stage, path)
            cube.CreateSizeAttr(CUBE_SIZE)
            cube.CreateDisplayColorAttr([Gf.Vec3f(*cfg["color"])])
            xf = UsdGeom.Xformable(cube.GetPrim())
            xf.ClearXformOpOrder()
            xf.AddTranslateOp().Set(Gf.Vec3d(*cfg["pos"]))
            print(f"[cube] {name} at body-local {cfg['pos']} (visual)")

    def _reset_cubes(self):
        # Delete cubes wherever they ended up, then recreate them on the back.
        import omni.kit.commands
        for name in CUBES:
            for parent in ("/World/Spot/body", self._ee_path, "/World"):
                p = f"{parent}/Cube_{name}"
                if self._stage.GetPrimAtPath(p).IsValid():
                    omni.kit.commands.execute("DeletePrims", paths=[p])
        self._make_cubes()

    # ---------------- physics helpers ----------------
    def _add_physics(self, prim_path):
        from pxr import UsdPhysics
        prim = self._stage.GetPrimAtPath(prim_path)
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.RigidBodyAPI.Apply(prim)

    def _remove_physics(self, prim_path):
        from pxr import UsdPhysics
        prim = self._stage.GetPrimAtPath(prim_path)
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            prim.RemoveAPI(UsdPhysics.CollisionAPI)

    def _reparent_keep_transform(self, src_path, new_parent_path):
        from pxr import UsdGeom
        import omni.kit.commands
        src = self._stage.GetPrimAtPath(src_path)
        world_xf = UsdGeom.Xformable(src).ComputeLocalToWorldTransform(0)
        leaf = src_path.split("/")[-1]
        dst_path = f"{new_parent_path}/{leaf}"
        omni.kit.commands.execute("MovePrim", path_from=src_path, path_to=dst_path)
        parent = self._stage.GetPrimAtPath(new_parent_path)
        parent_xf = UsdGeom.Xformable(parent).ComputeLocalToWorldTransform(0)
        new_local = world_xf * parent_xf.GetInverse()
        moved = self._stage.GetPrimAtPath(dst_path)
        xf = UsdGeom.Xformable(moved)
        xf.ClearXformOpOrder()
        for attr in list(moved.GetAttributes()):
            if attr.GetName().startswith("xformOp:"):
                moved.RemoveProperty(attr.GetName())
        xf.AddTransformOp().Set(new_local)
        return dst_path

    # ---------------- fake grasp / release ----------------
    def _do_grasp(self, name):
        # cube currently lives on the back; reparent under EE.
        src = f"/World/Spot/body/Cube_{name}"
        if not self._stage.GetPrimAtPath(src).IsValid():
            print(f"[grasp] {name} not on back (already held?)")
            return
        self._remove_physics(src)
        self._reparent_keep_transform(src, self._ee_path)
        self._held[name] = True
        print(f"[grasp] '{name}' now child of EE")

    def _do_release(self, name):
        src = f"{self._ee_path}/Cube_{name}"
        if not self._stage.GetPrimAtPath(src).IsValid():
            print(f"[release] {name} not held")
            return
        dst = self._reparent_keep_transform(src, "/World")
        self._add_physics(dst)
        self._held[name] = False
        print(f"[release] '{name}' dropped (physics ON)")

    # ---------------- sequences ----------------
    def _pick_seq(self, name):
        # open gripper -> move arm to grasp pose -> close gripper -> reparent
        grasp = GRASP_ARM[name]
        return [
            {"arm": grasp,    "grip": GRIPPER_OPEN,   "note": f"open+reach {name}"},
            {"arm": grasp,    "grip": GRIPPER_CLOSED, "note": f"close on {name}",
             "on_arrive": lambda n=name: self._do_grasp(n)},
            {"arm": HOME_ARM, "grip": GRIPPER_CLOSED, "note": "carry to home"},
        ]

    def _place_seq(self, name):
        # move to place pose (gripper closed, holding) -> open -> release
        place = PLACE_ARM[name]
        return [
            {"arm": place,    "grip": GRIPPER_CLOSED, "note": f"reach place {name}"},
            {"arm": place,    "grip": GRIPPER_OPEN,   "note": f"open / release {name}",
             "on_arrive": lambda n=name: self._do_release(n)},
            # Close the gripper again on the way home so HOME always looks the same
            # as the initial pose (default = closed).
            {"arm": HOME_ARM, "grip": GRIPPER_CLOSED, "note": "back to home (gripper closed)"},
        ]

    def _start(self, seq, label):
        self._seq = list(seq)
        self._step = None
        print(f"[seq] start: {label}")

    def _advance_seq(self):
        # Called each physics step. Pull a step if none active; check arrival.
        if self._step is None:
            if not self._seq:
                return
            self._step = self._seq.pop(0)
            for j in ARM_NO_GRIP:
                self._tgt[j] = self._step["arm"][j]
            self._tgt["f1x"] = self._step["grip"]
            self._dwell = 0
            self._fired = False
            print(f"[seq] -> {self._step['note']}")
            return
        # Arrived? Require the pose to be held for DWELL_STEPS so each stage is
        # visibly distinct (e.g. gripper finishes opening before the arm leaves).
        err = max(abs(self._cur[j] - self._tgt[j]) for j in ARM)
        if err < POSE_TOL:
            # Fire the action once, right when we first arrive.
            if not self._fired and "on_arrive" in self._step:
                self._step["on_arrive"]()
            self._fired = True
            self._dwell += 1
            if self._dwell >= DWELL_STEPS:
                self._step = None  # advance to next step next call
        else:
            self._dwell = 0

    # ---------------- main loop ----------------
    def setup(self):
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._sub = self._input.subscribe_to_keyboard_events(self._keyboard, self._on_key)
        self._world.add_physics_callback("step", callback_fn=self.on_physics_step)
        print("\n========== FAKE GRASP PICK & PLACE ==========")
        print("   NUMPAD_* : toggle base lock (freeze in air)")
        print("   1 : PICK mask      2 : PICK extinguisher")
        print("   3 : PLACE mask     4 : PLACE extinguisher")
        print("   P : print EE vs cube distances")
        print("=============================================\n")

    def on_physics_step(self, step_size):
        if self.needs_reset:
            return

        if self.first_step:
            self._spot.initialize()
            self._spot.robot.set_joint_positions(self._spot.default_pos)
            self._spot.robot.set_joint_velocities(self._spot.default_vel)
            if self._leg_idx is None:
                self._leg_idx = np.array([i for i in range(len(self._spot.robot.dof_names))
                                          if i not in ARM.values()])
            self.first_step = False
            return

        if self._base_locked:
            # Mode A: freeze the robot like a statue. Walk policy is NOT run while
            # locked (it would fight the freeze). Only the arm sequence moves.
            # Base is physically pinned to the world via FixedJoint (created in _on_key).
            pass
        else:
            self._lock_pose = None
            # Walk policy drives the legs (zero command = stand/idle in place).
            self._spot.forward(step_size, self._base_command)

        # Run the active sequence (sets targets, fires grasp/release on arrival).
        self._advance_seq()

        # Lerp every arm joint (incl gripper) toward its target.
        for j in ARM:
            d = self._tgt[j] - self._cur[j]
            self._cur[j] += max(-LERP_RATE, min(LERP_RATE, d))

        if self._base_locked:
            # Combine leg lock targets and arm targets into ONE action array
            # so they don't overwrite each other in the wrapper.
            all_targets = np.zeros(len(self._spot.robot.dof_names))
            all_targets[self._leg_idx] = self._lock_legs[self._leg_idx]
            for j, i in ARM.items():
                all_targets[i] = self._spot.default_pos[i] + self._cur[j]
            self._spot.robot.apply_action(ArticulationAction(joint_positions=all_targets))
        else:
            # Override just the arm targets (legs were set by policy forward())
            arm_idx = list(ARM.values())
            arm_targets = np.array([self._spot.default_pos[i] + self._cur[j]
                                    for j, i in ARM.items()])
            self._spot.robot.apply_action(
                ArticulationAction(joint_positions=arm_targets, joint_indices=np.array(arm_idx)))

    def _report(self):
        def wp(path):
            p = self._stage.GetPrimAtPath(path)
            if not p.IsValid():
                return None
            from pxr import UsdGeom
            m = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(0)
            t = m.ExtractTranslation()
            return np.array([t[0], t[1], t[2]])
        ee = wp(self._ee_path)
        print(f"\n--- EE world = {np.round(ee,3)} ---")
        for name in CUBES:
            for parent in ("/World/Spot/body", self._ee_path, "/World"):
                c = wp(f"{parent}/Cube_{name}")
                if c is not None:
                    where = parent.split('/')[-1] or "World"
                    d = np.linalg.norm(ee - c) if ee is not None else -1
                    print(f"  {name:<13} @{where:<6} dist={d:.3f}")
                    break
        print("--------------------\n")

    def _on_key(self, event, *args, **kwargs):
        if event.type != carb.input.KeyboardEventType.KEY_PRESS:
            return True
        n = event.input.name
        # Number-row keys arrive as "KEY_1"/"1"; accept all variants + numpad.
        def is_num(k):
            return n in (k, f"KEY_{k}", f"NUMPAD_{k}")
        if n in ("NUMPAD_MULTIPLY", "ASTERISK"):
            self._base_locked = not self._base_locked
            self._lock_pose = None
            if self._base_locked:
                self._create_base_lock()
                self._lock_legs = self._spot.robot.get_joint_positions().copy()
            else:
                self._destroy_base_lock()
                self._lock_legs = None
            print(f"[base lock] {'ON' if self._base_locked else 'OFF'}")
        elif is_num("1"):
            self._start(self._pick_seq("mask"), "PICK mask")
        elif is_num("2"):
            self._start(self._pick_seq("extinguisher"), "PICK extinguisher")
        elif is_num("3"):
            self._start(self._place_seq("mask"), "PLACE mask")
        elif is_num("4"):
            self._start(self._place_seq("extinguisher"), "PLACE extinguisher")
        elif n in ("P", "KEY_P"):
            self._report()
        else:
            print(f"[key] unhandled: {n}")   # debug: shows the real key name
        return True

    def _create_base_lock(self):
        from pxr import UsdPhysics, Gf, UsdGeom
        joint_path = "/World/Spot/body/LockJoint"
        if not self._stage.GetPrimAtPath(joint_path).IsValid():
            joint = UsdPhysics.FixedJoint.Define(self._stage, joint_path)
            joint.CreateBody1Rel().SetTargets(["/World/Spot/body"])
            
            # Use the exact world transform of the body prim, not the articulation root
            prim = self._stage.GetPrimAtPath("/World/Spot/body")
            xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0.0)
            pos = xf.ExtractTranslation()
            rot = xf.ExtractRotationQuat()
            
            joint.CreateLocalPos0Attr().Set(Gf.Vec3f(float(pos[0]), float(pos[1]), float(pos[2])))
            joint.CreateLocalRot0Attr().Set(Gf.Quatf(float(rot.GetReal()), float(rot.GetImaginary()[0]), float(rot.GetImaginary()[1]), float(rot.GetImaginary()[2])))
            joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
            joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

    def _destroy_base_lock(self):
        import omni.kit.commands
        joint_path = "/World/Spot/body/LockJoint"
        if self._stage.GetPrimAtPath(joint_path).IsValid():
            omni.kit.commands.execute("DeletePrims", paths=[joint_path])

    def run(self):
        while simulation_app.is_running():
            if self._world.is_playing() and self.needs_reset:
                self._base_locked = False
                self._destroy_base_lock()
                self._world.reset(True)
                self.needs_reset = False
                self.first_step = True
                self._cur = {k: 0.0 for k in ARM}
                self._tgt = {k: 0.0 for k in ARM}
                self._seq, self._step = [], None
                self._dwell, self._fired = 0, False
                self._held = {}
                self._lock_pose = None
                self._lock_legs = None
                self._reset_cubes()
                print("[reset] scene restored to initial state")

            self._world.step(render=True)

            if self._world.is_stopped():
                self.needs_reset = True
                if self._base_locked:
                    self._base_locked = False
                    self._destroy_base_lock()


def main():
    runner = StowRunner(physics_dt=1/200.0, render_dt=1/60.0)
    simulation_app.update()
    runner._world.reset()
    simulation_app.update()
    runner.setup()
    simulation_app.update()
    runner.run()
    simulation_app.close()


if __name__ == "__main__":
    main()
