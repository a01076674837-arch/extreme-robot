"""Static and feedback-gating tests for the fixed-yaw four-axis arm."""

import threading
import time
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import yaml

from dynamixel_control.arm_hardware import (
    ARM_COMMAND_CALIBRATED, ARM_FIXED_YAW_CALIBRATED,
    ARM_JOINT_CONFIG, ARM_JOINT_NAMES, ARM_MOTOR_IDS,
    load_srdf_group_state,
)
from dynamixel_control import arm_fsm_node
from dynamixel_control import keyboard_teleop_node
from dynamixel_control import moveit_dynamixel_bridge as bridge_module
from dynamixel_control import teleop_core_node


SRC = Path(__file__).resolve().parents[2]
MOVEIT = SRC / "robot_arm_moveit_config" / "config"
DESCRIPTION = SRC / "robot_arm_description"


class FakeSyncRead:
    """Record IDs registered for a SyncRead packet."""

    def __init__(self):
        self.ids = []

    def addParam(self, dxl_id):
        self.ids.append(dxl_id)
        return True


class FakeLogger:
    """Accept log calls made by command rejection paths."""

    def warn(self, _message):
        pass

    def error(self, _message):
        pass


def test_four_axis_configuration_is_shared_by_all_control_nodes():
    assert ARM_JOINT_NAMES == [
        "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]
    assert ARM_MOTOR_IDS == [14, 13, 12, 16]
    assert list(ARM_JOINT_CONFIG) == ARM_JOINT_NAMES
    assert arm_fsm_node.ARM_JOINT_NAMES == ARM_JOINT_NAMES
    assert teleop_core_node.DEFAULT_JOINT_NAMES == ARM_JOINT_NAMES
    assert teleop_core_node.DEFAULT_MOTOR_IDS == ARM_MOTOR_IDS
    assert keyboard_teleop_node.ARM_JOINT_NAMES == ARM_JOINT_NAMES
    assert bridge_module.JOINT_CONFIG == ARM_JOINT_CONFIG


def test_uncalibrated_motion_is_explicitly_blocked():
    assert not ARM_COMMAND_CALIBRATED
    assert not ARM_FIXED_YAW_CALIBRATED
    assert arm_fsm_node.ANALYTIC_JOINT_NAMES == []


def test_trajectory_success_gate_requires_fresh_complete_feedback():
    bridge = object.__new__(bridge_module.MoveItDynamixelBridge)
    bridge._feedback_lock = threading.Lock()
    bridge._latest_arm_positions = {name: 0.0 for name in ARM_JOINT_NAMES}
    bridge._latest_arm_feedback_time = time.monotonic()
    bridge.trajectory_feedback_timeout = 0.5
    bridge.trajectory_goal_tolerance = 0.03
    target = {name: 0.02 for name in ARM_JOINT_NAMES}
    assert bridge._arm_goal_reached(target)

    bridge._latest_arm_positions.pop("arm_joint_5")
    assert not bridge._arm_goal_reached(target)
    bridge._latest_arm_positions["arm_joint_5"] = 0.02
    bridge._latest_arm_feedback_time = time.monotonic() - 1.0
    assert not bridge._arm_goal_reached(target)


def test_read_only_registers_four_arm_and_two_gripper_ids_for_sync_read():
    bridge = object.__new__(bridge_module.MoveItDynamixelBridge)
    bridge.group_sync_read = FakeSyncRead()
    bridge.active_ids = set()
    bridge.gripper_ids = [3, 4]
    bridge._register_read_only_motors()
    assert bridge.group_sync_read.ids == [14, 13, 12, 16, 3, 4]
    assert bridge.active_ids == {14, 13, 12, 16, 3, 4}
    assert bridge_module.ADDR_SYNC_READ_START \
        == bridge_module.ADDR_TORQUE_ENABLE


def test_read_only_and_fixed_yaw_commands_are_rejected():
    bridge = object.__new__(bridge_module.MoveItDynamixelBridge)
    bridge.read_only = True
    bridge.gripper_only_mode = False
    bridge.get_logger = lambda: FakeLogger()
    assert bridge.goal_callback(None) == bridge_module.GoalResponse.REJECT

    arm_goal = SimpleNamespace(trajectory=SimpleNamespace())
    assert bridge.arm_goal_callback(arm_goal) \
        == bridge_module.GoalResponse.REJECT
    bridge._write_arm_point = lambda *_args: (_ for _ in ()).throw(
        AssertionError("read-only trajectory attempted a write"))
    bridge.trajectory_callback(SimpleNamespace(points=[object()]))
    bridge.teleop_goal_callback(SimpleNamespace(data=[14, 2048]))

    bridge.read_only = False
    fixed_goal = SimpleNamespace(trajectory=SimpleNamespace(
        joint_names=["arm_joint_1"], points=[SimpleNamespace(
            positions=[0.0],
            time_from_start=SimpleNamespace(sec=1, nanosec=0))]))
    assert bridge.arm_goal_callback(fixed_goal) \
        == bridge_module.GoalResponse.REJECT


def test_bridge_requires_a_complete_four_axis_trajectory():
    bridge = object.__new__(bridge_module.MoveItDynamixelBridge)
    bridge.read_only = False
    bridge.gripper_only_mode = False
    bridge.active_ids = set(ARM_MOTOR_IDS)
    bridge.get_logger = lambda: FakeLogger()
    partial = SimpleNamespace(trajectory=SimpleNamespace(
        joint_names=ARM_JOINT_NAMES[:-1], points=[SimpleNamespace(
            positions=[0.0] * 3,
            time_from_start=SimpleNamespace(sec=1, nanosec=0))]))
    assert bridge.arm_goal_callback(partial) \
        == bridge_module.GoalResponse.REJECT


def test_urdf_srdf_and_controller_joint_lists_are_consistent():
    urdf = ET.parse(DESCRIPTION / "urdf" / "robot_arm.urdf").getroot()
    joints = {joint.get("name"): joint for joint in urdf.findall("joint")}
    assert joints["arm_joint_1"].get("type") == "fixed"
    assert [name for name in ARM_JOINT_NAMES
            if joints[name].get("type") in ("revolute", "continuous")] \
        == ARM_JOINT_NAMES

    srdf = ET.parse(MOVEIT / "robot_arm.srdf").getroot()
    for state_name in ("home", "stow"):
        state = srdf.find(
            f"./group_state[@name='{state_name}'][@group='arm']")
        assert [joint.get("name") for joint in state.findall("joint")] \
            == ARM_JOINT_NAMES

    ros2 = yaml.safe_load((MOVEIT / "ros2_controllers.yaml").read_text())
    moveit = yaml.safe_load((MOVEIT / "moveit_controllers.yaml").read_text())
    assert ros2["arm_controller"]["ros__parameters"]["joints"] \
        == ARM_JOINT_NAMES
    manager = moveit["moveit_simple_controller_manager"]
    assert manager["arm_controller"]["joints"] == ARM_JOINT_NAMES

    control_text = (MOVEIT / "robot_arm.ros2_control.xacro").read_text()
    assert '<joint name="arm_joint_1">' not in control_text
    assert all(f'<joint name="{name}">' in control_text
               for name in ARM_JOINT_NAMES)


def test_stow_target_is_complete_four_axis_srdf_state():
    assert len(load_srdf_group_state("arm", "stow")) == 4


def test_fsm_state_set_is_unchanged():
    assert [state.name for state in arm_fsm_node.State] == [
        "IDLE", "PERCEIVE", "PLAN", "APPROACH", "DESCEND", "GRASP",
        "GRASP_CHECK", "LIFT", "CARRY", "RELEASE", "DONE", "FAILED",
        "GRIP_LOST", "LOWER_RELEASE", "STOWING", "STOWED_LOCKED",
        "LOCKED",
    ]
