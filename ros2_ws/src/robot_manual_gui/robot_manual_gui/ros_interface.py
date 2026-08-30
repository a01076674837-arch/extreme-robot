"""ROS-facing half of the manual GUI; contains no Dynamixel register access."""

import json
import math

from PyQt5.QtCore import QObject, pyqtSignal
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from robot_arm_msgs.msg import ArmStatus
from dynamixel_control.tool_profiles import validate_control_scope


ARM_JOINTS = [f'arm_joint_{index}' for index in range(1, 6)]


class GuiSignals(QObject):
    joint_states = pyqtSignal(dict)
    tool_status = pyqtSignal(dict)
    fsm_state = pyqtSignal(str)
    control_mode = pyqtSignal(str)
    arm_status = pyqtSignal(str)
    contact_status = pyqtSignal(bool)
    log = pyqtSignal(str)
    gripper_state = pyqtSignal(bool, str)


class ManualGuiNode(Node):
    """Translate UI intent into existing high-level ROS interfaces."""

    def __init__(self, signals):
        super().__init__('robot_manual_gui')
        self.signals = signals
        self.declare_parameter('mock_mode', False)
        self.declare_parameter('tool_type', 'spur_1motor_gripper')
        self.declare_parameter('control_scope', 'FULL_ROBOT')
        self.declare_parameter('temporary_jog_mode', False)
        self.declare_parameter('temporary_jog_safe_min_tick', 2867)
        self.declare_parameter('temporary_jog_safe_max_tick', 3807)
        self.declare_parameter('temporary_jog_mechanical_open_tick', 2817)
        self.declare_parameter('temporary_jog_mechanical_close_tick', 3857)
        self.mock_mode = bool(self.get_parameter('mock_mode').value)
        self.selected_tool = str(self.get_parameter('tool_type').value)
        self.control_scope = validate_control_scope(
            self.get_parameter('control_scope').value)
        self.temporary_jog_mode = bool(
            self.get_parameter('temporary_jog_mode').value)
        self.temporary_jog_safe_min = int(
            self.get_parameter('temporary_jog_safe_min_tick').value)
        self.temporary_jog_safe_max = int(
            self.get_parameter('temporary_jog_safe_max_tick').value)
        self.positions = {name: 0.0 for name in ARM_JOINTS}
        self.efforts = {name: 0.0 for name in ARM_JOINTS}
        self.control_mode = 'FSM'
        self.fsm_state = 'UNKNOWN'
        self.last_gripper_goal = None
        self.gripper_busy = False

        self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)
        self.create_subscription(String, '/tool/status', self._tool_cb, 10)
        self.create_subscription(String, '/fsm/state', self._fsm_cb, 10)
        self.create_subscription(
            String, '/control/mode_status', self._mode_cb, 10)
        self.create_subscription(ArmStatus, '/arm_status', self._arm_status_cb, 10)
        self.create_subscription(
            Bool, '/sensors/contact_status',
            lambda msg: self.signals.contact_status.emit(bool(msg.data)), 10)

        self.arm_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.cleaner_pub = self.create_publisher(Bool, '/cleaning/enable', 10)
        self.estop_pub = self.create_publisher(Bool, '/tool/emergency_stop', 10)
        self.detach_pub = self.create_publisher(Bool, '/tool/detached', 10)
        self.mode_pub = self.create_publisher(String, '/control/mode', 10)
        self.gripper = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory')

    def _joint_cb(self, msg):
        values = {}
        for index, name in enumerate(msg.name):
            position = msg.position[index] if index < len(msg.position) else None
            effort = msg.effort[index] if index < len(msg.effort) else None
            values[name] = {'position': position, 'effort': effort}
            if name in self.positions and position is not None:
                self.positions[name] = float(position)
            if name in self.efforts and effort is not None:
                self.efforts[name] = float(effort)
        self.signals.joint_states.emit(values)

    def _tool_cb(self, msg):
        try:
            self.signals.tool_status.emit(json.loads(msg.data))
        except ValueError:
            self.signals.log.emit('Invalid JSON received on /tool/status')

    def _fsm_cb(self, msg):
        self.fsm_state = msg.data
        self.signals.fsm_state.emit(msg.data)

    def _mode_cb(self, msg):
        self.control_mode = msg.data
        self.signals.control_mode.emit(msg.data)

    def _arm_status_cb(self, msg):
        self.signals.arm_status.emit(msg.status)

    def request_mode(self, mode):
        requested = mode.upper()
        self.get_logger().info(f'Publishing control mode request: {requested}')
        self.mode_pub.publish(String(data=requested))

    def jog_arm(self, joint, delta_deg):
        target = self.positions.get(joint, 0.0) + math.radians(delta_deg)
        self.command_arm(joint, target)

    def command_arm(self, joint, target_rad):
        if self.control_scope == 'END_EFFECTOR_ONLY':
            self.signals.log.emit(
                'Arm command blocked: control scope is END_EFFECTOR_ONLY')
            return
        if self.control_mode != 'MANUAL':
            self.signals.log.emit('Arm command blocked: ownership is not MANUAL')
            return
        trajectory = JointTrajectory()
        trajectory.joint_names = [joint]
        point = JointTrajectoryPoint()
        point.positions = [float(target_rad)]
        point.time_from_start = Duration(sec=1)
        trajectory.points = [point]
        self.arm_pub.publish(trajectory)
        if self.mock_mode:
            self.positions[joint] = float(target_rad)
            self.signals.joint_states.emit({
                joint: {'position': float(target_rad), 'effort': 0.0}})

    def command_gripper(self, position):
        if self.control_mode != 'MANUAL':
            self.signals.log.emit('Gripper command blocked: ownership is not MANUAL')
            return False
        if self.gripper_busy:
            self.signals.log.emit('Gripper command blocked: BUSY')
            return False
        if (self.temporary_jog_mode and self.control_scope == 'END_EFFECTOR_ONLY'
                and self.selected_tool == 'spur_1motor_gripper'):
            target = int(round(position))
            if not (self.temporary_jog_safe_min <= target
                    <= self.temporary_jog_safe_max):
                self.signals.log.emit(
                    f'Gripper jog blocked: target={target} outside '
                    f'[{self.temporary_jog_safe_min}, '
                    f'{self.temporary_jog_safe_max}]')
                return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['gripper_drive_joint']
        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        point.time_from_start = Duration(sec=2)
        goal.trajectory.points = [point]
        self.gripper_busy = True
        self.signals.gripper_state.emit(True, 'SENDING')
        self.get_logger().info(
            f'Sending gripper goal: logical_position={float(position):.9f}')
        future = self.gripper.send_goal_async(goal)
        future.add_done_callback(self._gripper_goal_response)
        return True

    def _gripper_goal_response(self, future):
        try:
            self.last_gripper_goal = future.result()
            if not self.last_gripper_goal.accepted:
                self.signals.log.emit('Gripper action rejected by bridge')
                self._set_gripper_idle('REJECTED')
                return
            self.signals.log.emit('Gripper action accepted')
            self.signals.gripper_state.emit(True, 'ACTIVE')
            self.last_gripper_goal.get_result_async().add_done_callback(
                self._gripper_result)
        except Exception as exc:
            self.signals.log.emit(f'Gripper action error: {exc}')
            self._set_gripper_idle('ERROR')

    def _gripper_result(self, future):
        try:
            wrapped = future.result()
            result = wrapped.result
            self.signals.log.emit(
                f'Gripper result: status={wrapped.status}, '
                f'error_code={result.error_code}, '
                f'message={result.error_string}')
            state = ('SUCCEEDED' if result.error_code ==
                     FollowJointTrajectory.Result.SUCCESSFUL else 'FAILED')
        except Exception as exc:
            self.signals.log.emit(f'Gripper result error: {exc}')
            state = 'ERROR'
        self._set_gripper_idle(state)

    def _set_gripper_idle(self, state):
        self.gripper_busy = False
        self.last_gripper_goal = None
        self.signals.gripper_state.emit(False, state)

    def stop_gripper(self):
        if not self.gripper_busy or self.last_gripper_goal is None:
            self.signals.log.emit('Gripper STOP: no active goal')
            return False
        self.signals.gripper_state.emit(True, 'STOPPING')
        self.signals.log.emit('Gripper STOP requested')
        future = self.last_gripper_goal.cancel_goal_async()
        future.add_done_callback(self._gripper_cancel_response)
        return True

    def _gripper_cancel_response(self, future):
        try:
            response = future.result()
            accepted = bool(response.goals_canceling)
            self.signals.log.emit(
                f'Gripper STOP accepted={accepted}; waiting for result')
            if not accepted:
                self._set_gripper_idle('STOP_REJECTED')
        except Exception as exc:
            self.signals.log.emit(f'Gripper STOP error: {exc}')
            self._set_gripper_idle('STOP_ERROR')

    def command_cleaner(self, enabled):
        if self.control_mode != 'MANUAL':
            self.signals.log.emit('Cleaner command blocked: ownership is not MANUAL')
            return
        self.cleaner_pub.publish(Bool(data=bool(enabled)))

    def emergency_stop(self):
        # Hold the manually commanded arm at the latest measured positions using
        # the existing trajectory interface, then invoke the shared tool E-stop.
        if (self.control_mode == 'MANUAL'
                and self.control_scope == 'FULL_ROBOT'):
            trajectory = JointTrajectory()
            trajectory.joint_names = list(ARM_JOINTS)
            point = JointTrajectoryPoint()
            point.positions = [self.positions[name] for name in ARM_JOINTS]
            point.time_from_start = Duration(nanosec=100000000)
            trajectory.points = [point]
            self.arm_pub.publish(trajectory)
        self.cleaner_pub.publish(Bool(data=False))
        self.estop_pub.publish(Bool(data=True))

    def tool_detached(self):
        self.detach_pub.publish(Bool(data=True))
