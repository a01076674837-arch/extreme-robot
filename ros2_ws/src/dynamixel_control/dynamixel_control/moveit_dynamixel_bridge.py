#!/usr/bin/env python3

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32MultiArray
from control_msgs.action import FollowJointTrajectory
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite, GroupSyncRead

from dynamixel_control.gripper_presets import DEFAULT_GRIPPER, get_preset
from dynamixel_control.arm_hardware import (
    ARM_COMMAND_CALIBRATED, ARM_JOINT_CONFIG,
)


ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR_STATUS = 70
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_LOAD = 126
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132

LEN_GOAL_POSITION = 4
LEN_HARDWARE_ERROR_STATUS = 1
LEN_PRESENT_LOAD = 2
LEN_PRESENT_VELOCITY = 4
LEN_PRESENT_POSITION = 4

# X-시리즈(XL430/XC430/XM 공통) Present Velocity 데이터시트 고정값: signed, 1 LSB = 0.229 rev/min.
# PRESENT_VELOCITY 는 이미 아래 SyncRead 범위(64~135) 안에 있어 버스 트랜잭션 추가 없이
# 파싱만 하면 된다 — 그동안 버려지던 바이트를 꺼내 쓰는 것뿐(Notion "그리퍼 tick/
# wrist_to_gripper/PRESENT_VELOCITY 실측·검증 절차" §2-3).
VELOCITY_LSB_TO_RAD_S = 0.229 * 2.0 * math.pi / 60.0

# TORQUE_ENABLE(64,1) ~ PRESENT_POSITION(132,4) 은 X-시리즈 컨트롤 테이블에서
# 연속 주소 범위라, 64부터 72바이트를 한 번의 SyncRead 로 받아 torque/fault/load/position 을
# 함께 추출(버스 트랜잭션 1회). 중간의 다른 필드(Profile Accel/Velocity 등)도 같이
# 읽히지만 안 쓰고 버림 — 주소가 연속이기만 하면 여분을 읽는 건 무해함.
# (XL430/XC430/XM 계열 공통. 다른 모델이면 주소 재확인 필요 — CLAUDE.md §8 모터모델 미확정.)
ADDR_SYNC_READ_START = ADDR_TORQUE_ENABLE
LEN_SYNC_READ = (ADDR_PRESENT_POSITION + LEN_PRESENT_POSITION) - ADDR_TORQUE_ENABLE

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

PROTOCOL_VERSION = 2.0
BAUDRATE = 1000000
DEVICENAME = "/dev/ttyUSB0"

DXL_MINIMUM_POSITION_VALUE = 0
DXL_MAXIMUM_POSITION_VALUE = 4095
DXL_CENTER_POSITION = 2048

TICKS_PER_RAD = 4096.0 / (2.0 * math.pi)
DXL_TICKS_PER_REV = 4096.0  # 물리 인코더 상수(=TICKS_PER_RAD*2π) — Present Velocity 환산용


JOINT_CONFIG = ARM_JOINT_CONFIG
ARM_ID_SEQUENCE = [config["id"] for config in JOINT_CONFIG.values()]
ARM_IDS = {config["id"] for config in JOINT_CONFIG.values()}


def to_signed(value, byte_len):
    """무부호 정수를 byte_len 바이트 2의 보수 부호 정수로 변환."""
    bits = byte_len * 8
    if value >= (1 << (bits - 1)):
        value -= (1 << bits)
    return value


class MoveItDynamixelBridge(Node):
    def __init__(self):
        super().__init__("moveit_dynamixel_bridge")

        # --- 그리퍼 파라미터 (랙피니언 2모터 동일방향 구동, ID 3/4) ---
        # gripper_type 이 gripper_presets.GRIPPER_PRESETS 의 기본값을 고르고,
        # 아래 개별 파라미터는 필요 시 CLI/런치로 여전히 개별 오버라이드 가능.
        self.declare_parameter("gripper_type", DEFAULT_GRIPPER)
        self.gripper_type = self.get_parameter("gripper_type").value
        preset = get_preset(self.gripper_type, self.get_logger())

        self.declare_parameter("gripper_joints", preset["gripper_joints"])
        self.declare_parameter("gripper_ids", preset["gripper_ids"])  # 빈 배열이면 그리퍼 비활성
        self.declare_parameter("gripper_open_rad", preset["gripper_open_rad"])
        self.declare_parameter("gripper_close_rad", preset["gripper_close_rad"])
        self.declare_parameter("gripper_open_tick", preset["gripper_open_tick"])
        self.declare_parameter("gripper_close_tick", preset["gripper_close_tick"])
        self.declare_parameter("read_only", False)
        self.declare_parameter("gripper_only_mode", False)
        self.declare_parameter("trajectory_goal_tolerance_rad", 0.03)
        self.declare_parameter("trajectory_goal_timeout_s", 10.0)
        self.declare_parameter("trajectory_feedback_timeout_s", 0.5)

        self.gripper_joints = list(self.get_parameter("gripper_joints").value)
        self.gripper_ids = list(self.get_parameter("gripper_ids").value)
        self.gripper_open_rad = float(self.get_parameter("gripper_open_rad").value)
        self.gripper_close_rad = float(self.get_parameter("gripper_close_rad").value)
        self.gripper_open_tick = int(self.get_parameter("gripper_open_tick").value)
        self.gripper_close_tick = int(self.get_parameter("gripper_close_tick").value)
        self.read_only = bool(self.get_parameter("read_only").value)
        self.gripper_only_mode = bool(
            self.get_parameter("gripper_only_mode").value)
        if (not self.read_only and not self.gripper_only_mode
                and not ARM_COMMAND_CALIBRATED):
            raise RuntimeError(
                "Arm writes blocked: center tick, direction, limits, and fixed "
                "yaw are not calibrated. Use read_only:=true.")
        self.trajectory_goal_tolerance = float(
            self.get_parameter("trajectory_goal_tolerance_rad").value)
        self.trajectory_goal_timeout = float(
            self.get_parameter("trajectory_goal_timeout_s").value)
        self.trajectory_feedback_timeout = float(
            self.get_parameter("trajectory_feedback_timeout_s").value)

        self._bus_lock = threading.Lock()
        self._feedback_lock = threading.Lock()
        self._latest_arm_positions = {}
        self._latest_arm_feedback_time = None
        self._torque_states = {}

        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            raise RuntimeError(f"Failed to open port: {DEVICENAME}")

        if not self.port_handler.setBaudRate(BAUDRATE):
            raise RuntimeError(f"Failed to set baudrate: {BAUDRATE}")

        self.group_sync_write = GroupSyncWrite(
            self.port_handler,
            self.packet_handler,
            ADDR_GOAL_POSITION,
            LEN_GOAL_POSITION,
        )

        # hardware error+address 126 feedback+position 블록을 한 번에 읽는 SyncRead
        self.group_sync_read = GroupSyncRead(
            self.port_handler,
            self.packet_handler,
            ADDR_SYNC_READ_START,
            LEN_SYNC_READ,
        )

        # SyncRead 등록 ID와 이 프로세스가 토크를 켠 ID를 별도로 추적한다.
        # read-only에서는 팔+그리퍼, gripper-only에서는 그리퍼만 write 없이 등록한다.
        self.active_ids = set()
        self.torque_enabled_ids = set()

        if self.read_only:
            # Read-only mode registers every arm and gripper motor for SyncRead.
            # addParam only configures the SDK's broadcast-read packet locally;
            # it does not write a Dynamixel register.
            self._register_read_only_motors()
            self.get_logger().info(
                f"Read-only mode: monitoring arm IDs {ARM_ID_SEQUENCE} and "
                f"gripper IDs {self.gripper_ids}; all register writes disabled")
        elif self.gripper_only_mode:
            # Gripper-only diagnostics also avoid startup register writes.
            for gid in self.gripper_ids:
                if self.group_sync_read.addParam(gid):
                    self.active_ids.add(gid)
            self.get_logger().info(
                "Gripper-only mode: monitoring gripper IDs only; "
                "startup torque/position writes are disabled"
            )
        else:
            # 팔 서보: 토크 ON 성공한 ID만 SyncRead 등록
            for joint_name, config in JOINT_CONFIG.items():
                if self._enable_torque(config["id"], joint_name):
                    self.group_sync_read.addParam(config["id"])
                    self.active_ids.add(config["id"])
                    self.torque_enabled_ids.add(config["id"])

            # 그리퍼 서보: 토크 ON 성공한 ID만 SyncRead 등록
            for gid in self.gripper_ids:
                if self._enable_torque(gid, f"gripper(id {gid})"):
                    self.group_sync_read.addParam(gid)
                    self.active_ids.add(gid)
                    self.torque_enabled_ids.add(gid)

        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            self.trajectory_callback,
            10,
        )

        # 벤치 teleop_core의 단일 관절 명령. 메시지는 [motor_id, goal_tick].
        # FSM/MoveIt 경로와 같은 GroupSyncWrite를 사용하되 알려진 팔 ID만 허용한다.
        self.teleop_goal_sub = self.create_subscription(
            Int32MultiArray,
            "/dynamixel/goal_position",
            self.teleop_goal_callback,
            10,
        )

        self._action_group = ReentrantCallbackGroup()
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            execute_callback=self.execute_follow_joint_trajectory,
            goal_callback=self.arm_goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._action_group,
        )

        # 그리퍼 액션 서버 (FSM 이 /gripper_controller/follow_joint_trajectory 로 파지/개방 명령)
        self.gripper_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory",
            execute_callback=self.execute_gripper,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.joint_state_pub = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )

        # 계약 §5.1 "locked heartbeat는 ... controller fault 0 ... 을 실제 확인한다" 대응.
        # arm_fsm 이 CARRYING_LOCKED/STOWED_LOCKED 발행 전 게이트로 구독(내부용 — 파워트레인
        # 쪽 DDS 경계를 넘지 않음, robot_arm_msgs 계약과 무관).
        self.fault_pub = self.create_publisher(
            Bool,
            "/dynamixel/controller_fault",
            10,
        )

        self.feedback_timer = self.create_timer(0.05, self.publish_joint_states)

        self.get_logger().info(
            f"MoveIt Dynamixel bridge started (arm={list(JOINT_CONFIG)}, "
            f"gripper_type={self.gripper_type}, gripper_ids={self.gripper_ids}, "
            f"read_only={self.read_only}, gripper_only_mode={self.gripper_only_mode})"
        )

    # ------------------------------------------------------------------ helpers
    def _register_read_only_motors(self):
        """Register all configured motors in SyncRead without register writes."""
        for dxl_id in dict.fromkeys(ARM_ID_SEQUENCE + list(self.gripper_ids)):
            if self.group_sync_read.addParam(dxl_id):
                self.active_ids.add(dxl_id)

    def _enable_torque(self, dxl_id, label):
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
        )
        if result != 0 or error != 0:
            self.get_logger().warn(
                f"Torque enable failed: {label}, id={dxl_id}, result={result}, error={error}"
            )
            return False
        else:
            self.get_logger().info(f"Torque enabled: {label} -> id {dxl_id}")
            return True

    def rad_to_tick(self, joint_name, rad):
        config = JOINT_CONFIG[joint_name]
        tick = config["center"] + config["direction"] * rad * TICKS_PER_RAD
        tick = int(round(tick))
        return max(DXL_MINIMUM_POSITION_VALUE, min(DXL_MAXIMUM_POSITION_VALUE, tick))

    def tick_to_rad(self, joint_name, tick):
        config = JOINT_CONFIG[joint_name]
        return (tick - config["center"]) / (config["direction"] * TICKS_PER_RAD)

    def gripper_pos_to_tick(self, rad):
        span = self.gripper_open_tick - self.gripper_close_tick
        denom = self.gripper_open_rad - self.gripper_close_rad
        frac = 0.0 if denom == 0.0 else (rad - self.gripper_close_rad) / denom
        tick = int(round(self.gripper_close_tick + frac * span))
        return max(DXL_MINIMUM_POSITION_VALUE, min(DXL_MAXIMUM_POSITION_VALUE, tick))

    def gripper_tick_to_pos(self, tick):
        span = self.gripper_open_tick - self.gripper_close_tick
        if span == 0:
            return self.gripper_close_rad
        frac = (tick - self.gripper_close_tick) / span
        return self.gripper_close_rad + frac * (self.gripper_open_rad - self.gripper_close_rad)

    def gripper_velocity_to_rad_s(self, velocity_raw):
        """Present Velocity(raw, 0.229rev/min 단위) → gripper_tick_to_pos 와 같은 논리 rad/s.

        Present Velocity 는 서보축 물리 회전속도(4096tick/rev 고정, 데이터시트 상수)이고
        gripper_tick_to_pos 의 tick→rad 기울기는 open/close 캘리브 span 기반의 별도 계수라
        두 스케일을 직접 연결해야 한다: raw → 물리 tick/s(4096tick/rev 경유) → 캘리브
        기울기(rad/tick)로 환산. 부호/스케일은 실기 검증 전까지 확정 아님(Notion 절차 §2-3).
        """
        span = self.gripper_open_tick - self.gripper_close_tick
        if span == 0:
            return 0.0
        ticks_per_s = velocity_raw * (0.229 / 60.0) * DXL_TICKS_PER_REV
        rad_per_tick = (self.gripper_open_rad - self.gripper_close_rad) / span
        return ticks_per_s * rad_per_tick

    def int_to_little_endian_4bytes(self, value):
        return [
            value & 0xFF,
            (value >> 8) & 0xFF,
            (value >> 16) & 0xFF,
            (value >> 24) & 0xFF,
        ]

    def goal_callback(self, goal_request):
        if self.read_only:
            self.get_logger().warn("Read-only mode: rejecting trajectory goal")
            return GoalResponse.REJECT
        self.get_logger().info("Received FollowJointTrajectory goal")
        return GoalResponse.ACCEPT

    def arm_goal_callback(self, goal_request):
        if self.gripper_only_mode:
            self.get_logger().error(
                "Gripper-only mode: rejecting arm FollowJointTrajectory goal")
            return GoalResponse.REJECT
        if self.read_only:
            return GoalResponse.REJECT
        trajectory = goal_request.trajectory
        if "arm_joint_1" in trajectory.joint_names:
            self.get_logger().error(
                "Rejecting arm_joint_1: chassis yaw is fixed and unactuated")
            return GoalResponse.REJECT
        if not trajectory.points or not trajectory.joint_names:
            self.get_logger().error("Rejecting empty arm trajectory")
            return GoalResponse.REJECT
        if len(set(trajectory.joint_names)) != len(trajectory.joint_names):
            self.get_logger().error("Rejecting duplicate arm joint names")
            return GoalResponse.REJECT
        if set(trajectory.joint_names) != set(JOINT_CONFIG):
            self.get_logger().error(
                "Rejecting incomplete arm trajectory: expected exactly "
                f"{list(JOINT_CONFIG)}, got {list(trajectory.joint_names)}")
            return GoalResponse.REJECT
        unknown = [name for name in trajectory.joint_names
                   if name not in JOINT_CONFIG]
        inactive = [name for name in trajectory.joint_names
                    if name in JOINT_CONFIG
                    and JOINT_CONFIG[name]["id"] not in self.active_ids]
        if unknown or inactive:
            self.get_logger().error(
                f"Rejecting trajectory: unknown={unknown}, inactive={inactive}")
            return GoalResponse.REJECT
        previous = -1.0
        for point in trajectory.points:
            if len(point.positions) != len(trajectory.joint_names):
                self.get_logger().error("Rejecting malformed trajectory point")
                return GoalResponse.REJECT
            stamp = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            if stamp < previous:
                self.get_logger().error("Rejecting non-monotonic trajectory timing")
                return GoalResponse.REJECT
            previous = stamp
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Cancel requested")
        return CancelResponse.ACCEPT

    # ------------------------------------------------------------------ arm
    def teleop_goal_callback(self, msg):
        if len(msg.data) != 2:
            self.get_logger().warn("Teleop goal must be [motor_id, goal_tick]")
            return

        dxl_id, goal_tick = (int(msg.data[0]), int(msg.data[1]))
        if dxl_id not in ARM_IDS:
            self.get_logger().warn(f"Unknown arm motor ID from teleop: {dxl_id}")
            return
        if self.gripper_only_mode:
            self.get_logger().error(
                f"Gripper-only mode: rejecting arm teleop command id={dxl_id}")
            return
        if self.read_only:
            self.get_logger().warn("Read-only mode: ignoring teleop goal")
            return
        if dxl_id not in self.active_ids:
            self.get_logger().error(f"Inactive arm motor ID from teleop: {dxl_id}")
            return

        goal_tick = max(DXL_MINIMUM_POSITION_VALUE,
                        min(DXL_MAXIMUM_POSITION_VALUE, goal_tick))
        with self._bus_lock:
            self.group_sync_write.clearParam()
            if not self.group_sync_write.addParam(
                    dxl_id, self.int_to_little_endian_4bytes(goal_tick)):
                self.get_logger().warn(
                    f"Failed to add teleop sync write param: id={dxl_id}")
                return
            result = self.group_sync_write.txPacket()
            self.group_sync_write.clearParam()
        if result != 0:
            self.get_logger().warn(f"Teleop GroupSyncWrite failed: result={result}")
            return
        self.get_logger().info(f"teleop -> id {dxl_id}: tick {goal_tick}")

    def execute_follow_joint_trajectory(self, goal_handle):
        trajectory = goal_handle.request.trajectory

        self.get_logger().info(
            f"Executing FollowJointTrajectory with {len(trajectory.points)} points"
        )

        result = FollowJointTrajectory.Result()
        start = time.monotonic()
        for index, point in enumerate(trajectory.points):
            due = (point.time_from_start.sec
                   + point.time_from_start.nanosec * 1e-9)
            # A one-point trajectory is a target to start moving toward now;
            # time_from_start is its desired arrival time, not a send delay.
            send_at = 0.0 if len(trajectory.points) == 1 else due
            if index == 0:
                send_at = 0.0
            while time.monotonic() - start < send_at:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = "Trajectory canceled"
                    return result
                time.sleep(0.01)
            if not self._write_arm_point(
                    trajectory.joint_names, point.positions):
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
                result.error_string = "Failed to write all trajectory joints"
                return result

        target = dict(zip(
            trajectory.joint_names, trajectory.points[-1].positions))
        deadline = time.monotonic() + self.trajectory_goal_timeout
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = "Trajectory canceled"
                return result
            if self._arm_goal_reached(target):
                goal_handle.succeed()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = "All commanded joints reached the goal"
                return result
            time.sleep(0.02)

        goal_handle.abort()
        result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
        result.error_string = "Timed out waiting for measured joint positions"
        return result

    def trajectory_callback(self, msg):
        if self.gripper_only_mode:
            self.get_logger().error(
                "Gripper-only mode: rejecting arm trajectory topic command")
            return
        if self.read_only:
            self.get_logger().warn("Read-only mode: ignoring arm trajectory")
            return
        if not msg.points:
            return

        point = msg.points[-1]

        if len(msg.joint_names) != len(point.positions):
            self.get_logger().warn("JointTrajectory names/positions length mismatch")
            return

        self._write_arm_point(msg.joint_names, point.positions)

    def _write_arm_point(self, joint_names, positions):
        """Write one complete trajectory point; reject partial hardware writes."""
        if len(joint_names) != len(positions):
            return False
        if set(joint_names) != set(JOINT_CONFIG):
            self.get_logger().warn(
                "Arm point must contain exactly the four active joints")
            return False
        params = []

        for joint_name, rad in zip(joint_names, positions):
            if joint_name == "arm_joint_1":
                self.get_logger().warn(
                    "arm_joint_1 is fixed to the chassis and cannot be commanded")
                return False
            if joint_name not in JOINT_CONFIG:
                self.get_logger().warn(f"Unknown joint from MoveIt: {joint_name}")
                return False

            dxl_id = JOINT_CONFIG[joint_name]["id"]
            if dxl_id not in self.active_ids:
                self.get_logger().error(
                    f"Inactive arm motor from trajectory: {joint_name}, id={dxl_id}"
                )
                return False
            goal_tick = self.rad_to_tick(joint_name, rad)
            params.append((dxl_id, joint_name, rad, goal_tick))

            self.get_logger().info(
                f"{joint_name} -> id {dxl_id}: {rad:.3f} rad -> {goal_tick}"
            )

        with self._bus_lock:
            self.group_sync_write.clearParam()
            for dxl_id, _name, _rad, goal_tick in params:
                if not self.group_sync_write.addParam(
                        dxl_id, self.int_to_little_endian_4bytes(goal_tick)):
                    self.group_sync_write.clearParam()
                    return False
            result = self.group_sync_write.txPacket()
            self.group_sync_write.clearParam()
        if result != 0:
            self.get_logger().warn(f"GroupSyncWrite failed: result={result}")
            return False
        return True

    def _arm_goal_reached(self, target):
        """Return true only for fresh measured feedback within goal tolerance."""
        with self._feedback_lock:
            positions = dict(self._latest_arm_positions)
            sample_time = self._latest_arm_feedback_time
        if sample_time is None:
            return False
        if time.monotonic() - sample_time > self.trajectory_feedback_timeout:
            return False
        return all(name in positions
                   and abs(positions[name] - desired)
                   <= self.trajectory_goal_tolerance
                   for name, desired in target.items())

    # ------------------------------------------------------------------ gripper
    def execute_gripper(self, goal_handle):
        trajectory = goal_handle.request.trajectory

        result = FollowJointTrajectory.Result()

        if not self.gripper_ids:
            self.get_logger().warn("Gripper goal received but gripper_ids is empty — ignored")
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result

        if trajectory.points:
            point = trajectory.points[-1]
            name_to_pos = dict(zip(trajectory.joint_names, point.positions))
            # 단일 구동 조인트(gripper_left_pinion_joint)만 사용 — 나머지 3개(우 피니언·좌우 랙)는
            # URDF <mimic> 으로 종속된다. 두 서보(id 3,4)에는 같은 goal_tick 을 보낸다.
            target_rad = None
            for jn in self.gripper_joints:
                if jn in name_to_pos:
                    target_rad = name_to_pos[jn]
                    break
            if target_rad is not None:
                self._write_gripper(target_rad)
            else:
                self.get_logger().warn(
                    f"Gripper goal has no known finger joint {self.gripper_joints}"
                )

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "Gripper command sent to Dynamixel"
        return result

    def _write_gripper(self, rad):
        if self.read_only:
            self.get_logger().warn("Read-only mode: ignoring gripper command")
            return
        goal_tick = self.gripper_pos_to_tick(rad)
        with self._bus_lock:
            for gid in self.gripper_ids:
                result, error = self.packet_handler.write4ByteTxRx(
                    self.port_handler, gid, ADDR_GOAL_POSITION, goal_tick
                )
                if result != 0 or error != 0:
                    self.get_logger().warn(
                        f"Gripper write failed: id={gid}, result={result}, "
                        f"error={error}")
        self.get_logger().info(
            f"gripper -> {rad:.4f} rad -> tick {goal_tick} "
            f"(ids {self.gripper_ids})"
        )

    # ------------------------------------------------------------------ feedback
    def publish_joint_states(self):
        with self._bus_lock:
            self.group_sync_read.txRxPacket()
        # 일부 ID가 버스에 없어도 응답받은 ID만 처리 (result 무시)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        # controller fault 집계 — SyncRead 에 등록된 팔 ID 중 하나라도
        # Hardware Error Status != 0 이거나 이번 tick 응답이 없으면 fault=True.
        # 응답 없음도 fault 로 보는 이유: 활성 등록된 서보가 갑자기 무응답이면 버스/전원
        # 이상일 수 있어 "정상"으로 오인하면 안 됨(안전 측 기본값).
        fault = False if self.gripper_only_mode else not ARM_IDS.issubset(
            self.active_ids)

        # 팔 관절: position(rad) + address-126 feedback(raw signed).
        # 주소 126의 의미는 실제 장착 모터 control table로 확인해야 한다.
        if not self.gripper_only_mode:
            for joint_name, config in JOINT_CONFIG.items():
                dxl_id = config["id"]
                if dxl_id not in self.active_ids:
                    continue
                sample = self._read_sample(dxl_id)
                if sample is None:
                    fault = True
                    continue
                feedback_raw, tick, hw_error, velocity_raw, torque_enabled = sample
                self._report_torque_state(dxl_id, joint_name, torque_enabled)
                if self.read_only and torque_enabled:
                    fault = True
                if hw_error != 0:
                    fault = True
                msg.name.append(joint_name)
                msg.position.append(self.tick_to_rad(joint_name, tick))
                msg.velocity.append(velocity_raw * VELOCITY_LSB_TO_RAD_S / config["direction"])
                msg.effort.append(float(feedback_raw))

        # XL430-W250 그리퍼: 주소 126은 signed Present Load(0.1% 추정 부하)다.
        # 랙피니언 2모터(ID 3,4)를 함께 읽어 하나의 논리 조인트(gripper_left_pinion_joint)로
        # 보고한다 — position(rad)=대표(첫 응답) 모터 tick, effort=가장 큰 abs(load).
        # 한 모터라도 부하가 크면 파지로 보는 보수적(안전 측) 집계이며, FSM 이 이 effort 로
        # 파지/DROP 을 판정한다.
        gripper_samples = []
        for gid in self.gripper_ids:
            if gid not in self.active_ids:
                fault = True
                continue
            sample = self._read_sample(gid)
            if sample is None:
                fault = True
                continue
            load_raw, tick, hw_error, velocity_raw, torque_enabled = sample
            self._report_torque_state(
                gid, f"gripper(id {gid})", torque_enabled)
            if self.read_only and torque_enabled:
                fault = True
            if hw_error != 0:
                fault = True
            gripper_samples.append((load_raw, to_signed(tick, LEN_PRESENT_POSITION), velocity_raw))

        # Publish the logical gripper joint from whatever motors responded.
        # Missing configured IDs still keep controller_fault=true above.
        if gripper_samples:
            representative_tick = gripper_samples[0][1]
            representative_velocity_raw = gripper_samples[0][2]
            max_abs_load = max(abs(sample[0]) for sample in gripper_samples)
            finger_rad = self.gripper_tick_to_pos(representative_tick)
            finger_vel = self.gripper_velocity_to_rad_s(representative_velocity_raw)
            for jn in self.gripper_joints:
                msg.name.append(jn)
                msg.position.append(finger_rad)
                msg.velocity.append(finger_vel)
                msg.effort.append(float(max_abs_load))

        arm_positions = {
            name: position for name, position in zip(msg.name, msg.position)
            if name in JOINT_CONFIG
        }
        if arm_positions:
            with self._feedback_lock:
                self._latest_arm_positions = arm_positions
                self._latest_arm_feedback_time = time.monotonic()
        self.joint_state_pub.publish(msg)
        self.fault_pub.publish(Bool(data=fault))

    def _read_sample(self, dxl_id):
        """Extract torque, fault, feedback, velocity, and position with SyncRead.

        PRESENT_VELOCITY(128,4)는 SyncRead 범위(64~135) 안에 이미 포함돼 있어 별도 버스
        요청 없이 같은 블록에서 꺼낸다. 미수신 시 None.
        """
        if not self.group_sync_read.isAvailable(
                dxl_id, ADDR_TORQUE_ENABLE, 1):
            return None
        if not self.group_sync_read.isAvailable(
                dxl_id, ADDR_HARDWARE_ERROR_STATUS, LEN_HARDWARE_ERROR_STATUS):
            return None
        if not self.group_sync_read.isAvailable(
                dxl_id, ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD):
            return None
        if not self.group_sync_read.isAvailable(
                dxl_id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY):
            return None
        if not self.group_sync_read.isAvailable(
                dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
            return None
        torque_enabled = bool(self.group_sync_read.getData(
            dxl_id, ADDR_TORQUE_ENABLE, 1))
        hw_error = self.group_sync_read.getData(
            dxl_id, ADDR_HARDWARE_ERROR_STATUS, LEN_HARDWARE_ERROR_STATUS)
        feedback_raw = to_signed(
            self.group_sync_read.getData(dxl_id, ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD),
            LEN_PRESENT_LOAD,
        )
        velocity_raw = to_signed(
            self.group_sync_read.getData(dxl_id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY),
            LEN_PRESENT_VELOCITY,
        )
        tick = self.group_sync_read.getData(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        return feedback_raw, tick, hw_error, velocity_raw, torque_enabled

    def _report_torque_state(self, dxl_id, label, enabled):
        """Log a motor's read-only Torque Enable state when it changes."""
        if self._torque_states.get(dxl_id) == enabled:
            return
        self._torque_states[dxl_id] = enabled
        message = f"Torque state: {label}, id={dxl_id}, enabled={enabled}"
        if self.read_only and enabled:
            self.get_logger().error(message + " — manual movement is unsafe")
        else:
            self.get_logger().info(message)

    def destroy_node(self):
        for dxl_id in self.torque_enabled_ids:
            self.packet_handler.write1ByteTxRx(
                self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
            )

        self.port_handler.closePort()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MoveItDynamixelBridge()

    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
