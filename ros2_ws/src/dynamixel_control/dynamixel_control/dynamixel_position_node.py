import math
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState

from dynamixel_sdk import (
    PortHandler, PacketHandler, GroupSyncRead, GroupSyncWrite,
)
from dynamixel_control.arm_hardware import (
    ARM_COMMAND_CALIBRATED, ARM_JOINT_NAMES, ARM_MOTOR_IDS,
)


# =========================
# Dynamixel 기본 설정
# =========================
DEVICENAME = "/dev/ttyUSB0"
BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0

# =========================
# XL430 Control Table 주소
# =========================
ADDR_OPERATING_MODE = 11        # EEPROM — 토크 OFF 상태에서만 써진다
ADDR_TORQUE_ENABLE = 64
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116

ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_TEMPERATURE = 146

# 126~135 는 연속이다 (current 2 + velocity 4 + position 4) → SyncRead 한 번에 읽는다.
SYNC_READ_ADDR = ADDR_PRESENT_CURRENT
SYNC_READ_LEN = 10

LEN_GOAL_POSITION = 4

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

OP_MODE_POSITION = 3            # 0~4095 단일회전 위치제어


def _set_ftdi_latency(port, value, logger):
    """FTDI 의 latency_timer 를 낮춘다.

    기본값 16ms 는 '보낼 데이터가 적으면 최대 16ms 모았다가 올린다'는 뜻이라,
    요청-응답으로 도는 다이나믹셀 통신에서는 **왕복마다 16ms** 가 그대로 붙는다.
    6관절 상태 읽기가 2.6Hz 까지 떨어졌던 원인이 이것이다.

    USB 를 다시 꽂으면 16 으로 되돌아가므로 기동할 때마다 직접 맞춘다.
    (컨테이너가 privileged 라 sysfs 에 쓸 수 있다. 안 되면 경고만 남기고 진행.)
    """
    name = os.path.basename(port)
    path = f"/sys/bus/usb-serial/devices/{name}/latency_timer"
    try:
        with open(path, "r") as f:
            before = f.read().strip()
        if before == str(value):
            return
        with open(path, "w") as f:
            f.write(str(value))
        logger.info(f"FTDI latency_timer {before} → {value} ms ({name})")
    except OSError as exc:
        logger.warn(
            f"latency_timer 설정 실패 ({path}): {exc} — "
            f"통신이 느리면 이 값을 1 로 낮추세요")


def _signed(value, byte_count):
    """dynamixel_sdk 의 read*ByteTxRx 는 unsigned 로 준다 → 2의 보수로 되돌린다.

    velocity/current 는 음수가 정상값인데, 그대로 두면 -1 이 4294967295 가 되어
    Int32MultiArray(int32 범위) 대입에서 노드가 죽는다.
    """
    bits = byte_count * 8
    value &= (1 << bits) - 1
    return value - (1 << bits) if value >= (1 << (bits - 1)) else value


# 2026-08-05 read-only manual encoder mapping. Gripper IDs 3/4 are owned by
# moveit_dynamixel_bridge and are deliberately not duplicated in this legacy
# direct-position node.
DEFAULT_MOTOR_IDS = ARM_MOTOR_IDS

# URDF(robot_arm.urdf)의 구동 관절 이름과 모터 ID 순서를 맞춤.
# 구동 조인트는 gripper_left_pinion_joint 하나뿐 — 나머지 그리퍼 관절(우 피니언·좌우 랙)은
# 전부 mimic 이라 여기 없다. 서보는 2개(id 3,4)지만 항상 같은 값으로 구동한다.
DEFAULT_JOINT_NAMES = ARM_JOINT_NAMES

# 프로파일 가감속 기본값. **0(=최고속 즉시 이동)으로 두지 말 것** — 명령마다
# 순간 과전류로 토크가 풀린다(HW-8 실기 검증, 재현율 100%, 명령 후 0.3초 내 트립).
DEFAULT_PROFILE_ACCELERATION = 25
DEFAULT_PROFILE_VELOCITY = 80


class DynamixelPositionNode(Node):
    def __init__(self):
        super().__init__("dynamixel_position_node")

        if not ARM_COMMAND_CALIBRATED:
            raise RuntimeError(
                "Direct Dynamixel writes blocked until center tick, direction, "
                "limits, and fixed yaw are calibrated")

        self.declare_parameter("port", DEVICENAME)
        self.declare_parameter("baudrate", BAUDRATE)
        self.declare_parameter("motor_ids", DEFAULT_MOTOR_IDS)
        self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)
        self.declare_parameter("profile_acceleration", DEFAULT_PROFILE_ACCELERATION)
        self.declare_parameter("profile_velocity", DEFAULT_PROFILE_VELOCITY)
        # 기동 시 operating mode 를 위치제어(3)로 맞출지. 버스에 velocity/extended
        # position 모드로 남아 있는 서보가 섞여 있으면 goal_position 이 먹지 않는다.
        self.declare_parameter("force_position_mode", True)
        self.declare_parameter("read_rate_hz", 20.0)
        # 목표값 flush 주기. 텔레옵(20Hz)보다 빨라야 명령이 한 틱씩 밀리지 않는다.
        self.declare_parameter("write_rate_hz", 50.0)
        # 온도는 SyncRead 묶음(126~135) 밖(146)이라 따로 읽어야 한다. 자주 읽을
        # 이유가 없으므로 매 주기 한 개씩 돌아가며 읽는다.
        self.declare_parameter("temperature_poll_hz", 2.0)
        # FTDI latency_timer [ms]. 0 이면 건드리지 않는다.
        self.declare_parameter("ftdi_latency_timer", 1)

        port = self.get_parameter("port").value
        baudrate = int(self.get_parameter("baudrate").value)
        self.motor_ids = [int(v) for v in self.get_parameter("motor_ids").value]
        self.joint_names = list(self.get_parameter("joint_names").value)
        self.profile_acc = int(self.get_parameter("profile_acceleration").value)
        self.profile_vel = int(self.get_parameter("profile_velocity").value)
        self.force_position_mode = bool(self.get_parameter("force_position_mode").value)
        read_rate = float(self.get_parameter("read_rate_hz").value)
        write_rate = float(self.get_parameter("write_rate_hz").value)
        temp_rate = float(self.get_parameter("temperature_poll_hz").value)

        if len(self.motor_ids) != len(self.joint_names):
            raise RuntimeError(
                f"motor_ids({len(self.motor_ids)})와 "
                f"joint_names({len(self.joint_names)}) 길이가 다릅니다"
            )

        latency = int(self.get_parameter("ftdi_latency_timer").value)
        if latency > 0:
            _set_ftdi_latency(port, latency, self.get_logger())

        # 포트/패킷 핸들러 생성
        self.port_handler = PortHandler(port)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        # 포트 열기
        if not self.port_handler.openPort():
            self.get_logger().error(f"Failed to open port: {port}")
            raise RuntimeError("Failed to open Dynamixel port")

        # Baudrate 설정
        if not self.port_handler.setBaudRate(baudrate):
            self.get_logger().error(f"Failed to set baudrate: {baudrate}")
            raise RuntimeError("Failed to set baudrate")

        self.get_logger().info(f"Dynamixel port opened: {port} @ {baudrate}bps")

        # 응답하는 서보만 등록한다 — 빠진 서보 하나가 나머지 readback 을 망치지 않게.
        self.active = []
        for index, dxl_id in enumerate(self.motor_ids):
            if self._init_motor(dxl_id):
                self.active.append((dxl_id, self.joint_names[index]))

        if not self.active:
            raise RuntimeError("응답하는 서보가 없습니다 — 배선/전원을 확인하세요")

        # 개별 read/write 는 매 건마다 status 패킷을 기다린다. FTDI latency_timer
        # 까지 겹치면 6관절 × 4항목 = 24 왕복에 수백 ms 가 든다(실측 2.6Hz).
        # → 상태는 SyncRead 한 번, 목표는 SyncWrite 한 번으로 묶는다.
        self.sync_read = GroupSyncRead(
            self.port_handler, self.packet_handler, SYNC_READ_ADDR, SYNC_READ_LEN)
        for dxl_id, _name in self.active:
            if not self.sync_read.addParam(dxl_id):
                self.get_logger().warn(f"SyncRead addParam 실패 ID {dxl_id}")
        self.sync_write = GroupSyncWrite(
            self.port_handler, self.packet_handler,
            ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

        # goal_callback 은 시리얼을 만지지 않고 여기에 최신값만 남긴다.
        # 콜백에서 바로 쓰면 20Hz × 6관절 = 120 왕복/초가 읽기와 버스를 두고 다툰다.
        self.pending_goals = {}
        self.temperature = {dxl_id: 0 for dxl_id, _ in self.active}
        self._temp_cursor = 0

        self.get_logger().info(
            "구동 대상: "
            + ", ".join(f"{name}(ID {i})" for i, name in self.active)
        )

        # 위치 명령 구독
        # 메시지 형식: [모터ID, 목표위치]
        self.subscription = self.create_subscription(
            Int32MultiArray,
            "/dynamixel/goal_position",
            self.goal_callback,
            10,
        )

        # 모터 상태 publish
        # 데이터 형식: [id, position, velocity, current, temperature, ...]
        self.state_pub = self.create_publisher(
            Int32MultiArray,
            "/dynamixel/state",
            10,
        )

        # RViz / MoveIt용 joint_states publish
        self.joint_state_pub = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )

        self.write_timer = self.create_timer(
            1.0 / max(1.0, write_rate), self.flush_goals)
        self.timer = self.create_timer(1.0 / max(0.1, read_rate), self.read_state)
        self.temp_timer = self.create_timer(
            1.0 / max(0.1, temp_rate), self.poll_temperature)

        self.get_logger().info(
            f"Dynamixel node started (read={read_rate}Hz, write={write_rate}Hz)")

    # ------------------------------------------------------------------ 기동
    def _init_motor(self, dxl_id):
        """서보 하나를 위치제어 + 프로파일 가감속 상태로 만들고 토크를 켠다.

        operating mode 는 EEPROM 이라 **토크 OFF 상태에서만** 써진다.
        반환값: 이 서보를 구동 대상으로 등록해도 되는가.
        """
        model, result, error = self.packet_handler.ping(self.port_handler, dxl_id)
        if result != 0:
            self.get_logger().warn(
                f"ID {dxl_id} 응답 없음 — 건너뜁니다 "
                f"({self.packet_handler.getTxRxResult(result)})"
            )
            return False

        # 모드 변경/EEPROM 쓰기를 위해 일단 토크를 내린다.
        self.packet_handler.write1ByteTxRx(
            self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

        if self.force_position_mode:
            mode, _, _ = self.packet_handler.read1ByteTxRx(
                self.port_handler, dxl_id, ADDR_OPERATING_MODE)
            if mode != OP_MODE_POSITION:
                result, error = self.packet_handler.write1ByteTxRx(
                    self.port_handler, dxl_id,
                    ADDR_OPERATING_MODE, OP_MODE_POSITION)
                if result != 0 or error != 0:
                    self.get_logger().error(
                        f"ID {dxl_id} operating mode 변경 실패 "
                        f"({mode} → {OP_MODE_POSITION}) — 건너뜁니다")
                    return False
                self.get_logger().info(
                    f"ID {dxl_id} operating mode {mode} → {OP_MODE_POSITION}(위치제어)")

        # 프로파일 가감속 — 0이면 명령마다 최고속으로 튀어 과전류 트립이 난다.
        self.packet_handler.write4ByteTxRx(
            self.port_handler, dxl_id, ADDR_PROFILE_ACCELERATION, self.profile_acc)
        self.packet_handler.write4ByteTxRx(
            self.port_handler, dxl_id, ADDR_PROFILE_VELOCITY, self.profile_vel)

        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if result != 0:
            self.get_logger().error(
                f"Torque enable failed ID {dxl_id}: "
                f"{self.packet_handler.getTxRxResult(result)}")
            return False
        if error != 0:
            self.get_logger().error(
                f"Torque enable error ID {dxl_id}: "
                f"{self.packet_handler.getRxPacketError(error)}")
            return False

        self.get_logger().info(
            f"Torque enabled : ID {dxl_id} (model={model}, "
            f"prof_acc={self.profile_acc}, prof_vel={self.profile_vel})")
        return True

    # ------------------------------------------------------------------ 명령
    def goal_callback(self, msg):
        """목표 위치 명령을 받아 Dynamixel에 전송."""
        if len(msg.data) < 2:
            self.get_logger().error("Message must be [id, goal_position]")
            return

        dxl_id = int(msg.data[0])
        goal_position = int(msg.data[1])

        if dxl_id not in [i for i, _ in self.active]:
            self.get_logger().error(f"Unknown Dynamixel ID: {dxl_id}")
            return

        # 시리얼은 여기서 만지지 않는다 — flush_goals 가 SyncWrite 로 한 번에 보낸다.
        self.pending_goals[dxl_id] = max(0, min(4095, goal_position))

    def flush_goals(self):
        """쌓인 목표를 SyncWrite 한 방으로 보낸다. 없으면 버스를 건드리지 않는다."""
        if not self.pending_goals:
            return

        goals = self.pending_goals
        self.pending_goals = {}

        self.sync_write.clearParam()
        for dxl_id, tick in goals.items():
            param = [
                tick & 0xFF,
                (tick >> 8) & 0xFF,
                (tick >> 16) & 0xFF,
                (tick >> 24) & 0xFF,
            ]
            if not self.sync_write.addParam(dxl_id, param):
                self.get_logger().warn(f"SyncWrite addParam 실패 ID {dxl_id}")

        result = self.sync_write.txPacket()
        if result != 0:
            self.get_logger().warn(
                f"SyncWrite 실패: {self.packet_handler.getTxRxResult(result)}")

    def poll_temperature(self):
        """온도(146)는 SyncRead 묶음(126~135) 밖이라 따로 읽는다.

        전부 매 주기 읽으면 왕복이 그만큼 늘어 텔레옵이 끊긴다 → 한 번에 하나씩.
        """
        if not self.active:
            return
        self._temp_cursor %= len(self.active)
        dxl_id = self.active[self._temp_cursor][0]
        self._temp_cursor += 1

        value, result, error = self.packet_handler.read1ByteTxRx(
            self.port_handler, dxl_id, ADDR_PRESENT_TEMPERATURE)
        if result == 0 and error == 0:
            self.temperature[dxl_id] = int(value)

    def read_state(self):
        """전 서보의 현재 전류/속도/위치를 SyncRead 한 번으로 읽어서 publish."""
        result = self.sync_read.txRxPacket()
        if result != 0:
            self.get_logger().warn(
                f"SyncRead 실패: {self.packet_handler.getTxRxResult(result)}")
            return

        state_msg = Int32MultiArray()
        state_data = []

        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()

        for dxl_id, joint_name in self.active:
            if not self.sync_read.isAvailable(
                    dxl_id, SYNC_READ_ADDR, SYNC_READ_LEN):
                self.get_logger().warn(f"SyncRead 데이터 없음 ID {dxl_id}")
                continue

            current = _signed(
                self.sync_read.getData(dxl_id, ADDR_PRESENT_CURRENT, 2), 2)
            velocity = _signed(
                self.sync_read.getData(dxl_id, ADDR_PRESENT_VELOCITY, 4), 4)
            position = _signed(
                self.sync_read.getData(dxl_id, ADDR_PRESENT_POSITION, 4), 4)

            # raw position 0~4095를 radian으로 근사 변환
            # 2048을 중앙, 한 바퀴를 2pi로 가정
            rad = (position - 2048) * (2.0 * math.pi / 4096.0)

            state_data.extend([
                int(dxl_id),
                int(position),
                int(velocity),
                int(current),
                int(self.temperature.get(dxl_id, 0)),
            ])

            joint_msg.name.append(joint_name)
            joint_msg.position.append(rad)
            joint_msg.velocity.append(float(velocity))
            joint_msg.effort.append(float(current))

        state_msg.data = state_data

        self.state_pub.publish(state_msg)
        self.joint_state_pub.publish(joint_msg)


def main(args=None):
    rclpy.init(args=args)

    node = DynamixelPositionNode()

    try:
        rclpy.spin(node)
    finally:
        node.port_handler.closePort()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
