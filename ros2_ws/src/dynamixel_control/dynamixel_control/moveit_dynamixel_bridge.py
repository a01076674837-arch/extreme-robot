#!/usr/bin/env python3

import json
import math
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from control_msgs.action import FollowJointTrajectory
from robot_arm_msgs.action import ArmRecordedPath, ArmTestMove, EndEffectorRotate
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite, GroupSyncRead
from ament_index_python.packages import get_package_share_directory

from dynamixel_control.tool_manager import (
    ParameterToolIdentityProvider, ToolManager)
from dynamixel_control.tool_profiles import (
    load_profiles, ToolProfileError, validate_control_scope)

ADDR_TORQUE_ENABLE = 64
ADDR_OPERATING_MODE = 11
ADDR_HARDWARE_ERROR_STATUS = 70
ADDR_GOAL_VELOCITY = 104
ADDR_GOAL_PWM = 100
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_LOAD = 126
ADDR_PRESENT_POSITION = 132

LEN_GOAL_POSITION = 4
LEN_GOAL_VELOCITY = 4
LEN_HARDWARE_ERROR_STATUS = 1
LEN_PRESENT_LOAD = 2
LEN_PRESENT_POSITION = 4

# X-시리즈(XL430/XC430/XM 공통) Present Velocity 데이터시트 고정값: signed, 1 LSB = 0.229 rev/min.
# PRESENT_VELOCITY 는 이미 아래 SyncRead 범위(70~135) 안에 있어 버스 트랜잭션 추가 없이
# 파싱만 하면 된다 — 그동안 버려지던 바이트를 꺼내 쓰는 것뿐(Notion "그리퍼 tick/
# wrist_to_gripper/PRESENT_VELOCITY 실측·검증 절차" §2-3).
VELOCITY_LSB_TO_RAD_S = 0.229 * 2.0 * math.pi / 60.0

# HARDWARE_ERROR_STATUS(70,1) ~ PRESENT_POSITION(132,4) 은 X-시리즈 컨트롤 테이블에서
# 연속 주소 범위라, 70부터 66바이트를 한 번의 SyncRead 로 받아 fault/load/position 을
# 함께 추출(버스 트랜잭션 1회). 중간의 다른 필드(Profile Accel/Velocity 등)도 같이
# 읽히지만 안 쓰고 버림 — 주소가 연속이기만 하면 여분을 읽는 건 무해함.
# (XL430/XC430/XM 계열 공통. 다른 모델이면 주소 재확인 필요 — CLAUDE.md §8 모터모델 미확정.)
ADDR_SYNC_READ_START = ADDR_HARDWARE_ERROR_STATUS
LEN_SYNC_READ = (ADDR_PRESENT_POSITION + LEN_PRESENT_POSITION) - ADDR_HARDWARE_ERROR_STATUS  # = 66

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

PROTOCOL_VERSION = 2.0
BAUDRATE = 1000000
DEVICENAME = "/dev/ttyUSB0"

# tick 상수와 캘리브 측정식의 단일 출처는 calib_math 다(ROS 비의존 → pytest 로 고정).
# 여기서 재노출하는 이유는 `scripts/measure_*.py` 와 외부 코드가 예전부터 이 모듈에서
# 가져다 쓰고 있어서다 — import 경로를 깨지 않으면서 정의는 한 곳으로 모은다.
DXL_MINIMUM_POSITION_VALUE = calib_math.DXL_MINIMUM_POSITION_VALUE
DXL_MAXIMUM_POSITION_VALUE = calib_math.DXL_MAXIMUM_POSITION_VALUE
DXL_CENTER_POSITION = calib_math.DXL_CENTER_POSITION

TICKS_PER_RAD = calib_math.TICKS_PER_RAD
DXL_TICKS_PER_REV = calib_math.DXL_TICKS_PER_REV  # Present Velocity 환산용


# 팔 관절 ↔ 다이나믹셀 ID 매핑.
#
# 2026-08-07 실기 버스 스캔에 맞춰 갱신. 그 전까지 이 dict 는 id 0/1/2 를 가리켰는데
# **버스에 존재하지 않는 ID** 라 팔 서보 토크 인가가 전부 실패했다. 값의 출처는
# teleop_core_node.py 의 DEFAULT_MOTOR_IDS/DEFAULT_DIRECTIONS(2026-08-01/08-02 벤치
# 실측) 이며, 이 두 파일은 같은 물리 버스를 가리키므로 **한쪽을 바꾸면 다른 쪽도
# 같이 바꿔야 한다.**
#
# ⚠️ arm_joint_1(ID 11, 베이스 요축)은 **모터가 물리적으로 없다**(2026-08-07 사용자
#    확인). 스캔에도 안 잡혀서 아예 등록하지 않는다 — 등록해두면 매 tick 무응답으로
#    fault 가 서게 된다. 모터가 붙으면 여기에 한 줄 추가하면 된다.
#
# `gear_ratio` = 서보축 회전 / 관절 회전. 1.0 이면 직결.
#
# arm_joint_2/3 값은 2026-08-07 `scripts/measure_gear_ratio.py` 로 실측했다(토크를
# 끄고 관절을 손으로 돌려 서보각 변화 / 관절각 변화). **두 축의 감속비는 서로 다르다**
# (사용자 확인) — 그 전까지 teleop_core_node.py 주석이 두 축을 묶어 "약 10:1" 로
# 추정했지만 실측 결과 arm_joint_2 만 10:1 에 가깝고 arm_joint_3 은 절반 이하다.
#
# ⚠️ 실측 정밀도는 관절각을 얼마나 정확히 쟀는지에 달려 있다(90° 를 ±9° 오차로 재면
#    기어비도 약 ±10% 흔들린다). 파지 위치가 계통적으로 어긋나면 이 값부터 의심할 것.
#    재측정 없이 시험할 땐 `gear_ratios` 파라미터로 덮어쓸 수 있다.
#    `center`(영점)도 같은 방식으로 `centers` 파라미터가 덮어쓴다
#    (`scripts/measure_zero_offset.py` 또는 관제 GUI 의 영점 마법사 결과를 바로 시험).
#
# 🔒 **모를 때는 낮은 값을 쓴다.** 기어비를 실제보다 낮게 잡으면 관절이 명령보다 덜
#    움직여(언더슈트) 안전하지만, 높게 잡으면 그 배수만큼 과주행해 구조물을 때린다.
#
# `extended` = Extended Position Control Mode(다회전) 축. tick 이 0~4095 를 넘어가고
# 음수도 나오므로 부호 있는 정수로 해석해야 한다(teleop_core 의 EXTENDED_POSITION_NAMES 와 짝).
#
# `center` = 관절 0도에 해당하는 tick. 2026-08-07 `scripts/measure_zero_offset.py` 로
# URDF home 자세(전 관절 0도)에서 실측했다. 그 전까지는 전 축 2048(서보 중앙값)이라는
# **검증된 적 없는 가정**이었고, 실제로 축마다 최대 1100 tick(≈97°) 어긋나 있었다.
# 기어비와 마찬가지로 영점이 틀리면 IK 결과가 통째로 그만큼 어긋난다.
# ⚠️ 팔을 분해·재조립하거나 서보를 뿔에서 뺐다 끼우면 이 값은 무효다 — 다시 측정할 것.
#
# 🔁 **2026-08-09 재측정.** `extended` 축(arm_joint_2/3)의 다회전 카운트는 전원을 내리면
#    초기화되므로, 한 바퀴(4096) 밖의 center 는 그걸 잰 전원 세션 안에서만 유효하다.
#    실제로 arm_joint_3 의 구 center=4281 은 전원 사이클 후 관절각을 -1.58 rad 로
#    읽게 만들었다(안전범위는 0~2.034) — 그대로 구동하면 틀린 기준점 위에서 +90°
#    스윙한다. 아래 값은 그래서 다시 잰 것이다.
#      교차검증: 순수 카운트 초기화라면 새 center 는 4281-4096=185 여야 하는데 278 이
#      나왔다(관절 2.0° 차) — 즉 -87° 편차는 전부 카운트 초기화分이고 자세 재현
#      오차는 2° 수준이었다는 뜻.
#
# ⚠️ **직결(1:1) 축인 arm_joint_4/5 는 토크를 끄면 중력으로 흘러내린다** — 감속기 축
#    (9:1, 4:1)은 역구동이 안 돼 손으로 세운 자세가 유지되지만 이 둘은 손을 떼면 처진다.
#    영점 측정은 반드시 **팔을 붙잡은 상태에서** 할 것.
#
# 🔁 **2026-08-19 3차 재측정 (현재 값).** 팔 전체 재조립 → 전 축 영점 무효.
#    같은 회차에 arm_joint_1(ID 11, XM540-W270)이 새로 실장된 것도 확인됐다.
#    ⚠️ extended 축(arm_joint_2/3)의 아래 center 는 **이 전원 세션 안에서만** 유효하다.
#
# 🔁 **2026-08-09 2차 재측정** (이력). 1차 측정 후 실기 구동 중 손목(arm_joint_5)
#    결합이 물리적으로 빠져 재조립했고, 서보 전원도 내려갔다 — 둘 다 영점 무효 사유다.
#    재조립 후 측정에서는 붙잡은 상태의 드리프트가 8초간 arm_joint_2/5 **정확히 0.000°**
#    로 나왔다.
#    ⚠️ 1차 측정 때 arm_joint_5 가 붙잡고 있는데도 +1.5°/s 로 미끄러졌던 것은 중력이
#       아니라 **결합이 이미 헐거웠다는 신호**였다(수리 후 그 드리프트가 완전히 사라진
#       것으로 확인). 어떤 축이 "잡아도 계속 흐르면" 측정을 계속하지 말고 결합부터
#       점검할 것 — 그대로 두면 구동 중 빠진다.
JOINT_CONFIG = {
    # 🆕 **2026-08-19 신설.** 이 축은 그동안 "모터가 물리적으로 없다"는 전제로
    # STATIC_JOINTS 에 0.0 고정 발행돼 있었는데, 재조립 후 버스 스캔에서 ID 11
    # (XM540-W270)이 정상 응답했다. 사용자 확인 결과 기구에도 물려 있다.
    # gear_ratio 1.0 = **감속기 없는 직결**이다(2026-08-19 사용자 확인) — 측정으로
    # 나온 값이 아니라 기구가 그렇다. center 는 같은 날 zero_offset 실측값.
    "arm_joint_1": {"id": 11, "center": 2081, "direction": 1,
                    "gear_ratio": 1.0, "extended": False},
    # 2026-08-07 실측: 9.034:1 (관절 90° 회전 기준)
    "arm_joint_2": {"id": 14, "center": 506, "direction": -1,
                    "gear_ratio": 9.034, "extended": True},
    # 2026-08-07 실측: 4.040:1 — arm_joint_2 와 다른 감속기다(오타 아님)
    "arm_joint_3": {"id": 13, "center": 1855, "direction": 1,
                    "gear_ratio": 4.040, "extended": True},
    "arm_joint_4": {"id": 12, "center": 1184, "direction": 1,
                    "gear_ratio": 1.0, "extended": False},
    "arm_joint_5": {"id": 16, "center": 675, "direction": 1,
                    "gear_ratio": 1.0, "extended": False},
}
ARM_IDS = {config["id"] for config in JOINT_CONFIG.values()}
ARM_ID_SEQUENCE = [config["id"] for config in JOINT_CONFIG.values()]
ARM_TEST_SEQUENCE = ((14, 5), (13, 10), (12, 10), (16, 20))
RANDOM_ARM_RANGES = {14: 20, 13: 40, 12: 40, 16: 80}
RECORDED_PATH_IDS = (14, 13, 12)
RECORDED_PATH_START_TOLERANCE = {14: 20, 13: 30, 12: 20}
RECORDED_PATH_MAX_WAYPOINT_STEP = 50

# 모터가 없어 실측할 수 없지만 **URDF 상으로는 존재하는** 관절 — 고정값으로 발행한다.
#
# arm_joint_1(베이스 요축)은 서보가 물리적으로 없다(2026-08-07). 그런데 URDF 에서는
# `link_002 → link_004` 를 잇는 관절이라, /joint_states 에 값이 없으면
# `robot_state_publisher` 가 이 관절을 못 넘어가 **TF 트리가 두 조각으로 갈린다**
# ("Tf has two or more unconnected trees") → `base_link → link_043`(tip) 변환이 아예
# 안 만들어져서 arm_fsm 의 IK/carry pose 계산이 전부 실패한다. MoveIt 도 5축 전체
# 관절값을 기대하므로 같은 이유로 필요하다.
#
# 값의 근거:
#   arm_joint_2/3/4 는 축이 서로 평행해서 이 팔은 **평면 로봇**이고, 그 평면의 방위를
#   정하는 유일한 관절이 arm_joint_1 이다(축 0 0 1, 회전중심은 base_link 원점 위
#   z=0.0465). 즉 이 값이 틀리면 팔이 향하는 방향 전체가 틀린다.
#
# 🔁 **2026-08-12 정정: 1.405 → 0.0.**
#   이 값은 그동안 +1.405 rad 였다. 근거는 "0.0 이면 FK 가 그리퍼를 방위각 -80.5°
#   (거의 정오른쪽)에 놓는데, 실기의 팔은 정면(+x)을 향하므로 방위각을 0 으로 돌려야
#   한다" 는 것이었다 — 그 **전제가 틀렸다. 팔은 실제로 오른쪽으로 틀어져 있다**
#   (2026-08-12 사용자 확인).
#
#   확인 방법(같은 방식으로 재확인 가능): 카메라 TF 캘리브를 마친 상태에서 박스를
#   그리퍼 정면에 놓고 RViz 를 본다. joint_1=0 인 모델의 tip(link_043)은 방위각
#   -80.7°/반경 15.5cm 에 서고, 카메라가 본 박스는 -90.7°/46.1cm 에 찍혔다 —
#   **두 방향이 10° 안쪽으로 일치**했고 화면상으로도 박스가 그리퍼 앞이었다.
#   1.405 를 쓰면 이 관계가 통째로 80° 어긋난다.
#
#   ⚠️ 그 오차는 조용하다: 브릿지가 떠 있는 동안에만 TF 가 80° 돌아가므로, RViz 만
#      띄워 보면(jsp_gui 가 0 을 발행) 멀쩡해 보이고 arm_fsm 을 붙였을 때만 목표가
#      틀어진다. "인식·캘리브는 맞는데 팔이 엉뚱한 데로 간다" 면 여기를 볼 것.
#
# ⚠️ 이 축은 모터가 없다. **기구적으로 고정돼 있다는 전제**이며, 만약 자유회전
#    상태라면 팔의 평면이 운용 중 돌아가고 이 값은 무의미해진다 — 그 경우 IK 목표가
#    조용히 틀어지므로, 물리적으로 고정돼 있는지 반드시 확인할 것.
#    팔을 재장착했다면 위 확인 절차를 다시 밟을 것.
# 🔁 **2026-08-19 비움.** arm_joint_1 은 ID 11 서보가 실재하는 것이 확인돼
#    JOINT_CONFIG 로 옮겨졌다(위 참고). 이제 실측값이 /joint_states 로 나가므로
#    고정 발행이 필요 없다 — 고정값을 남겨두면 실제 서보각과 충돌한다.
#    아래 dict 와 발행 경로는 다음에 모터 없는 축이 생길 때를 위해 남겨둔다.
STATIC_JOINTS = {}

# X 시리즈 Extended Position Control Mode 의 raw tick 한계(약 ±256회전).
DXL_EXTENDED_MIN_TICK = calib_math.DXL_EXTENDED_MIN_TICK
DXL_EXTENDED_MAX_TICK = calib_math.DXL_EXTENDED_MAX_TICK


#: 캘리브 파라미터의 "비어 있음" 기본값.
#:
#: ⚠️ **`[]` 을 쓰면 안 된다.** rclpy 는 빈 리스트에서 타입을 추론하지 못해
#: `BYTE_ARRAY` 로 선언해 버리고, 그러면 런타임 `set_parameters` 가
#: *"Wrong parameter type, expected 'Type.BYTE_ARRAY' got 'Type.STRING_ARRAY'"* 로
#: **거절된다**(2026-08-12 실기 확인). CLI `-p gear_ratios:=` 는 선언 시점에 값을
#: 덮어써서 멀쩡히 동작하므로, **런타임에 처음 바꿔 볼 때까지 드러나지 않는다.**
#: `ParameterDescriptor(type=...)` 로도 추론을 못 바꾼다 — 빈 문자열 하나가 답이다.
#: 파서가 이름 없는 항목을 건너뛰므로 의미상으로는 "없음" 그대로다.
EMPTY_STR_ARRAY = [""]


# 캘리브 파라미터(`gear_ratios`·`centers`)의 파서. rclpy 에 dict 타입 파라미터가 없어
# "<joint>:<값>" 문자열 배열로 받는다. 기동 시와 런타임 변경(파라미터 콜백)이 **같은
# 검증**을 쓰도록 함수로 뺐다 — 한쪽만 느슨하면 기동은 되는데 변경은 거절되는 식이 된다.
def _parse_gear_ratios(entries):
    """`["arm_joint_2:9.034", …]` → `({이름: 비}, [오류 사유])`."""
    out, errors = {}, []
    for entry in entries or []:
        name, _, value = str(entry).partition(":")
        if not name:
            continue                       # 빈 문자열은 "없음" 으로 본다(기본값 [""])
        if name not in JOINT_CONFIG:
            errors.append(f"모르는 관절 '{name}'")
            continue
        try:
            ratio = float(value)
        except ValueError:
            errors.append(f"'{entry}' 파싱 실패")
            continue
        if ratio <= 0.0:
            errors.append(f"'{entry}' 은 양수여야 함")
            continue
        out[name] = ratio
    return out, errors


def _parse_centers(entries):
    """`["arm_joint_2:1627", …]` → `({이름: tick}, [오류 사유])`."""
    out, errors = {}, []
    for entry in entries or []:
        name, _, value = str(entry).partition(":")
        if not name:
            continue
        if name not in JOINT_CONFIG:
            errors.append(f"모르는 관절 '{name}'")
            continue
        try:
            center = int(round(float(value)))
        except ValueError:
            errors.append(f"'{entry}' 파싱 실패")
            continue
        reason = calib_math.center_out_of_range(center, JOINT_CONFIG[name]["extended"])
        if reason is not None:
            errors.append(f"{name}: {reason}")
            continue
        out[name] = center
    return out, errors


# Profile Acceleration(108) / Velocity(112). 기본값 0 은 "최고속 즉시 이동" 이라
# 그리퍼가 움직일 때마다 순간 과전류로 토크가 풀린다(HW-8 실기, 재현율 100%,
# 명령 후 0.3초 내 트립). 트립이 풀리면 Hardware Error Status 도 0 으로 돌아가
# 나중에 보면 흔적이 안 남는다 — 반드시 기동 시 넣어야 한다(CLAUDE.md).
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
PROFILE_ACCELERATION = 25
PROFILE_VELOCITY = 80


def to_signed(value, byte_len):
    """무부호 정수를 byte_len 바이트 2의 보수 부호 정수로 변환."""
    bits = byte_len * 8
    if value >= (1 << (bits - 1)):
        value -= (1 << bits)
    return value


class MoveItDynamixelBridge(Node):
    def __init__(self):
        super().__init__("moveit_dynamixel_bridge")

        self.declare_parameter("read_only", False)
        self.declare_parameter("mock_mode", False)
        self.declare_parameter("tool_type", "spur_1motor_gripper")
        self.declare_parameter("control_scope", "FULL_ROBOT")
        self.declare_parameter("temporary_jog_mode", False)
        self.declare_parameter("temporary_jog_safe_min_tick", 2867)
        self.declare_parameter("temporary_jog_safe_max_tick", 3807)
        self.declare_parameter("temporary_jog_mechanical_open_tick", 2817)
        self.declare_parameter("temporary_jog_mechanical_close_tick", 3857)
        self.declare_parameter("gripper_target_tolerance_ticks", 20)
        default_profiles = str(Path(get_package_share_directory(
            'dynamixel_control')) / 'config' / 'tool_profiles.yaml')
        self.declare_parameter("tool_profile_file", default_profiles)
        # 털털이 ZIP에 모터 ID/방향/속도가 없어 모두 fail-closed 기본값이다.
        self.declare_parameter("cleaning_actuator_joint", "")
        self.declare_parameter("cleaning_actuator_id", -1)
        self.declare_parameter("cleaning_direction", 0)
        self.declare_parameter("cleaning_velocity_raw", 0)

        self.read_only = bool(self.get_parameter("read_only").value)
        self.mock_mode = bool(self.get_parameter("mock_mode").value)
        self.tool_type = str(self.get_parameter("tool_type").value)
        self.control_scope = validate_control_scope(
            self.get_parameter("control_scope").value)
        self.temporary_jog_mode = bool(
            self.get_parameter("temporary_jog_mode").value)
        self.temporary_jog_safe_min = int(
            self.get_parameter("temporary_jog_safe_min_tick").value)
        self.temporary_jog_safe_max = int(
            self.get_parameter("temporary_jog_safe_max_tick").value)
        self.temporary_jog_enabled = bool(
            self.temporary_jog_mode
            and self.control_scope == 'END_EFFECTOR_ONLY'
            and self.tool_type == 'spur_1motor_gripper'
            and self.temporary_jog_safe_min < self.temporary_jog_safe_max)
        self.gripper_target_tolerance = int(
            self.get_parameter("gripper_target_tolerance_ticks").value)
        if self.gripper_target_tolerance < 0:
            raise ValueError('gripper_target_tolerance_ticks must be non-negative')
        self.control_mode = 'FSM'
        self.emergency_stop_active = False
        self.tool_detached = False
        self._tool_samples = {}
        self._gripper_goal_active = False
        self._gripper_goal_lock = threading.Lock()
        self._bus_lock = threading.RLock()
        self.cleaning_actuator_joint = self.get_parameter("cleaning_actuator_joint").value
        self.cleaning_actuator_id = int(self.get_parameter("cleaning_actuator_id").value)
        self.cleaning_direction = int(self.get_parameter("cleaning_direction").value)
        self.cleaning_velocity_raw = int(self.get_parameter("cleaning_velocity_raw").value)
        self.cleaning_configured = (
            bool(self.cleaning_actuator_joint) and self.cleaning_actuator_id >= 0
            and self.cleaning_direction in (-1, 1) and self.cleaning_velocity_raw > 0
        )
        try:
            profiles = load_profiles(self.get_parameter('tool_profile_file').value)
            if self.tool_type == 'cleaner' and self.cleaning_configured:
                profiles['cleaner'].update({
                    'calibrated': True,
                    'actuator_ids': [self.cleaning_actuator_id],
                    'joint_names': [self.cleaning_actuator_joint],
                    'direction': self.cleaning_direction,
                    'profile_velocity': self.cleaning_velocity_raw,
                    'profile_acceleration': 1,
                })
            self.tool_manager = ToolManager(
                profiles, ParameterToolIdentityProvider(self.tool_type),
                mock_mode=self.mock_mode)
            self.tool_selection = self.tool_manager.refresh('IDLE')
        except (ToolProfileError, KeyError) as exc:
            self.get_logger().error(f'tool profile rejected: {exc}')
            self.tool_selection = None
        self.tool_motion_allowed = bool(
            self.tool_selection and self.tool_selection.valid
            and not self.read_only and not self.mock_mode)
        if self.temporary_jog_enabled and not self.read_only and not self.mock_mode:
            # The calibrated profile remains invalid; temporary mode only permits
            # the explicitly bounded single-actuator jog path below.
            self.tool_motion_allowed = True
        self.tool_profile = (
            self.tool_selection.profile if self.tool_selection else {})

        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        self.port_connected = self.mock_mode
        if not self.mock_mode:
            try:
                self.port_connected = bool(self.port_handler.openPort())
                if self.port_connected:
                    self.port_connected = bool(
                        self.port_handler.setBaudRate(BAUDRATE))
            except Exception as exc:
                self.port_connected = False
                self.get_logger().error(f'Cannot open {DEVICENAME}: {exc}')
            if not self.port_connected and not self.read_only:
                raise RuntimeError(f"Failed to open/configure port: {DEVICENAME}")

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
        # gripper-only/read-only에서는 register write 없이 그리퍼 ID만 active_ids에 등록된다.
        self.active_ids = set()
        self.torque_enabled_ids = set()

        if not self.read_only and not self.mock_mode:
            if self.control_scope == 'FULL_ROBOT':
                # 팔 서보: 토크 ON 성공한 ID만 SyncRead 등록
                for joint_name, config in JOINT_CONFIG.items():
                    if self._enable_torque(config["id"], joint_name):
                        self.group_sync_read.addParam(config["id"])
                        self.active_ids.add(config["id"])
            if self.cleaning_configured:
                self._configure_cleaning_actuator()

        self.tool_ids = list(self.tool_profile.get('actuator_ids', []))
        self.tool_discovered = self.mock_mode
        if not self.mock_mode and self.port_connected:
            self.tool_discovered = self._discover_tool_ids()
            if self.read_only:
                for dxl_id in self.tool_ids:
                    self.group_sync_read.addParam(dxl_id)
                    self.active_ids.add(dxl_id)
            elif self.tool_motion_allowed and self.tool_discovered:
                if self.temporary_jog_enabled:
                    self._configure_temporary_jog_actuator()
                else:
                    self._configure_tool_actuators()
            elif self.tool_ids and not self.tool_discovered:
                self.tool_motion_allowed = False

        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            self.trajectory_callback,
            10,
        )
        self.create_subscription(Bool, "/cleaning/enable", self._on_cleaning_enable, 10)
        self.create_subscription(Bool, "/tool/emergency_stop", self._on_emergency_stop, 10)
        self.create_subscription(Bool, "/tool/detached", self._on_tool_detached, 10)
        self.create_subscription(
            String, "/control/mode_status", self._on_control_mode, 10)

        # 벤치 teleop_core의 단일 관절 명령. 메시지는 [motor_id, goal_tick].
        # FSM/MoveIt 경로와 같은 GroupSyncWrite를 사용하되 알려진 팔 ID만 허용한다.
        # 토크 on/off 요청 — `position_node` 와 **같은 토픽·같은 포맷**을 쓴다
        # (`[enable, id...]`). 벤치 텔레옵 쪽에만 있던 인터페이스라 브릿지 경로에서는
        # 팔을 손으로 만지려면 스택을 통째로 내리는 수밖에 없었다. 어휘를 새로 만들지
        # 않고 기존 것을 그대로 받아, `teleop_core` 의 stop/freedrive 나 관제 GUI 버튼이
        # 어느 런타임에서든 같은 뜻을 갖게 한다.
        # 확장 한 가지: id 목록을 생략하면(`[enable]`) **등록된 전 축**에 적용한다 —
        # 요청자가 서보 ID 를 몰라도 되게 하기 위함이다(mission_console 이 이걸 쓴다).
        self.torque_request_sub = self.create_subscription(
            Int32MultiArray, "/dynamixel/torque_request",
            self.torque_request_callback, 10)

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
        )
        self.gripper_action_server = ActionServer(
            self, FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory",
            execute_callback=self.execute_gripper,
            goal_callback=self.gripper_goal_callback,
            cancel_callback=self.gripper_cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.rotate_action_server = ActionServer(
            self,
            EndEffectorRotate,
            "/end_effector/rotate",
            execute_callback=self.execute_rotate,
            goal_callback=self.rotate_goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._action_group,
        )

        self.arm_test_action_server = ActionServer(
            self,
            ArmTestMove,
            "/arm/test_move",
            execute_callback=self.execute_arm_test_move,
            goal_callback=self.arm_test_goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._action_group,
        )

        self.arm_recorded_path_action_server = ActionServer(
            self,
            ArmRecordedPath,
            "/arm/recorded_path",
            execute_callback=self.execute_arm_recorded_path,
            goal_callback=self.arm_recorded_path_goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._action_group,
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
        self.tool_type_pub = self.create_publisher(String, '/tool/type', 10)
        self.tool_status_pub = self.create_publisher(String, '/tool/status', 10)

        self.feedback_timer = self.create_timer(0.05, self.publish_joint_states)
        self.tool_status_timer = self.create_timer(0.5, self.publish_tool_status)

        self.get_logger().info(
            f"MoveIt Dynamixel bridge started (arm={list(JOINT_CONFIG)}, "
            f"cleaning_actuator={self.cleaning_actuator_joint or 'UNCONFIGURED'}, "
            f"tool_type={self.tool_type}, tool_ready={self.tool_motion_allowed}, "
            f"control_scope={self.control_scope}, "
            f"temporary_jog={self.temporary_jog_enabled}, "
            f"read_only={self.read_only}, mock_mode={self.mock_mode})"
        )

    # ------------------------------------------------------------------ helpers
    def _write_motion_profile(self, dxl_id, label, velocity=None):
        """Profile Acceleration/Velocity 설정 — 토크 인가 **전에** 호출한다.

        기본값 0(=최고속 즉시 이동)이면 그리퍼가 움직일 때마다 순간 과전류로 토크가
        풀린다(HW-8 실기 검증, 재현율 100%). 팔 축도 같은 이유로 완만하게 둔다.

        ⚠️ **여기가 팔의 실제 속도를 정하는 유일한 곳이다.** `trajectory_callback` 은
        `time_from_start` 를 쓰지 않고 goal tick 만 SyncWrite 하므로, 궤적의 duration
        (arm_fsm 의 `arm_move_speed`)은 FSM 내부 타임아웃 추정에만 쓰이고 서보 속도에는
        영향이 없다. 속도를 바꾸려면 `arm_profile_velocity` 를 조정할 것.

        ⚠️ 그리퍼는 팔과 **따로** 둔다(`gripper_profile_velocity`). 그리퍼 속도를 낮추면
        완전 개폐 시간이 늘어나는데, `gripper_presets.gripper_action_time`(2.5s)은 그
        시간을 넘겨야 파지 effort 를 제대로 읽는다 — 같이 낮추면 "닫히는 도중에 판정"
        해서 grasp effort 가 0 으로 읽히는 알려진 실패로 돌아간다.
        """
        if velocity is None:
            velocity = self.arm_profile_velocity
        for addr, value, field in (
            (ADDR_PROFILE_ACCELERATION, self.profile_acceleration, "Profile Acceleration"),
            (ADDR_PROFILE_VELOCITY, velocity, "Profile Velocity"),
        ):
            result, error = self.packet_handler.write4ByteTxRx(
                self.port_handler, dxl_id, addr, value
            )
            if result != 0 or error != 0:
                self.get_logger().warn(
                    f"{field} 설정 실패: {label}, id={dxl_id}, "
                    f"result={result}, error={error} — 과전류 토크 트립 위험"
                )

    def _write_gripper_goal_pwm(self, dxl_id):
        """그리퍼 토크 상한(Goal PWM) 설정 — 파지 중 Overload 트립 방지.

        물체를 문 채 목표에 도달 못 하면 서보는 무한정 밀어붙이다 Overload 로 토크가
        끊긴다(2026-08-09 실기). 여기서 상한을 걸면 그 힘에서 멈춰 계속 물고 있는다.
        값 근거와 조정 방향은 `gripper_presets.py` 의 `gripper_goal_pwm` 주석 참고.
        """
        if self.gripper_goal_pwm <= 0:
            return
        result, error = self.packet_handler.write2ByteTxRx(
            self.port_handler, dxl_id, ADDR_GOAL_PWM, self.gripper_goal_pwm)
        if result != 0 or error != 0:
            self.get_logger().warn(
                f"Goal PWM 쓰기 실패: id={dxl_id}, result={result}, error={error} — "
                "토크 상한이 안 걸려 파지 중 Overload 트립 가능")
        else:
            self.get_logger().info(
                f"Goal PWM 설정: id={dxl_id} -> {self.gripper_goal_pwm} "
                f"(최대 885, 파지 토크 상한)")

    def _required_gripper_mode(self, dxl_id):
        return self.gripper_required_operating_modes.get(
            dxl_id, self.gripper_required_operating_mode)

    def _enable_torque(self, dxl_id, label, required_mode=None):
        """현재 위치를 goal로 검증 동기화한 뒤에만 해당 ID의 torque를 켠다."""
        try:
            with self._bus_lock:
                torque = self._read_register(
                    dxl_id, ADDR_TORQUE_ENABLE, 1, "startup torque")
                if torque != TORQUE_DISABLE:
                    raise RuntimeError(
                        f"startup requires Torque OFF, readback={torque}")
                if required_mode is not None:
                    mode = self._read_register(
                        dxl_id, ADDR_OPERATING_MODE, 1,
                        "startup operating mode")
                    if mode != required_mode:
                        raise RuntimeError(
                            f"operating mode mismatch: expected "
                            f"{required_mode}, read {mode}; automatic mode "
                            "writes are disabled")
                present = self._read_register(
                    dxl_id, ADDR_PRESENT_POSITION, 4,
                    "startup present position", signed=True)
                self._write_register(
                    dxl_id, ADDR_GOAL_POSITION, 4,
                    present & 0xFFFFFFFF, "startup synchronize goal")
                goal_readback = self._read_register(
                    dxl_id, ADDR_GOAL_POSITION, 4,
                    "startup goal readback", signed=True)
                # Startup synchronization is a fail-closed safety gate: even a
                # one-tick mismatch means the value written was not read back
                # exactly, so torque must remain disabled.
                if goal_readback != present:
                    raise RuntimeError(
                        f"Present->Goal readback mismatch: "
                        f"present={present}, goal={goal_readback}")
                self._write_motion_profile(dxl_id, label)
                self._write_register(
                    dxl_id, ADDR_TORQUE_ENABLE, 1,
                    TORQUE_ENABLE, "startup torque enable")
                torque_readback = self._read_register(
                    dxl_id, ADDR_TORQUE_ENABLE, 1,
                    "startup torque readback")
                if torque_readback != TORQUE_ENABLE:
                    raise RuntimeError(
                        f"Torque ON readback failed: {torque_readback}")
        except Exception as exc:
            self.get_logger().error(
                f"Torque enable blocked: {label}, id={dxl_id}: {exc}")
            return False
        self.get_logger().info(f"Torque enabled safely: {label} -> id {dxl_id}")
        return True

    def _joint_gear_ratio(self, joint_name):
        """실측으로 덮어쓸 수 있는 기어비(`gear_ratios` 파라미터 > JOINT_CONFIG 기본값)."""
        return self.gear_ratios.get(joint_name, JOINT_CONFIG[joint_name]["gear_ratio"])

    def _read_register(self, dxl_id, address, size, label, signed=False):
        reader = {
            1: self.packet_handler.read1ByteTxRx,
            2: self.packet_handler.read2ByteTxRx,
            4: self.packet_handler.read4ByteTxRx,
        }[size]
        value, result, error = reader(self.port_handler, dxl_id, address)
        if result != 0 or error != 0:
            raise RuntimeError(
                f"ID {dxl_id} {label} read failed: result={result}, error={error}")
        return to_signed(value, size) if signed else value

    def _write_register(self, dxl_id, address, size, value, label):
        writer = {
            1: self.packet_handler.write1ByteTxRx,
            4: self.packet_handler.write4ByteTxRx,
        }[size]
        result, error = writer(
            self.port_handler, dxl_id, address, value)
        if result != 0 or error != 0:
            raise RuntimeError(
                f"ID {dxl_id} {label} write failed: result={result}, error={error}")

    def _discover_tool_ids(self):
        """Ping every configured actuator; any missing ID closes the backend."""
        if not self.tool_ids:
            return False
        missing = []
        for dxl_id in self.tool_ids:
            _model, result, error = self.packet_handler.ping(
                self.port_handler, dxl_id)
            if result != 0 or error != 0:
                missing.append(dxl_id)
        if missing:
            self.get_logger().error(f'tool actuator IDs not discovered: {missing}')
            return False
        return True

    def _configure_tool_actuators(self):
        """Apply profile motion limits only after strict validation and discovery."""
        if self.tool_profile.get('backend') == 'cleaner':
            return
        modes = self.tool_profile.get('required_operating_modes', {})
        for dxl_id in self.tool_ids:
            mode = modes.get(dxl_id, modes.get(str(dxl_id), 3))
            self.packet_handler.write1ByteTxRx(
                self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            result, error = self.packet_handler.write1ByteTxRx(
                self.port_handler, dxl_id, ADDR_OPERATING_MODE, int(mode))
            if result != 0 or error != 0:
                self.tool_motion_allowed = False
                self.get_logger().error(f'operating mode setup failed: id={dxl_id}')
                return
            for address, value in (
                    (ADDR_PROFILE_ACCELERATION,
                     self.tool_profile['profile_acceleration']),
                    (ADDR_PROFILE_VELOCITY,
                     self.tool_profile['profile_velocity'])):
                self.packet_handler.write4ByteTxRx(
                    self.port_handler, dxl_id, address, int(value))
            goal_pwm = int(self.tool_profile.get('goal_pwm', 0))
            if goal_pwm > 0:
                self.packet_handler.write2ByteTxRx(
                    self.port_handler, dxl_id, ADDR_GOAL_PWM, goal_pwm)
            if self._enable_torque(dxl_id, f'{self.tool_type} tool'):
                self.group_sync_read.addParam(dxl_id)
                self.active_ids.add(dxl_id)
            else:
                self.tool_motion_allowed = False

    def _stop_tool(self, reason):
        """Best-effort stop for emergency, detach, cancellation, and shutdown."""
        self.tool_motion_allowed = False
        if self.mock_mode or self.read_only:
            return
        with self._bus_lock:
            for dxl_id in self.tool_ids:
                self.packet_handler.write4ByteTxRx(
                    self.port_handler, dxl_id, ADDR_GOAL_VELOCITY, 0)
                self.packet_handler.write1ByteTxRx(
                    self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        self.get_logger().warn(f'tool actuator stopped: {reason}')

    def _on_emergency_stop(self, msg):
        if msg.data:
            self.emergency_stop_active = True
            self._stop_tool('emergency stop')

    def _on_tool_detached(self, msg):
        if msg.data:
            self.tool_detached = True
            self._stop_tool('tool detach')

    def _on_control_mode(self, msg):
        mode = msg.data.strip().upper()
        if mode not in ('MANUAL', 'FSM'):
            self.get_logger().warn(f'unknown control mode ignored: {msg.data!r}')
            return
        self.control_mode = mode

    def publish_tool_status(self):
        self.tool_type_pub.publish(String(data=self.tool_type))
        reason = ''
        if not self.tool_selection:
            reason = 'profile load failed'
        elif not self.tool_selection.valid:
            reason = self.tool_selection.reason
        elif not self.tool_discovered and not self.mock_mode:
            reason = 'actuator not discovered'
        elif self.read_only:
            reason = 'read-only diagnostic mode'
        status = {
            'control_scope': self.control_scope,
            'tool_type': self.tool_type,
            'backend': self.tool_profile.get('backend', 'invalid'),
            'profile_valid': bool(self.tool_selection and self.tool_selection.valid),
            'calibrated': bool(self.tool_profile.get('calibrated')),
            'temporary_jog_mode': self.temporary_jog_enabled,
            'temporary_jog_ready': self._tool_backend_ready(),
            'actuators_discovered': self.tool_discovered,
            'motion_allowed': self._tool_backend_ready(),
            'read_only': self.read_only, 'mock_mode': self.mock_mode,
            'bridge_connected': True,
            'u2d2_connected': self.port_connected,
            'control_mode': self.control_mode,
            'emergency_stop': self.emergency_stop_active,
            'tool_detached': self.tool_detached,
            'actuators': [self._tool_samples.get(dxl_id, {
                'id': dxl_id, 'joint': '', 'position': None,
                'effort': 0.0 if self.mock_mode else None,
                'online': self.mock_mode}) for dxl_id in self.tool_ids],
            'reason': reason,
        }
        self.tool_status_pub.publish(String(data=json.dumps(status, sort_keys=True)))

    def _tool_actuators_online(self):
        """Return true only when every profile-selected actuator is online."""
        if not self.tool_ids:
            return False
        return all(
            (self._tool_samples.get(dxl_id) or {}).get('id') == dxl_id
            and bool((self._tool_samples.get(dxl_id) or {}).get('online'))
            for dxl_id in self.tool_ids)

    def _tool_backend_ready(self):
        if self.mock_mode:
            return True
        profile_ready = bool(self.tool_selection and self.tool_selection.valid
                             and self.tool_profile.get('calibrated'))
        temporary_ready = bool(
            self.temporary_jog_enabled
            and self.tool_type == 'spur_1motor_gripper'
            and self.tool_ids == [5])
        return bool(
            (profile_ready or temporary_ready)
            and self.tool_discovered and self._tool_actuators_online()
            and self.tool_motion_allowed and not self.read_only
            and not self.emergency_stop_active and not self.tool_detached)

    def _configure_temporary_jog_actuator(self):
        if self.tool_ids != [5]:
            self.tool_motion_allowed = False
            return
        dxl_id = self.tool_ids[0]
        if self._enable_torque(dxl_id, 'spur temporary jog'):
            self.group_sync_read.addParam(dxl_id)
            self.active_ids.add(dxl_id)
        else:
            self.tool_motion_allowed = False

    def _configure_cleaning_actuator(self):
        """Dynamixel Protocol 2.0 velocity mode(Operating Mode=1)로 설정한다."""
        dxl_id = self.cleaning_actuator_id
        self.packet_handler.write1ByteTxRx(
            self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, dxl_id, ADDR_OPERATING_MODE, 1)
        if result != 0 or error != 0:
            self.get_logger().error(
                f"Cleaning actuator velocity-mode setup failed: id={dxl_id}")
            self.cleaning_configured = False
            return
        if self._enable_torque(dxl_id, self.cleaning_actuator_joint):
            self.group_sync_read.addParam(dxl_id)
            self.active_ids.add(dxl_id)
        else:
            self.cleaning_configured = False

    def _on_cleaning_enable(self, msg):
        if (self.read_only or self.mock_mode or not self.cleaning_configured
                or self.tool_type != 'cleaner'
                or self.control_mode != 'MANUAL'
                or not self._tool_backend_ready()):
            if msg.data:
                self.get_logger().error(
                    "Cleaning command rejected: actuator ID/direction/velocity not configured")
            return
        velocity = self.cleaning_direction * self.cleaning_velocity_raw if msg.data else 0
        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, self.cleaning_actuator_id, ADDR_GOAL_VELOCITY,
            velocity & 0xffffffff)
        if result != 0 or error != 0:
            self.get_logger().error(
                f"Cleaning velocity write failed: result={result}, error={error}")

    def rad_to_tick(self, joint_name, rad):
        """관절 rad → 서보 tick. 안전 리밋 clamp 후 기어비를 곱해 서보축 도메인으로 올린다.

        tick 범위 clamp(아래)는 "서보가 표현할 수 있는 값" 일 뿐 "관절이 안 부딪히는
        범위" 가 아니다 — 그래서 joint_limits 를 **먼저** 적용한다.
        """
        config = JOINT_CONFIG[joint_name]
        # clamp 를 조용히 하면 IK 버그가 "왜 목표에 안 닿지?" 로만 보인다 — 반드시 남긴다.
        rad, was_clamped = joint_limits.clamp(joint_name, rad)
        if was_clamped:
            lower, upper = joint_limits.get_limits(joint_name)
            self.get_logger().warn(
                f"{joint_name}: 목표각이 안전 범위를 벗어나 clamp 됨 "
                f"→ {rad:+.4f} rad (범위 [{lower:+.4f}, {upper:+.4f}])"
            )
        tick = int(round(calib_math.rad_to_tick(
            self._joint_center(joint_name), config["direction"],
            self._joint_gear_ratio(joint_name), rad)))
        if config["extended"]:
            return max(DXL_EXTENDED_MIN_TICK, min(DXL_EXTENDED_MAX_TICK, tick))
        return max(DXL_MINIMUM_POSITION_VALUE, min(DXL_MAXIMUM_POSITION_VALUE, tick))

    def tick_to_rad(self, joint_name, tick):
        """서보 tick → 관절 rad. rad_to_tick 의 역변환."""
        return calib_math.tick_to_rad(
            self._joint_center(joint_name), JOINT_CONFIG[joint_name]["direction"],
            self._joint_gear_ratio(joint_name), tick)

    def int_to_little_endian_4bytes(self, value):
        return [
            value & 0xFF,
            (value >> 8) & 0xFF,
            (value >> 16) & 0xFF,
            (value >> 24) & 0xFF,
        ]

    def _tool_position_tick(self, dxl_id, raw_tick):
        endpoints = self.tool_profile.get('motor_endpoints') or {}
        endpoint = endpoints.get(dxl_id, endpoints.get(str(dxl_id), {}))
        if any(value is not None and value < 0 for value in endpoint.values()):
            return to_signed(raw_tick, LEN_PRESENT_POSITION)
        return int(raw_tick)

    def goal_callback(self, goal_request):
        if self.control_scope == 'END_EFFECTOR_ONLY':
            self.get_logger().warn(
                'Arm trajectory rejected: control_scope=END_EFFECTOR_ONLY')
            return GoalResponse.REJECT
        if (self.read_only or not self.tool_selection
                or not self.tool_selection.valid
                or (not self.mock_mode and not self.tool_motion_allowed)):
            self.get_logger().warn(
                "Arm trajectory rejected: read-only or tool interlock not ready")
            return GoalResponse.REJECT
        self.get_logger().info("Received FollowJointTrajectory goal")
        return GoalResponse.ACCEPT

    def _gripper_commands_allowed(self):
        """캘리브레이션된 위치 명령 동작일 때만 참을 반환한다."""
        if (not self.gripper_command_calibrated
                or self.gripper_open_rad == self.gripper_close_rad):
            return False
        if self.gripper_required_operating_modes:
            return (set(self.gripper_required_operating_modes)
                    == set(self.gripper_ids)
                    and self.gripper_observed_operating_modes
                    == self.gripper_required_operating_modes)
        return (self.gripper_observed_operating_mode
                == self.gripper_required_operating_mode)

    def _require_gripper_command_mapping(self):
        """안전성이 확인되지 않은 rad→tick 변환을 fail-closed로 차단한다."""
        if not self.gripper_command_calibrated:
            raise RuntimeError("gripper command calibration is not verified")
        if self.gripper_open_rad == self.gripper_close_rad:
            raise RuntimeError("gripper open/close rad endpoints are identical")

    def _gripper_startup_torque_allowed(self):
        """듀얼 그리퍼에만 기존 기동 동작을 허용한다."""
        return self.end_effector_kind == "gripper" \
            and self._gripper_commands_allowed()

    def gripper_goal_callback(self, goal_request):
        """모드와 끝점의 시운전이 끝날 때까지 그리퍼 동작을 거부한다."""
        if (getattr(self, "gripper_disabled", False)
                or self.end_effector_kind != "gripper" or self.read_only
                or not self._gripper_commands_allowed()):
            self.get_logger().warn(
                "Rejecting gripper goal: command calibration/mode gate closed")
            return GoalResponse.REJECT
        return self.goal_callback(goal_request)

    def rotate_goal_callback(self, goal_request):
        """선택한 단일축 회전 프리셋에 대해서만 회전을 수락한다."""
        if (self.read_only or self.end_effector_kind != "rotary"
                or self.gripper_ids != [5]
                or not self._gripper_commands_allowed()):
            self.get_logger().warn(
                "Rejecting rotate goal: rotary_id5 preset is not active")
            return GoalResponse.REJECT
        if goal_request.max_abs_current < 0 or goal_request.timeout < 0.0:
            return GoalResponse.REJECT
        if not goal_request.relative and not 0 <= goal_request.ticks <= 4095:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def arm_test_goal_callback(self, goal_request):
        """명시적으로 승인된 4축 tick 시퀀스만 수락한다."""
        requested = tuple(zip(goal_request.motor_ids, goal_request.delta_ticks))
        fixed_sequence = requested == ARM_TEST_SEQUENCE
        random_sequence = (
            bool(goal_request.random_demo)
            and self.random_demo_enabled
            and tuple(goal_request.motor_ids) == tuple(ARM_ID_SEQUENCE)
            and len(goal_request.delta_ticks) == len(ARM_ID_SEQUENCE)
            and all(abs(int(delta)) <= 2 * RANDOM_ARM_RANGES[dxl_id]
                    for dxl_id, delta in requested)
        )
        if (self.read_only or not self.integrated_test_mode
                or self.gripper_only_mode
                or self.end_effector_kind != "rotary"
                or self.gripper_ids != [5]
                or not (fixed_sequence and not goal_request.random_demo
                        or random_sequence)):
            self.get_logger().warn(
                "Rejecting arm test goal: mode/preset/sequence gate closed")
            return GoalResponse.REJECT
        if (goal_request.max_abs_current < 0
                or goal_request.stall_timeout < 0.0
                or goal_request.step_timeout < 0.0):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def split_recorded_path_request(goal_request):
        """평탄화된 signed 기록 경로 요청을 검증하고 분리한다."""
        motor_ids = tuple(int(v) for v in goal_request.motor_ids)
        counts = tuple(int(v) for v in goal_request.waypoint_counts)
        flat = tuple(int(v) for v in goal_request.signed_waypoints)
        if motor_ids != RECORDED_PATH_IDS:
            raise ValueError(
                f"motor_ids must be exactly {list(RECORDED_PATH_IDS)}")
        if len(counts) != len(motor_ids) or any(count <= 0 for count in counts):
            raise ValueError("one positive waypoint_count is required per motor")
        if sum(counts) != len(flat):
            raise ValueError("waypoint_counts do not match signed_waypoints")

        paths = []
        offset = 0
        for dxl_id, count in zip(motor_ids, counts):
            waypoints = flat[offset:offset + count]
            offset += count
            if dxl_id == 12 and any(not 0 <= value <= 4095
                                    for value in waypoints):
                raise ValueError("ID 12 Mode 3 waypoint outside [0, 4095]")
            deltas = [b - a for a, b in zip(waypoints, waypoints[1:])]
            if any(delta == 0 or abs(delta) > RECORDED_PATH_MAX_WAYPOINT_STEP
                   for delta in deltas):
                raise ValueError("waypoint step must be in [1, 50] ticks")
            signs = {1 if delta > 0 else -1 for delta in deltas}
            if len(signs) > 1:
                raise ValueError(f"ID {dxl_id} waypoint direction reverses")
            paths.append((dxl_id, waypoints))
        return paths

    def arm_recorded_path_goal_callback(self, goal_request):
        """명시적인 3축 signed 기록 경로 액션만 수락한다."""
        try:
            self.split_recorded_path_request(goal_request)
        except ValueError as exc:
            self.get_logger().warn(f"Rejecting recorded path: {exc}")
            return GoalResponse.REJECT
        if (self.read_only or not self.integrated_test_mode
                or self.gripper_only_mode
                or self.end_effector_kind != "rotary"
                or self.gripper_ids != [5]
                or int(goal_request.max_abs_current) <= 0
                or float(goal_request.stall_timeout) <= 0.0
                or float(goal_request.step_timeout) <= 0.0
                or not 1 <= int(goal_request.goal_tolerance) <= 10):
            self.get_logger().warn(
                "Rejecting recorded path: mode/preset/safety gate closed")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def arm_goal_callback(self, goal_request):
        if getattr(self, "integrated_test_mode", False):
            self.get_logger().error(
                "Integrated test mode: rejecting normal arm trajectory")
            return GoalResponse.REJECT
        if self.gripper_only_mode:
            self.get_logger().error(
                "Gripper-only mode: rejecting arm FollowJointTrajectory goal")
            return GoalResponse.REJECT
        return self.goal_callback(goal_request)

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Cancel requested")
        self._stop_tool('action cancelled')
        return CancelResponse.ACCEPT

    def gripper_goal_callback(self, goal_request):
        if self.tool_profile.get('backend') != 'gripper':
            self.get_logger().error('gripper goal rejected: selected tool is not a gripper')
            return GoalResponse.REJECT
        if (not self.mock_mode and
                (self.control_mode != 'MANUAL' or not self._tool_backend_ready())):
            self.get_logger().error(
                'gripper goal rejected: MANUAL ownership or tool backend '
                'interlock not ready')
            return GoalResponse.REJECT
        with self._gripper_goal_lock:
            if self._gripper_goal_active:
                self.get_logger().warn(
                    'gripper goal rejected: another gripper goal is active')
                return GoalResponse.REJECT
            self._gripper_goal_active = True
        self.get_logger().info('gripper goal accepted')
        return GoalResponse.ACCEPT

    def gripper_cancel_callback(self, _goal_handle):
        self.get_logger().info('gripper cancel requested')
        return CancelResponse.ACCEPT

    def execute_gripper(self, goal_handle):
        try:
            return self._execute_gripper(goal_handle)
        finally:
            with self._gripper_goal_lock:
                self._gripper_goal_active = False

    def _execute_gripper(self, goal_handle):
        """Map one logical gripper joint to one or two calibrated actuators."""
        result = FollowJointTrajectory.Result()
        trajectory = goal_handle.request.trajectory
        if not trajectory.points or not trajectory.points[-1].positions:
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = 'empty gripper trajectory'
            goal_handle.abort()
            return result
        if self.mock_mode:
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            result.error_string = 'mock gripper action'
            goal_handle.succeed()
            return result
        position = float(trajectory.points[-1].positions[0])
        if self.temporary_jog_enabled:
            target = int(round(position))
            if not (self.temporary_jog_safe_min <= target
                    <= self.temporary_jog_safe_max):
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = (
                    f'temporary jog target {target} outside '
                    f'[{self.temporary_jog_safe_min}, '
                    f'{self.temporary_jog_safe_max}]')
                goal_handle.abort()
                self.get_logger().error(result.error_string)
                return result
            targets = {self.tool_ids[0]: target}
        else:
            targets = None
        open_pos = (1.0 if self.temporary_jog_enabled else
                    float(self.tool_profile.get('open_position', 1.0)))
        close_pos = (0.0 if self.temporary_jog_enabled else
                     float(self.tool_profile.get('close_position', 0.0)))
        denominator = open_pos - close_pos
        if self.temporary_jog_enabled:
            denominator = 1.0
        if denominator == 0.0:
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = 'invalid logical gripper endpoints'
            goal_handle.abort()
            return result
        ratio = max(0.0, min(1.0, (position - close_pos) / denominator))
        endpoints = self.tool_profile.get('motor_endpoints') or {
            self.tool_ids[0]: {
                'open': self.tool_profile['open_tick'],
                'close': self.tool_profile['close_tick']}}
        if self.temporary_jog_enabled:
            low, high = self.temporary_jog_safe_min, self.temporary_jog_safe_max
        else:
            low = int(self.tool_profile['safe_min_tick'])
            high = int(self.tool_profile['safe_max_tick'])
        targets = targets or {}
        try:
            with self._bus_lock:
                for dxl_id in self.tool_ids:
                    if self.temporary_jog_enabled:
                        tick = targets[dxl_id]
                    else:
                        ep = endpoints.get(dxl_id, endpoints.get(str(dxl_id)))
                        tick = int(round(ep['close'] + ratio *
                                         (ep['open'] - ep['close'])))
                    if not low <= tick <= high:
                        raise RuntimeError(
                            f'id {dxl_id} goal {tick} outside [{low},{high}]')
                    comm, error = self.packet_handler.write4ByteTxRx(
                        self.port_handler, dxl_id, ADDR_GOAL_POSITION,
                        tick & 0xffffffff)
                    if comm != 0 or error != 0:
                        raise RuntimeError(f'goal write failed for id {dxl_id}')
                    targets[dxl_id] = tick
            self.get_logger().info(
                f'gripper targets dispatched: normalized={ratio:.6f}, '
                f'targets={targets}')
            deadline = time.monotonic() + float(
                self.tool_profile.get('action_time', 0.0))
            max_effort = float(self.tool_profile.get(
                'max_abs_effort', float('inf')))
            errors = {}
            while time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    self._hold_tool_position()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = 'gripper goal canceled and held'
                    goal_handle.canceled()
                    self.get_logger().info(result.error_string)
                    return result
                with self._bus_lock:
                    for dxl_id in self.tool_ids:
                        position, load = self._read_tool_state(dxl_id)
                        if abs(load) > max_effort:
                            raise RuntimeError(
                                f'id {dxl_id} effort limit exceeded')
                        errors[dxl_id] = targets[dxl_id] - position
                if all(abs(error) <= self.gripper_target_tolerance
                       for error in errors.values()):
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = (
                        f'gripper targets reached: targets={targets}, '
                        f'errors={errors}')
                    goal_handle.succeed()
                    self.get_logger().info(result.error_string)
                    return result
                time.sleep(0.05)
            raise RuntimeError(
                f'gripper target tolerance not reached: targets={targets}, '
                f'errors={errors}, tolerance={self.gripper_target_tolerance}')
        except Exception as exc:
            self._stop_tool(str(exc))
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            result.error_string = str(exc)
            goal_handle.abort()
            return result

    def _read_tool_state(self, dxl_id):
        hw, comm, error = self.packet_handler.read1ByteTxRx(
            self.port_handler, dxl_id, ADDR_HARDWARE_ERROR_STATUS)
        load, load_comm, load_error = self.packet_handler.read2ByteTxRx(
            self.port_handler, dxl_id, ADDR_PRESENT_LOAD)
        position, pos_comm, pos_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, dxl_id, ADDR_PRESENT_POSITION)
        if (comm != 0 or error != 0 or load_comm != 0 or load_error != 0
                or pos_comm != 0 or pos_error != 0 or hw != 0):
            raise RuntimeError(f'fault reading id {dxl_id}')
        return self._tool_position_tick(dxl_id, position), to_signed(load, 2)

    def _hold_tool_position(self):
        with self._bus_lock:
            positions = {
                dxl_id: self._read_tool_state(dxl_id)[0]
                for dxl_id in self.tool_ids}
            for dxl_id, position in positions.items():
                comm, error = self.packet_handler.write4ByteTxRx(
                    self.port_handler, dxl_id, ADDR_GOAL_POSITION,
                    position & 0xffffffff)
                if comm != 0 or error != 0:
                    raise RuntimeError(f'hold write failed for id {dxl_id}')

    # ------------------------------------------------------------------ arm
    def teleop_goal_callback(self, msg):
        if len(msg.data) != 2:
            self.get_logger().warn("Teleop goal must be [motor_id, goal_tick]")
            return

        dxl_id, goal_tick = (int(msg.data[0]), int(msg.data[1]))
        if dxl_id not in ARM_IDS:
            self.get_logger().warn(f"Unknown arm motor ID from teleop: {dxl_id}")
            return
        if (self.gripper_only_mode
                or getattr(self, "integrated_test_mode", False)):
            self.get_logger().error(
                f"Diagnostic mode: rejecting arm teleop command id={dxl_id}")
            return
        if self.read_only:
            self.get_logger().warn("Read-only mode: ignoring teleop goal")
            return
        if dxl_id not in self.active_ids:
            self.get_logger().error(f"Inactive arm motor ID from teleop: {dxl_id}")
            return

        # 다회전(Extended Position) 축을 0~4095 로 clamp 하면 감속기 축이 한 바퀴
        # 넘는 순간 명령이 끝단에 박혀 더는 안 움직인다 — 축별 범위로 clamp 한다.
        if any(c["id"] == dxl_id and c["extended"] for c in JOINT_CONFIG.values()):
            goal_tick = max(DXL_EXTENDED_MIN_TICK, min(DXL_EXTENDED_MAX_TICK, goal_tick))
        else:
            goal_tick = max(DXL_MINIMUM_POSITION_VALUE,
                            min(DXL_MAXIMUM_POSITION_VALUE, goal_tick))
        self.group_sync_write.clearParam()
        if not self.group_sync_write.addParam(
                dxl_id, self.int_to_little_endian_4bytes(goal_tick)):
            self.get_logger().warn(f"Failed to add teleop sync write param: id={dxl_id}")
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

        self.trajectory_callback(trajectory)

        goal_handle.succeed()

        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "Trajectory sent to Dynamixel motors"
        return result

    def trajectory_callback(self, msg):
        if self.control_scope == 'END_EFFECTOR_ONLY':
            self.get_logger().warn(
                'Ignoring arm trajectory: control_scope=END_EFFECTOR_ONLY')
            return
        if (self.read_only or not self.tool_selection
                or not self.tool_selection.valid
                or (not self.mock_mode and not self.tool_motion_allowed)):
            self.get_logger().warn(
                "Ignoring arm trajectory: tool interlock not ready")
            return
        if not msg.points:
            return
        self._warn_if_torque_off()

        point = msg.points[-1]

        if len(msg.joint_names) != len(point.positions):
            self.get_logger().warn("JointTrajectory names/positions length mismatch")
            return

        self.group_sync_write.clearParam()
        added_any = False

        for joint_name, rad in zip(msg.joint_names, point.positions):
            if joint_name not in JOINT_CONFIG:
                self.get_logger().warn(f"Unknown joint from MoveIt: {joint_name}")
                continue

            dxl_id = JOINT_CONFIG[joint_name]["id"]
            if dxl_id not in self.active_ids:
                self.get_logger().error(
                    f"Inactive arm motor from trajectory: {joint_name}, id={dxl_id}"
                )
                continue
            goal_tick = self.rad_to_tick(joint_name, rad)
            param_goal_position = self.int_to_little_endian_4bytes(goal_tick)

            ok = self.group_sync_write.addParam(dxl_id, param_goal_position)
            if not ok:
                self.get_logger().warn(f"Failed to add sync write param: id={dxl_id}")
                continue
            added_any = True

            self.get_logger().info(
                f"{joint_name} -> id {dxl_id}: {rad:.3f} rad -> {goal_tick}"
            )

        if not added_any:
            self.get_logger().error("Trajectory contains no active arm motors")
            self.group_sync_write.clearParam()
            return

        result = self.group_sync_write.txPacket()
        if result != 0:
            self.get_logger().warn(f"GroupSyncWrite failed: result={result}")

        self.group_sync_write.clearParam()

    # ------------------------------------------------------------------ feedback
    def publish_joint_states(self):
        if self.mock_mode:
            self.joint_state_pub.publish(JointState())
            self.fault_pub.publish(Bool(data=False))
            return
        if not self.port_connected:
            self.joint_state_pub.publish(JointState())
            self.fault_pub.publish(Bool(data=True))
            return
        with self._bus_lock:
            self.group_sync_read.txRxPacket()
        # 일부 ID가 버스에 없어도 응답받은 ID만 처리 (result 무시)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        # controller fault 집계 — SyncRead 에 등록된(토크 ON 성공) ID 중 하나라도
        # Hardware Error Status != 0 이거나 이번 tick 응답이 없으면 fault=True.
        # 응답 없음도 fault 로 보는 이유: 활성 등록된 서보가 갑자기 무응답이면 버스/전원
        # 이상일 수 있어 "정상"으로 오인하면 안 됨(안전 측 기본값).
        fault = False if self.gripper_only_mode else not ARM_IDS.issubset(
            self.active_ids)

        # 팔 관절: position(rad) + address-126 feedback(raw signed).
        # 주소 126의 의미는 실제 장착 모터 control table로 확인해야 한다.
        for joint_name, config in JOINT_CONFIG.items():
            dxl_id = config["id"]
            if dxl_id not in self.active_ids:
                continue
            sample = self._read_sample(dxl_id)
            if sample is None:
                fault = True
                continue
            feedback_raw, tick, hw_error = sample
            if hw_error != 0:
                fault = True
            msg.name.append(joint_name)
            msg.position.append(self.tick_to_rad(joint_name, tick))
            msg.effort.append(float(feedback_raw))

        if (self.cleaning_configured
                and self.cleaning_actuator_id in self.active_ids):
            sample = self._read_sample(self.cleaning_actuator_id)
            if sample is None:
                fault = True
                self._tool_samples[self.cleaning_actuator_id] = {
                    'id': self.cleaning_actuator_id,
                    'joint': self.cleaning_actuator_joint, 'position': None,
                    'effort': None, 'online': False}
            else:
                load_raw, tick, hw_error = sample
                tick = self._tool_position_tick(dxl_id, tick)
                fault = fault or hw_error != 0
                self._tool_samples[self.cleaning_actuator_id] = {
                    'id': self.cleaning_actuator_id,
                    'joint': self.cleaning_actuator_joint,
                    'position': int(tick), 'effort': float(abs(load_raw)),
                    'online': hw_error == 0}
                msg.name.append(self.cleaning_actuator_joint)
                msg.position.append(float(to_signed(tick, LEN_PRESENT_POSITION)))
                msg.effort.append(float(load_raw))

        if self.tool_profile.get('backend') == 'gripper':
            joint_names = self.tool_profile.get('joint_names', [])
            loads = []
            positions = []
            for dxl_id in self.tool_ids:
                if dxl_id not in self.active_ids:
                    fault = True
                    self._tool_samples[dxl_id] = {
                        'id': dxl_id, 'joint': '', 'position': None,
                        'effort': None, 'online': False}
                    continue
                sample = self._read_sample(dxl_id)
                if sample is None:
                    fault = True
                    self._tool_samples[dxl_id] = {
                        'id': dxl_id, 'joint': '', 'position': None,
                        'effort': None, 'online': False}
                    continue
                load_raw, tick, hw_error = sample
                fault = fault or hw_error != 0
                loads.append(abs(load_raw))
                tick = self._tool_position_tick(dxl_id, tick)
                positions.append(tick)
                self._tool_samples[dxl_id] = {
                    'id': dxl_id,
                    'joint': (joint_names[0] if joint_names else ''),
                    'position': int(tick), 'effort': float(abs(load_raw)),
                    'online': hw_error == 0}
            if joint_names and loads:
                msg.name.append(joint_names[0])
                msg.position.append(float(positions[0]))
                msg.effort.append(float(max(loads)))

        self.joint_state_pub.publish(msg)
        self.fault_pub.publish(Bool(data=fault))

    def _read_sample(self, dxl_id):
        """SyncRead 블록에서 (signed address-126 feedback, position, hw error) 추출.

        PRESENT_VELOCITY(128,4)는 SyncRead 범위(70~135) 안에 이미 포함돼 있어 별도 버스
        요청 없이 같은 블록에서 꺼낸다. 미수신 시 None.
        """
        with self._bus_lock:
            if not self.group_sync_read.isAvailable(
                    dxl_id, ADDR_HARDWARE_ERROR_STATUS,
                    LEN_HARDWARE_ERROR_STATUS):
                return None
            if not self.group_sync_read.isAvailable(
                    dxl_id, ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD):
                return None
            if not self.group_sync_read.isAvailable(
                    dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
                return None
            hw_error = self.group_sync_read.getData(
                dxl_id, ADDR_HARDWARE_ERROR_STATUS,
                LEN_HARDWARE_ERROR_STATUS)
            feedback_raw = to_signed(
                self.group_sync_read.getData(
                    dxl_id, ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD),
                LEN_PRESENT_LOAD,
            )
            tick = self.group_sync_read.getData(
                dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        return feedback_raw, tick, hw_error

    def destroy_node(self):
        if not self.read_only and not self.mock_mode:
            self._stop_tool('node shutdown')
            if self.cleaning_configured:
                self.packet_handler.write4ByteTxRx(
                    self.port_handler, self.cleaning_actuator_id, ADDR_GOAL_VELOCITY, 0)
                self.packet_handler.write1ByteTxRx(
                    self.port_handler, self.cleaning_actuator_id,
                    ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            if self.control_scope == 'FULL_ROBOT':
                for config in JOINT_CONFIG.values():
                    self.packet_handler.write1ByteTxRx(
                        self.port_handler, config["id"],
                        ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if not self.mock_mode and self.port_connected:
            self.port_handler.closePort()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MoveItDynamixelBridge()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    # position_node 와 같은 이유로 traceback 대신 한 줄로 죽는다(그쪽 main 참고).
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
