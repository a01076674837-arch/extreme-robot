"""로봇팔 미션 FSM 노드 (Phase 3, 구간2 구호물자 운반 중심).

설계 문서: `project_docs/PHASE3_FSM_설계.md` §4 상태표 / §5 핸드셰이크.
구현 방식(결정 '가', 2026-06-29): **MoveIt 단일 경로**.
  - 팔 모션: MoveIt `move_action`(MoveGroup)에 목표 pose 전송 → IK·경로계획은
    MoveIt이 수행 → MoveIt이 `arm_controller` FollowJointTrajectory로 실행 →
    upstream `moveit_dynamixel_bridge`가 실제 다이나믹셀 구동.
  - 털털이: `/cleaning/enable`로 start/stop 의도를 bridge에 전달하고, bridge가 별도
    Dynamixel velocity mode actuator를 구동한다. MoveIt planning joint에서는 제외한다.
  - 접촉 피드백: `/joint_states` effort의 cleaning_actuator_joint 값을 사용한다.

  ⚠️ 전제(브릿지 측 선행 작업, PHASE3_FSM_설계.md §6 → 결정 '가'로 이관):
    1) `moveit_dynamixel_bridge`가 `/joint_states`에 **effort(전류)** 를 채워야 함
       (현재는 position만 발행) — 안 그러면 파지/DROP 판정 불가.
    2) 털털이 ID/방향/속도와 contact effort 임계값을 실기에서 확인해야 함.
    3) 카메라 frame → planning frame(base_link) **TF**가 있어야 MoveIt이 목표를 변환.

2026-07-13 파워트레인 "계약 v2"(Notion `2026 국방로봇 자율주행 SW 전체 개발계획` §5.1/5.2)
반영 — status/mode 문자열과 트리거 조건을 아래처럼 갱신:
  - 작업 개시/하역 모두 **`/chassis_mode == MISSION_STOP` AND 같은 mission_id의
    `/arrival_status`** 를 순서 무관하게 둘 다 받아야 전이(`_try_advance`). 미인식
    mode·stale/미래/역행 stamp는 default-deny(상태 변경 없음).
  - `DONE`은 더 이상 완료 권위가 아님 — 픽업 완료는 `CARRYING_LOCKED`, 하역 완료는
    `STOWED_LOCKED`가 최종 권위. `RELEASE` 뒤 `STOWING` 경유해서 `STOWED_LOCKED` 도달.
  - `CARRY` 중 DROP 감지 시 기존 자동 재파지 루프(REGRASP↔PERCEIVE) 대신 `GRIP_LOST`로
    **완전 래치**(자동 재시도 없음) — 새 MISSION_STOP+ArrivalStatus(ARRIVED_PICKUP,
    같은 mission_id 재발행 가능) conjunction이 다시 와야만 PERCEIVE 재진입.
  - `STOW_REQUEST` mode 수신 시 진행 중인 작업을 강제로 RELEASE→STOWING 경로로 유도.
  - mission_id 멱등성: 이미 `STOWED_LOCKED`까지 완료한 mission_id의 ArrivalStatus
    재수신은 무시(중복 재실행 방지).

2026-07-14 자체 결정(우리 쪽에서 결정 가능한 항목은 회의 전에 확정·구현):
  - **`STOW_REQUEST` 범위 확장**: `GRIP_LOST` 전용이었던 것을 `STOW_ABORTABLE_STATES`
    (진행 중인 모든 작업 상태 + `CARRY` + `LOCKED`)로 확장.
  - **`WORK_READY` vs `STOWED_LOCKED` 역할 확정**: `WORK_READY`=MISSION_STOP+ArrivalStatus
    conjunction 수락 순간의 1회성 ack, `STOWED_LOCKED`=그 외 평상시(빈손) 상시 하트비트.
  - **`_is_settled()`**: locked 하트비트(`CARRYING_LOCKED`/`STOWED_LOCKED`) 발행 전 실제
    확인 — TF(`base_frame`←`tip_link`) tip 위치가 `locked_pos_tol` 이내로 안정되고,
    관절각 유한차분 속도가 `locked_vel_tol` 이내인 상태가 `locked_dwell`초 이상
    지속돼야 True. 브릿지가 `/joint_states`에 velocity를 안 실어도 위치 유한차분으로
    자체 계산하므로 문제 없음.

2026-07-15 — origin/main 재합류 + 실제 STOWING 모션 구현:
  - PR #17("파워트레인 DDS 통신 복구 + arm_status 10Hz heartbeat")이 이 파일을 이
    브랜치와 무관하게 독립적으로 다시 손대(계약 v2 상태/게이트 로직 없는 이전 버전 위에
    `contract.py`/`qos_profiles.py` 단일 출처 + heartbeat 전용 타이머·MultiThreadedExecutor
    를 추가) `main`에 먼저 병합됨. 이 세션에서 그 인프라(heartbeat 아키텍처·QoS·contract
    상수 단일 출처) 위에 위 계약 v2 FSM 로직(conjunction 게이트·GRIP_LOST 래치·
    STOW_ABORTABLE_STATES·`_is_settled()`)을 재적용.
  - **LOCK_MODES를 `contract.py` 것으로 통일**(기존엔 이 파일이 로컬로 `DRIVING`을 제외한
    부분집합을 따로 들고 있었음) — `contract.py`(파워트레인 contract.py와 짝, 단일 출처)는
    `DRIVING`도 LOCK_MODES에 포함한다. 즉 PERCEIVE~LIFT 중 `DRIVING` 수신 시에도 이제
    `_enter_locked()`가 걸린다("MISSION_STOP만 허가, 나머지 전부 잠금"을 문자 그대로 적용).
    LOCKED 상태에서 `DRIVING`으로 자동 언락되는 옛 버그(PR #17이 미수정으로 지적)는
    애초에 이 파일에 그런 분기가 없으므로 해당 없음 — `_try_advance()`의
    MISSION_STOP+ArrivalStatus conjunction으로만 탈출.
  - **`STOWING` 실제 접이 모션 구현**(`_begin_stow_move`) — 이전까지는 스켈레톤이라 현재
    자세 그대로 `_is_settled()`만 확인했음(모션 자체가 없어 접힘 자세 검증이 아니라
    "멈춰있나" 검증에 불과했음). 이제 `stow_joint_positions` 파라미터가 정의하는 목표
    관절각으로 `/arm_controller/joint_trajectory`에 직접 궤적을 발행 → 완료 후 `_is_settled()`
    게이트를 거쳐 `STOWED_LOCKED`.
    `stow_joint_positions` 기본값은 2026-07-29 팀 결정으로 **all-zero**다(주행 안정성 —
    도달 가능한 자세 중 CG가 가장 낮음). 아래 파라미터 선언부의 근거 주석 참고.
    ⚠️ **파워트레인 문서 §6의 all-zero home 금지와 정면 충돌한다 — 양 팀 합의 전까지
    실차 연동 금지.** 또한 URDF 상으로만 검증됐고 실기 검증은 아직이다.

2026-07-15 — 파워트레인 §5.1 잔여 합의 2건 해결(`project_docs/파워트레인_계약_충돌점검.md`
항목 1·2 대응):
  - **`_near_stow_posture()` 추가** — `LOCKED`(지형/주행 이벤트로 작업 중단) 경유로 도달한
    임의 자세를 예전엔 정지만 확인되면 바로 `STOWED_LOCKED`로 근사했음. 이제 관절각이
    `stow_joint_positions` 근처(`stow_pos_tol_rad`)인지 확인한 경우에만 `STOWED_LOCKED`를
    발행하고, 아니면 `EXECUTING`을 유지해 파워트레인 쪽 motion hold를 받는다(거짓 주행
    허가 방지).
  - **`LOWER_RELEASE` 상태 신설** — `PAYLOAD_ALOFT_STATES`(`LIFT`/`CARRY`, 화물을 든 채
    공중일 수 있는 상태)에서 `STOW_REQUEST`로 중단되면, 예전엔 바로 `RELEASE`(그리퍼
    오픈)로 갔는데 이는 화물이 공중에서 그대로 낙하하는 경로였음. 이제 `RELEASE` 전에
    `lift_height`만큼 먼저 내려(`_lower_pose`/`_lower_target_xyz`, `_carry_pose`/
    `_lift_target_xyz`의 역) grasp 당시 높이 근처로 되돌아간 뒤에만 그리퍼를 연다.
  - **controller fault 게이트 추가** — `moveit_dynamixel_bridge`가 Hardware Error
    Status(주소 70)를 기존 current/position SyncRead 블록에 합쳐 읽어
    `/dynamixel/controller_fault`(Bool, 내부용 — DDS 경계 안 넘음)로 발행하도록
    확장. `_is_settled()`가 이 값이 True(등록된 서보 중 하나라도 에러 또는 무응답)
    이면 즉시 미확인 처리 — 이전엔 이 필드 자체가 없어 검사 불가였던 항목(§5.1
    잔여 항목 3).

상태 흐름: IDLE → PERCEIVE → PLAN → DESCEND → CLEAN_START → CONTACT_CHECK
  → CLEAN → CLEAN_STOP → LIFT → LOCK_CHECK → CARRY → RELEASE → DONE
  → STOWING → STOWED_LOCKED → IDLE
  CARRY 중 DROP 감지 → GRIP_LOST(래치) → (재발행 conjunction) PERCEIVE
  PERCEIVE~LIFT 중 지형/주행 이벤트 → LOCKED(래치, 하트비트 유지) → (재발행 conjunction) PERCEIVE
  LIFT/CARRY(또는 LIFT 중 LOCKED) 중 STOW_REQUEST → LOWER_RELEASE → RELEASE → STOWING
  그 외 상태 중 STOW_REQUEST → RELEASE → STOWING (이미 grasp 높이 근처라 낙하 낙차 없음)
  LOCKED/GRIP_LOST 모두: 진행 중 모션 취소 + 현재 자세 홀드, MISSION_STOP conjunction으로만 탈출.

⚠️ 스켈레톤: MoveGroup pose goal / 그리퍼 액션 / effort 판정 / FSM 골격은 구현.
   LIFT·CARRY 목표 pose, 임계값 캘리브, TF 연결은 구현됨. STOWING 모션은 위 참고
   (목표 관절각 실측 필요). CARRYING_LOCKED/STOWED_LOCKED 발행 전 controller fault
   확인은 `moveit_dynamixel_bridge`의 `/dynamixel/controller_fault`(Hardware Error
   Status 집계)를 `_is_settled()`에서 게이트로 사용해 구현 완료(2026-07-15).
"""
from copy import deepcopy
from enum import Enum, auto
import random

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration as RclpyDuration
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose

from builtin_interfaces.msg import Duration
from std_msgs.msg import Bool, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.action import MoveGroup
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import (MotionPlanRequest, Constraints, PositionConstraint,
                             OrientationConstraint, BoundingVolume, RobotState)
from moveit_msgs.srv import GetPositionFK
from shape_msgs.msg import SolidPrimitive
from robot_arm_msgs.msg import (ArrivalStatus, ChassisMode, ArmStatus,
                                DetectedObject, TaskCommand, TaskResult)
from dynamixel_control.moveit_dynamixel_bridge import JOINT_CONFIG
from dynamixel_control.gripper_presets import DEFAULT_GRIPPER, get_preset


# 2026-07-15 Isaac Sim 기반 재export(robotarm_urdf_20260711.urdf) 기준 — URDF 자체는
# 팔 5축(arm_joint_1~5)을 전부 반영하지만, analytic IK(FK+수치 자코비안)는 아직 앞의
# 3관절만 풀도록 남겨둠(HW-7 당시 6DOF pose goal이 NO_IK_SOLUTION이던 문제 회피용으로
# 도입된 3DOF 위치전용 IK — URDF가 3축만 있어서가 아니라 solver를 아직 5DOF로 확장 안
# 해서임, 방향은 여전히 무시). MoveGroup 경로(§6 결정 '가')는 남겨두되 ik_mode:='moveit'로
# 전환 가능하게만 유지.
# 팔 관절 이름·순서는 **moveit_dynamixel_bridge.JOINT_CONFIG 가 단일 출처**다.
# 여기에 리터럴로 다시 적지 말 것 — 2026-08-07 이전에는 `['arm_joint_1','arm_joint_2',
# 'arm_joint_3']` 로 하드코딩돼 있었고, 브릿지가 실기 버스(arm_joint_2~5)에 맞춰
# 갱신된 뒤에도 그대로 남아 **모터가 없는 arm_joint_1 을 명령하고 실제로 달려 있는
# arm_joint_4/5 는 건드리지도 않는** 상태였다. dict 는 삽입 순서를 보존하므로
# 궤적 메시지의 관절 순서도 브릿지와 자동으로 일치한다.
ARM_JOINT_NAMES = list(JOINT_CONFIG)


# ──────────────────────────────────────────────
# status / mode 문자열 — 단일 출처는 contract.py (파워트레인 contract.py 와 짝).
# 여기서 상수를 새로 정의하지 말 것. 어휘 변경은 양 팀 합의 사항이다.
# ──────────────────────────────────────────────
from dynamixel_control.contract import (       # noqa: E402
    ARRIVED_PICKUP, ARRIVED_DROP,
    ARM_PERCEIVING, ARM_PLANNING, ARM_EXECUTING, ARM_DONE, ARM_FAILED,
    ARM_WORK_READY, ARM_STOWING, ARM_STOWED_LOCKED, ARM_CARRYING_LOCKED, ARM_GRIP_LOST,
    LOCK_MODES, MODE_MISSION_STOP, MODE_STOW_REQUEST, HEARTBEAT_RATE_HZ,
)
from dynamixel_control.qos_profiles import HEARTBEAT_QOS, ARRIVAL_QOS   # noqa: E402
from dynamixel_control.sensor_manager import SensorManager              # noqa: E402
from dynamixel_control.tool_manager import (                            # noqa: E402
    ParameterToolIdentityProvider, ToolManager)
from dynamixel_control.tool_profiles import (                           # noqa: E402
    load_profiles, ToolProfileError)
from ament_index_python.packages import get_package_share_directory     # noqa: E402
from pathlib import Path                                                # noqa: E402
import json                                                             # noqa: E402

# contract.py의 LOCK_MODES는 DRIVING을 포함한다("MISSION_STOP만 허가, 나머지 전부 잠금").
RECOGNIZED_MODES = LOCK_MODES | {MODE_MISSION_STOP, MODE_STOW_REQUEST}

# stamp freshness — 미래/역행 판정 허용오차 [s] (계약 §5.1 age 0~0.5s 기준)
STAMP_FUTURE_TOL = 0.5

# moveit_msgs/MoveItErrorCodes.SUCCESS
MOVEIT_SUCCESS = 1
MISSION_PICK_PLACE = "PICK_PLACE"
MISSION_ROTARY_TOOL = "ROTARY_TOOL"


class State(Enum):
    IDLE = auto()
    PERCEIVE = auto()
    PLAN = auto()
    APPROACH = auto()
    TOOL_ACTION = auto()
    GRASP = auto()
    GRASP_CHECK = auto()
    CLEAN_START = auto()
    CONTACT_CHECK = auto()
    CLEAN = auto()
    CLEAN_STOP = auto()
    RETRACT = auto()
    LOCK_CHECK = auto()
    CARRY = auto()
    RELEASE = auto()
    ARM_TEST_MOVE = auto()
    RANDOM_ARM_DEMO = auto()
    END_EFFECTOR_ROTATE = auto()
    DONE = auto()
    FAILED = auto()
    # 기존 파워트레인 계약에서 유지하는 감독/안전 상태.
    GRIP_LOST = auto()
    LOWER_RELEASE = auto()
    STOWING = auto()
    STOWED_LOCKED = auto()
    LOCKED = auto()


# 지형/주행 이벤트(LOCK_MODES)로 preempt 대상이 되는 상태 — 실제 모션/그리퍼 동작이 진행
# 중일 수 있는 상태만. CARRY는 이미 정지-유지 상태(그리퍼 effort 감시만)라 preempt
# 불필요 — 계약 v2 하트비트를 CARRY 자체 루프가 계속 발행해야 하므로 굳이 LOCKED로 빼지 않음.
PREEMPTIBLE_STATES = (
    State.PERCEIVE, State.PLAN, State.APPROACH, State.TOOL_ACTION, State.CLEAN_START,
    State.CONTACT_CHECK, State.CLEAN, State.CLEAN_STOP, State.RETRACT,
    State.LOCK_CHECK,
)

# STOW_REQUEST(운영자 포기·재정렬 유도)로 즉시 RELEASE(or LOWER_RELEASE)→STOWING 강제
# 진입 가능한 상태 — 작업이 진행 중이거나 래치된 모든 상태(2026-07-14 결정: GRIP_LOST
# 전용이었던 것을 확장). IDLE/LOWER_RELEASE/RELEASE/STOWING/STOWED_LOCKED는 이미
# 정지/포기 진행 중이라 대상에서 제외.
STOW_ABORTABLE_STATES = PREEMPTIBLE_STATES + (
    State.CARRY, State.FAILED, State.GRIP_LOST, State.LOCKED,
)

# STOW_REQUEST로 중단될 때 화물을 든 채 공중에 있을 수 있는 상태 — 그리퍼를 바로 열면
# 낙하 위험(파워트레인 §5.1 잔여 합의 ①). 이 상태들에서만 RELEASE 전에 파지 높이까지
# 먼저 내리는 LOWER_RELEASE를 경유한다. 그 외(PERCEIVE/PLAN/DESCEND/청소 상태)는 이미
# grasp 높이 근처라 낙하 낙차가 없어 바로 RELEASE해도 안전.
PAYLOAD_ALOFT_STATES = (State.RETRACT, State.CARRY)


class ArmFsmNode(Node):
    def __init__(self):
        super().__init__('arm_fsm_node')

        # ── 파라미터 ──────────────────────────────
        # 형상과 tip_link를 포함한 모든 엔드이펙터 기본값을 같은 preset에서 고른다.
        self.declare_parameter('end_effector_preset', DEFAULT_GRIPPER)
        gripper_type = self.get_parameter('end_effector_preset').value
        gpreset = get_preset(gripper_type, self.get_logger())

        # MoveIt
        self.declare_parameter('planning_group', 'arm')          # SRDF group
        # tip_link: arm_joint_5 이후 고정 조인트 체인의 마지막 링크(link_051).
        # 털털이 cleaner_base_link는 이 링크 아래 wrist_to_cleaner fixed joint로 부착된다.
        # 2026-07-15 Isaac Sim 재export(robotarm_urdf_20260711.urdf) 기준.
        self.declare_parameter('tip_link', 'link_051')            # 그리퍼 부모 링크
        self.declare_parameter('base_frame', 'base_link')        # planning frame (리프트 기준)
        self.declare_parameter('lift_height', 0.10)              # LIFT 시 base_link +Z [m]
        self.declare_parameter('approach_height', 0.08)          # target 위 접근 오프셋 [m]
        self.declare_parameter('pick_frame_id', 'camera_color_optical_frame')
        self.declare_parameter('pos_tolerance', 0.01)            # [m]
        self.declare_parameter('orient_tolerance', 0.1)          # [rad]
        self.declare_parameter('planning_time', 5.0)
        self.declare_parameter('vel_scale', 0.1)                 # 저속(파지 안전)
        self.declare_parameter('acc_scale', 0.1)
        # 'analytic'(기본, URDF 3관절 한정 수치 IK) | 'moveit'(URDF 5축 완성 후 전환)
        self.declare_parameter('ik_mode', 'analytic')
        self.declare_parameter('ik_max_iters', 8)
        self.declare_parameter('ik_tol', 0.01)          # [m] 위치 수렴 허용오차
        self.declare_parameter('ik_accept_tol', 0.03)   # [m] 최종 실패 판정 기준
        # 재시도(FAILED→PERCEIVE→PLAN) 때 타겟을 다시 인식하지 않고 처음 얼린 값을 재사용한다.
        #
        # ⚠️ 왜 필요한가 (2026-08-09 실기): APPROACH/DESCEND 로 팔이 움직이면 **팔 자신이
        #    카메라 시야에 들어온다.** 그 상태에서 재시도가 PERCEIVE 를 다시 돌면 YOLO 가
        #    팔/그리퍼를 박스로 잡거나 진짜 박스가 가려져서 타겟이 통째로 튄다 — 실측으로
        #    base_link (0.348,-0.053,0.056) → (0.603,0.481,0.067) 로 53cm 점프했고, 그
        #    뒤로는 IK 실패만 반복하는 루프에 빠졌다.
        #
        # 기본값 False(=매번 다시 인식)는 기존 동작이다. 현장에서는 박스가 실제로 움직였을
        # 수 있어 다시 보는 게 맞을 때도 있기 때문에 옵트인으로 둔다.
        #
        # 🔧 근본 해결은 따로다: 재인식 전에 팔을 시야 밖(관측 자세)으로 물린 뒤 보는 것.
        #    그게 들어가기 전까지는 이 플래그가 실용적인 우회다.
        self.declare_parameter('freeze_target_on_retry', False)
        self.declare_parameter('gripper_change_mode', False)
        self.declare_parameter('gripper_disabled', False)
        self.declare_parameter(
            'gripper_command_calibrated', gpreset['command_calibrated'])
        self.declare_parameter('stop_after_descend', False)
        self.declare_parameter('arm_move_speed', 0.5)   # [rad/s] 직접명령 시 소요시간 추정용
        # 털털이는 MoveIt planning joint가 아닌 별도 velocity actuator다. ZIP에 실제
        # joint/모터 정보가 없으므로 actuator joint 이름은 빈 값(안전 비활성)이 기본이다.
        self.declare_parameter('cleaning_actuator_joint', '')
        self.declare_parameter('vla_command_topic', '/vla/command')
        self.declare_parameter('vla_result_topic', '/vla/result')
        self.declare_parameter('vla_standalone_mode', False)
        self.declare_parameter('dry_run_mode', False)
        self.declare_parameter('validation_mode', False)
        self.declare_parameter('tool_type', 'spur_1motor_gripper')
        default_profiles = str(Path(get_package_share_directory(
            'dynamixel_control')) / 'config' / 'tool_profiles.yaml')
        self.declare_parameter('tool_profile_file', default_profiles)
        self.declare_parameter('tool_status_timeout', 1.5)
        self.declare_parameter('cleaning_start_time', 0.5)
        self.declare_parameter('contact_timeout', 3.0)
        self.declare_parameter('lock_check_timeout', 3.0)
        self.declare_parameter('clean_duration', 5.0)
        self.declare_parameter('tick_rate', 10.0)
        # chassis_mode 수신 끊김 워치독 — §5.1 "수신 끊김 = default-deny(잠금 유지)"
        self.declare_parameter('chassis_mode_timeout', 1.0)      # [s]
        # locked 하트비트(CARRYING_LOCKED/STOWED_LOCKED) 발행 전 실제 확인 조건 — §5.1
        # "문자열만 바꾸는 게 아니라 자세 오차·관절 속도·유지 시간을 확인한 뒤 발행".
        # 수치는 실측 캘리브 전 placeholder(TODO). 컨트롤러 fault 확인은 브릿지에 아직
        # 해당 필드가 없어 미포함(별도 후속 작업).
        self.declare_parameter('locked_pos_tol', 0.005)   # [m] tip 위치 흔들림 허용치
        self.declare_parameter('locked_vel_tol', 0.05)    # [rad/s] 관절 속도(유한차분) 허용치
        self.declare_parameter('locked_dwell', 0.5)       # [s] 안정 유지 시간
        # STOWING 목표 관절각(ARM_JOINT_NAMES 순서 — 2026-08-19 기준 5축:
        # arm_joint_1..5. ID 11 서보 실장으로 4축에서 5축이 됐다).
        # **팀이 주행 안정성 기준으로 all-zero 를 접힘 자세로 확정**(사용자 지시, 2026-07-29).
        # 2026-07-29 랙피니언 그리퍼 URDF 실측 지표:
        #   점유 bbox x 224mm × y 606mm   높이 285mm   최저점 +50mm
        #   CG 높이 160mm (도달 가능한 자세 중 최저)   j2+j3 중력토크 13.13N·m   자기충돌 0
        # 근거: j2·j3 하한이 0이라 팔은 [수평 → 수직 → 반대쪽 수평]만 훑고 아래로는 못 내려간다.
        # 그래서 도달 가능한 자세 중 CG 가 가장 낮은 것이 all-zero 이고, 경사·요철에서 전복
        # 여유가 가장 크다. (대안이던 '수직 마스트' [0, 1.40, 2.85] 는 bbox 213×235mm 로
        # 발자국은 훨씬 작고 중력토크도 0.79N·m 로 20배 낮지만, 높이 978mm·CG 400mm 라
        # 전복 여유가 나쁘다. 차체가 짧아 y 606mm 가 안 들어가면 그쪽으로 되돌릴 것.)
        #
        # ⚠️ **파워트레인 계약 위반 상태다 — 양 팀 합의 전까지 실차 연동 금지.**
        # 파워트레인 문서 §6 "all-zero home 과 direct dynamixel goal publisher 는 production
        # 에서 금지한다" (project_docs/파워트레인_계약_충돌점검.md:110).
        # 실질 위험: `_near_stow_posture()` 가 무력화된다 — 전원만 들어오고 초기화 안 된 팔도
        # 관절각이 ~0 이라 이 검사를 그냥 통과해서, 실제로 접히지 않았는데 STOWED_LOCKED 를
        # 발행 → 파워트레인이 주행 허가로 받는다. 금지 조항의 이유가 정확히 이것이다.
        # 회피책: 물리적으로 거의 같으면서 0 과 구분되는 값(예: [0.0, 0.15, 0.15] — bbox
        # 224×606mm 로 all-zero 와 동일, 높이 328mm, 토크 12.61N·m)을 쓰면 stow_pos_tol_rad
        # (0.1) 밖이라 위 검사가 되살아난다. 주행 안정성은 사실상 그대로다.
        #
        # 이전 기본값 [0.0, -0.6, 1.2]은 j2=-0.6이 URDF 하한(0) 밖이라 애초에 도달 불가였다.
        # ⚠️ URDF 검증만 끝났고 실기 검증은 아직 — 실물 구동 전 서보 tick 대응 확인 필요.
        # 길이는 ARM_JOINT_NAMES 를 따라간다 — 리터럴로 박아두면 축 수가 바뀔 때
        # 길이 검증(아래)에 걸려 STOWING 모션이 **조용히 비활성**된다.
        # 파지 후 **물건을 문 채** 접힘(=URDF home) 자세로 돌아갈지. 기본 false 는
        # 기존 동작(들어올린 자리에서 CARRY 대기)이다. true 면 LIFT 완료 후
        # stow_joint_positions 로 접고 나서 CARRY 로 들어간다 — 화물을 든 채
        # 주행하려면 팔이 펴져 있는 것보다 접혀 있는 쪽이 안전하다는 판단.
        # ⚠️ 접는 경로에 충돌 검사가 없다(_begin_stow_move 는 known-safe 가정으로
        #    직접 궤적을 쏜다). 화물이 큰 경우 차체·자기 링크와 부딪히는지 눈으로
        #    확인하고 켤 것.
        self.declare_parameter('carry_home', False)
        self.declare_parameter('stow_joint_positions', [0.0] * len(ARM_JOINT_NAMES))
        # STOWED_LOCKED 발행 전 "실제로 접힌 자세인지" 확인용 관절각 허용오차 — §5.1 잔여
        # 합의 ②(정지 안정성만 검사하고 접힘 자세 근접은 미확인) 대응. LOCKED 경유(지형/주행
        # 이벤트로 작업 중단)로 도달한 임의 자세를 STOWED_LOCKED로 착칭하지 않기 위함.
        self.declare_parameter('stow_pos_tol_rad', 0.1)   # [rad] ≈5.7도, placeholder

        g = self.get_parameter
        self.planning_group = g('planning_group').value
        self.tip_link = g('tip_link').value
        self.base_frame = g('base_frame').value
        self.lift_height = g('lift_height').value
        self.approach_height = g('approach_height').value
        self.pick_frame_id = g('pick_frame_id').value
        self.pos_tol = g('pos_tolerance').value
        self.orient_tol = g('orient_tolerance').value
        self.planning_time = g('planning_time').value
        self.vel_scale = g('vel_scale').value
        self.acc_scale = g('acc_scale').value
        self.ik_mode = g('ik_mode').value
        self.ik_max_iters = int(g('ik_max_iters').value)
        self.ik_tol = g('ik_tol').value
        self.ik_accept_tol = g('ik_accept_tol').value
        self.freeze_target_on_retry = bool(g('freeze_target_on_retry').value)
        self.gripper_change_mode = bool(g('gripper_change_mode').value)
        self.gripper_command_calibrated = bool(
            g('gripper_command_calibrated').value)
        self.validation_mode = bool(g('validation_mode').value)
        # An uncalibrated preset must not even emit a gripper action goal.  The
        # bridge has the same independent guard, so ID5 remains blocked if a
        # caller bypasses this FSM.
        self.gripper_disabled = (
            self.gripper_change_mode
            or bool(g('gripper_disabled').value)
            or (not self.gripper_command_calibrated
                and not self.validation_mode))
        self.stop_after_descend = (
            self.gripper_change_mode or bool(g('stop_after_descend').value))
        self.arm_move_speed = g('arm_move_speed').value
        self.cleaning_actuator_joint = g('cleaning_actuator_joint').value
        self.vla_standalone_mode = bool(g('vla_standalone_mode').value)
        self.dry_run_mode = bool(g('dry_run_mode').value)
        self.selected_tool_type = str(g('tool_type').value)
        self.tool_status_timeout = float(g('tool_status_timeout').value)
        try:
            profiles = load_profiles(g('tool_profile_file').value)
            self.tool_manager = ToolManager(
                profiles,
                ParameterToolIdentityProvider(self.selected_tool_type),
                mock_mode=self.dry_run_mode)
            self.tool_selection = self.tool_manager.refresh('IDLE')
        except ToolProfileError as exc:
            self.get_logger().error(f'tool profile rejected: {exc}')
            self.tool_selection = None
        self.tool_profile = (
            self.tool_selection.profile if self.tool_selection else {})
        self.tool_profile_valid = bool(
            self.tool_selection and self.tool_selection.valid)
        self.cleaning_start_time = g('cleaning_start_time').value
        self.contact_timeout = g('contact_timeout').value
        self.lock_check_timeout = g('lock_check_timeout').value
        self.clean_duration = g('clean_duration').value
        self.chassis_mode_timeout = g('chassis_mode_timeout').value
        self.locked_pos_tol = g('locked_pos_tol').value
        self.locked_vel_tol = g('locked_vel_tol').value
        self.locked_dwell = g('locked_dwell').value
        self.stow_pos_tol_rad = g('stow_pos_tol_rad').value
        self.carry_home = bool(g('carry_home').value)
        self.stow_joint_positions = list(g('stow_joint_positions').value)
        if len(self.stow_joint_positions) != len(ARM_JOINT_NAMES):
            self.get_logger().error(
                f'stow_joint_positions 길이({len(self.stow_joint_positions)})가 '
                f'ARM_JOINT_NAMES({len(ARM_JOINT_NAMES)})와 다름 — STOWING 모션 비활성')
            self.stow_joint_positions = None
        else:
            self.get_logger().warn(
                f'stow_joint_positions={self.stow_joint_positions} — URDF 상으로만 검증된 '
                '값이다(실기 미검증). 실물 구동 전 서보 tick 대응을 확인할 것.')

        # 모든 개별 센서 topic과 threshold는 SensorManager가 소유한다. FSM은 아래
        # backend-independent 판정 메서드만 사용한다.
        self.sensors = SensorManager(self)

        # ── 토픽/액션 I/O ─────────────────────────
        # QoS 는 계약(contract.py/qos_profiles.py) 기준. heartbeat 계열을 depth 10 으로
        # 두면 낡은 샘플이 큐에 쌓여 파워트레인의 age(신선도) 판정이 어긋난다.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(DetectedObject, '/pick_target', self._on_pick_target, latched)
        self.create_subscription(ArrivalStatus, '/arrival_status', self._on_arrival, ARRIVAL_QOS)
        self.create_subscription(
            ChassisMode, '/chassis_mode', self._on_chassis_mode, HEARTBEAT_QOS)
        self.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)
        self.create_subscription(
            TaskCommand, g('vla_command_topic').value, self._on_task_command, 10)
        # 계약 §5.1 "locked heartbeat는 ... controller fault 0 ... 을 실제 확인한다" —
        # moveit_dynamixel_bridge가 Hardware Error Status를 집계해 발행(내부용 토픽,
        # 파워트레인 DDS 경계를 넘지 않음). _is_settled()에서 게이트로 사용.
        self.create_subscription(
            Bool, '/dynamixel/controller_fault', self._on_controller_fault, 10)
        self.create_subscription(String, '/tool/status', self._on_tool_status, 10)
        self.create_subscription(Bool, '/tool/emergency_stop', self._on_tool_stop, 10)
        self.create_subscription(Bool, '/tool/detached', self._on_tool_stop, 10)
        self.create_subscription(String, '/control/mode', self._on_control_mode, 10)

        self.pub_status = self.create_publisher(ArmStatus, '/arm_status', HEARTBEAT_QOS)
        self.pub_task_result = self.create_publisher(
            TaskResult, g('vla_result_topic').value, 10)
        self.pub_fsm_state = self.create_publisher(String, '/fsm/state', 10)
        self.pub_control_mode = self.create_publisher(
            String, '/control/mode_status', 10)
        self.pub_contact_status = self.create_publisher(
            Bool, '/sensors/contact_status', 10)

        # TF: base_link ← tip_link 조회용 (LIFT 시 현재 TCP 기준 수직 리프트 계산)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # MoveIt action is used when ik_mode == 'moveit'.
        self._move = ActionClient(self, MoveGroup, 'move_action')
        self._cleaning_pub = self.create_publisher(Bool, '/cleaning/enable', 10)
        self._tool_stop_pub = self.create_publisher(Bool, '/tool/emergency_stop', 10)
        self._gripper = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory')

        # analytic IK 경로 (ik_mode=='analytic', 기본): FK 서비스 + 직접 관절궤적 publish
        # ⚠️ FK 호출은 _tick(타이머 콜백) 안에서 블로킹 대기함 — self 를 spin하면 이미
        # 실행 중인 콜백을 재진입 spin 하게 되어 응답을 못 받고 타임아웃(실측 확인:
        # 독립 스크립트로는 2회 반복만에 수렴하는데 노드 내부에서는 즉시 실패).
        # 별도 헬퍼 노드/이그제큐터로 분리해서 우회.
        self._fk_node = rclpy.create_node('arm_fsm_fk_client')
        self._fk_client = self._fk_node.create_client(GetPositionFK, '/compute_fk')
        self._arm_traj_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self._joint_position = {}          # joint_name -> position(rad), /joint_states 에서 갱신
        self._joint_effort = {}
        self._arm_move_deadline = None      # analytic 이동 완료 예상 시각

        # ── 내부 상태 ─────────────────────────────
        # 빈손으로 접혀 잠긴 평상시 상태. 외부 heartbeat의 STOWED_LOCKED와 내부 FSM
        # 상태를 일치시켜, 시작 직후에도 새 pickup conjunction을 받을 수 있게 한다.
        self.state = State.STOWED_LOCKED
        self._state_enter_t = self.get_clock().now()
        self._prev_state = None
        self.locked = False
        self.pick_target = None
        self.mission_id = 0
        self.task_command = ''
        self.tool_type = ''
        self._tool_status = None
        self._tool_status_stamp = None
        self._gripper_command_state = 'idle'
        self._gripper_command_ok = False
        self.control_mode = 'FSM'
        self._task_result_sent = False
        # 브릿지 controller fault 게이트 — 첫 샘플 받기 전엔 알 수 없으니 보수적으로 True
        # (TF 미가용 시 _is_settled()가 False를 리턴하는 것과 같은 안전 측 기본값).
        self._controller_fault = True
        # 계약 v2 — MISSION_STOP + ArrivalStatus conjunction 게이트 (순서 무관)
        self._mission_stop_active = False
        self._chassis_mode = None
        self._pending_arrival = None        # 아직 소비 안 한 최신 ArrivalStatus
        self._last_arrival_stamp = None
        self._last_chassis_stamp = None
        self._last_chassis_recv_wall = None     # chassis_mode 마지막 수신 시각(워치독용)
        self._last_completed_mission_id = None  # STOWED_LOCKED 도달한 mission_id (중복 재실행 방지)
        # locked-heartbeat 실측 확인용 (_is_settled)
        self._settle_start = None          # 안정 유지 시작 시각(불안정 감지 시 리셋)
        self._last_settle_pos = None       # 직전 tick의 tip 위치(np.array)
        self._last_settle_time = None
        self._last_settle_joints = {}      # 직전 tick의 관절각 스냅샷(유한차분 속도용)
        # 팔 모션(MoveIt/직접궤적) 진행 추적
        self._motion_state = 'idle'        # 'idle' | 'active' | 'done'
        self._motion_ok = False
        self._arm_goal_handle = None
        self._cleaning_command_sent = False
        self._stow_move_sent = False       # 상태 진입 시 _transition에서 리셋
        self._carry_home_sent = False      # carry_home 접이 모션 1회 발행 플래그

        # ── heartbeat ─────────────────────────────
        # 계약: 현재 상태를 10Hz 로 끊임없이 발행한다. 0.5초 넘게 끊기면 파워트레인이
        # arm_status_stale 로 차를 세운다.
        #
        # 발행 경로는 **반드시 이 타이머 하나뿐**이어야 한다. 상태 핸들러가 각자
        # publish 하면 stamp 가 뒤섞여 나갈 수 있는데, 파워트레인은 stamp 가 0.5초 이상
        # 역행하면 **영구 latch**(프로세스 재시작 전까지 해제 불가)를 건다.
        # 그래서 핸들러는 _set_status() 로 값만 바꾸고, 실제 발행은 여기서만 한다.
        #
        # 별도 콜백그룹인 이유: _tick 은 analytic IK 의 FK 호출에서 블로킹 대기한다.
        # 같은 그룹이면 IK 도는 동안 heartbeat 가 굶어 stale 판정을 맞는다.
        # → main() 이 MultiThreadedExecutor 로 띄운다.
        #
        # ⚠️ 기동 직후엔 실제 접힘 자세를 아직 확인 못 했으므로 STOWED_LOCKED(주행 허가)로
        # 시작하지 않는다 — 첫 _tick() 이전 짧은 순간(재시작 시 팔이 임의 자세여도) 파워
        # 트레인이 즉시 주행 허가로 오인할 수 있었다. _do_stowed_locked()가 실제 검증 후
        # 승격하도록 아래에서 함께 수정 — _do_locked()가 이미 쓰는 것과 같은 안전 측
        # 기본값(ARM_EXECUTING, 미확인 상태 표시)으로 시작한다.
        self._status = ARM_EXECUTING
        self._hb_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(1.0 / HEARTBEAT_RATE_HZ, self._publish_heartbeat,
                          callback_group=self._hb_group)

        period = 1.0 / g('tick_rate').value
        self._tick_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(period, self._tick, callback_group=self._tick_group)
        self.get_logger().info(
            f'arm_fsm_node started (MoveIt 경로, state=IDLE, cleaning_actuator='
            f'{self.cleaning_actuator_joint or "UNCONFIGURED"}, '
            f'tool_type={self.selected_tool_type}, '
            f'tool_profile_valid={self.tool_profile_valid}, '
            f'heartbeat={HEARTBEAT_RATE_HZ}Hz)'
        )

    # ── 콜백 ───────────────────────────────────

    def _on_pick_target(self, msg):
        self.pick_target = msg

    def _on_task_command(self, msg):
        """Accept only mission-level VLA commands; hardware remains behind this FSM."""
        command = msg.command.upper()
        if self.control_mode != 'FSM':
            self._publish_task_result(
                msg.mission_id, False, self.state.name,
                'control ownership is MANUAL')
            return
        if command not in {'CLEAN', 'PICK', 'MOVE', 'STOP', 'STOW',
                           'VALIDATE_OPEN'}:
            self._publish_task_result(msg.mission_id, False, 'REJECTED', 'unknown command')
            return
        if command == 'VALIDATE_OPEN':
            if (not self.validation_mode
                    or self.selected_tool_type != 'spur_1motor_gripper'):
                self._publish_task_result(
                    msg.mission_id, False, 'REJECTED',
                    'VALIDATE_OPEN requires validation_mode + spur profile')
                return
            self.mission_id = msg.mission_id
            self.task_command = command
            self.tool_type = self.selected_tool_type
            self._task_result_sent = False
            self._transition(State.TOOL_ACTION)
            return
        if command == 'STOP':
            self._cancel_arm_motion()
            self._set_cleaning(False)
            self._tool_stop_pub.publish(Bool(data=True))
            self._publish_task_result(msg.mission_id, True, 'IDLE', 'stopped')
            self._transition(State.IDLE)
            return
        if command == 'STOW':
            self.mission_id = msg.mission_id
            self._transition(State.STOWING)
            return
        if self.state != State.IDLE:
            self._publish_task_result(msg.mission_id, False, self.state.name, 'FSM busy')
            return
        requested_tool = msg.tool_type.strip()
        accepted_aliases = {
            'gripper': ('spur_1motor_gripper', 'dual_motor_gripper'),
            'cleaner': ('cleaner',),
        }
        if (requested_tool and requested_tool != self.selected_tool_type
                and self.selected_tool_type not in accepted_aliases.get(
                    requested_tool, ())):
            self._publish_task_result(
                msg.mission_id, False, 'REJECTED',
                f'command requests {requested_tool}, selected tool is '
                f'{self.selected_tool_type}')
            return
        if not self.vla_standalone_mode and not self._mission_stop_active:
            self._publish_task_result(
                msg.mission_id, False, 'LOCKED', 'MISSION_STOP interlock not active')
            return
        self.mission_id = msg.mission_id
        self.task_command = command
        self.tool_type = self.selected_tool_type
        self._task_result_sent = False
        target = DetectedObject()
        target.class_name = msg.target_object
        target.confidence = msg.confidence
        target.pose = msg.target_pose.pose
        self.pick_target = target
        self._transition(State.PERCEIVE)

    def _on_arrival(self, msg):
        if not self._stamp_is_fresh(msg.header.stamp, self._last_arrival_stamp):
            self.get_logger().warn('ArrivalStatus stamp 무효(0/미래/역행) — 무시')
            return
        self._last_arrival_stamp = msg.header.stamp
        self._pending_arrival = msg
        self._try_advance()

    def _on_chassis_mode(self, msg):
        if not self._stamp_is_fresh(msg.header.stamp, self._last_chassis_stamp):
            return
        self._last_chassis_stamp = msg.header.stamp
        self._last_chassis_recv_wall = self.get_clock().now()
        self._chassis_mode = msg.mode

        if msg.mode not in RECOGNIZED_MODES:
            # 미인식 mode — default-deny, 상태 변경 없음(락 유지)
            self.get_logger().warn(f'미인식 chassis_mode={msg.mode!r} — 무시')
            return

        # 계약 v2: MISSION_STOP만이 유일한 언락·작업 허가. DRIVING 포함 나머지는 전부
        # 잠금 유지 — LOCKED 탈출은 _try_advance()의 MISSION_STOP+ArrivalStatus
        # conjunction으로만 가능(자동 언락 분기 없음).
        self._mission_stop_active = (msg.mode == MODE_MISSION_STOP)

        if msg.mode in LOCK_MODES:      # contract.py 기준 DRIVING 포함
            self._enter_locked()
        elif msg.mode == MODE_STOW_REQUEST and self.state in STOW_ABORTABLE_STATES:
            # 운영자 포기/재정렬 유도 — 진행 중인 작업(또는 GRIP_LOST/LOCKED 래치) 중단하고
            # 접어 잠금. 2026-07-14: GRIP_LOST 전용이었던 범위를 작업 중 모든 상태로 확장.
            # 화물을 든 채 공중일 수 있으면(PAYLOAD_ALOFT_STATES, 또는 LIFT 중 LOCKED로
            # 중단된 경우) 그리퍼를 바로 열지 않고 LOWER_RELEASE로 먼저 내린다 — §5.1
            # 잔여 합의 ①(무조건 RELEASE 전이 = 화물 낙하 경로) 대응.
            aloft = (self.state in PAYLOAD_ALOFT_STATES
                     or (self.state == State.LOCKED and self._prev_state == State.RETRACT))
            self._cancel_arm_motion()
            self._set_cleaning(False)
            self.locked = False
            if self.mission_type == MISSION_ROTARY_TOOL:
                self._transition(State.STOWING)
            else:
                self._transition(State.LOWER_RELEASE if aloft else State.RELEASE)

        self._try_advance()

    def _enter_locked(self):
        """PREEMPTIBLE_STATES 중이면 모션 취소 후 LOCKED 진입. 이미 락이면 no-op."""
        self.locked = True
        if self.state in PREEMPTIBLE_STATES:
            self._prev_state = self.state
            self._cancel_arm_motion()
            self._set_cleaning(False)
            self._transition(State.LOCKED)

    def _try_advance(self):
        """MISSION_STOP + ArrivalStatus conjunction(순서 무관) 충족 시에만 전이.

        픽업 개시: IDLE/STOWED_LOCKED/GRIP_LOST/FAILED 에서 ARRIVED_PICKUP 수신 시
        PERCEIVE.
        지형 중단 복귀: LOCKED(같은 mission_id) 에서도 동일 conjunction으로 PERCEIVE 재진입
        (중단 시점 재개 대신 PERCEIVE부터 다시 — 중단 중 타겟이 변했을 수 있어 더 안전).
        하역: CARRY 에서 ARRIVED_DROP(같은 mission_id) 수신 시 RELEASE.
        이미 STOWED_LOCKED 까지 끝난 mission_id 의 재발행(중복)은 무시.
        """
        msg = self._pending_arrival
        if msg is None or not self._mission_stop_active:
            return

        pickup_states = (State.IDLE, State.STOWED_LOCKED, State.GRIP_LOST, State.FAILED)
        if self.state in pickup_states and msg.status == ARRIVED_PICKUP:
            if msg.mission_id == self._last_completed_mission_id:
                return  # 이미 완료된 mission_id 재발행 — 재실행 금지
            self.mission_id = msg.mission_id
            if not self.task_command:
                self.task_command = (
                    'CLEAN' if self.selected_tool_type == 'cleaner' else 'PICK')
                self.tool_type = self.selected_tool_type
            self._pending_arrival = None
            self._set_status(ARM_WORK_READY)
            self._transition(
                self._rotary_entry_state()
                if self.mission_type == MISSION_ROTARY_TOOL
                else State.PERCEIVE)
            self._pending_arrival = None
        elif (self.state == State.LOCKED and msg.status == ARRIVED_PICKUP
                and msg.mission_id == self.mission_id):
            self.locked = False
            self._set_status(ARM_WORK_READY)
            self._transition(
                self._rotary_entry_state()
                if self.mission_type == MISSION_ROTARY_TOOL
                else State.PERCEIVE)
            self._pending_arrival = None
        elif (self.state == State.CARRY and msg.status == ARRIVED_DROP
                and msg.mission_id == self.mission_id):
            self._transition(State.RELEASE)
            self._pending_arrival = None

    def _rotary_entry_state(self):
        if not self.integrated_test_mode:
            return State.END_EFFECTOR_ROTATE
        return (State.RANDOM_ARM_DEMO if self.random_demo_enabled
                else State.ARM_TEST_MOVE)

    def _stamp_is_fresh(self, stamp, prev_stamp):
        """0/미래/동일·역행 stamp 거부 (계약 §5.1 heartbeat freshness 기준)."""
        t = stamp.sec + stamp.nanosec * 1e-9
        if t <= 0.0:
            return False
        now = self.get_clock().now().nanoseconds * 1e-9
        if t > now + STAMP_FUTURE_TOL:
            return False
        if prev_stamp is not None:
            pt = prev_stamp.sec + prev_stamp.nanosec * 1e-9
            if t <= pt:
                return False
        return True

    def _on_joint_states(self, msg):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self._joint_position[name] = msg.position[i]
            if i < len(msg.effort):
                self._joint_effort[name] = abs(float(msg.effort[i]))

    def _on_controller_fault(self, msg):
        self._controller_fault = bool(msg.data)

    def _on_tool_status(self, msg):
        try:
            status = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if status.get('tool_type') != self.selected_tool_type:
            self.get_logger().error(
                'bridge tool_type differs from FSM selection; motion remains blocked')
            self._tool_status = None
            self._tool_status_stamp = None
            return
        self._tool_status = status
        self._tool_status_stamp = self.get_clock().now()

    def _on_tool_stop(self, msg):
        if not msg.data:
            return
        self._set_cleaning(False)
        self._cancel_arm_motion()
        if self.state not in (State.IDLE, State.STOWED_LOCKED):
            self._set_status(ARM_FAILED)
            self._transition(State.STOWING)

    def _on_control_mode(self, msg):
        requested = msg.data.strip().upper()
        self.get_logger().info(
            f'Control mode request received: requested={requested!r}, '
            f'fsm_state={self.state.name}, current_mode={self.control_mode}')
        if requested not in ('MANUAL', 'FSM'):
            self.get_logger().warn(
                f'Control mode request denied: unsupported mode {requested!r}')
            return
        if (requested == 'MANUAL'
                and self.state not in (State.IDLE, State.STOWED_LOCKED)):
            self.get_logger().warn(
                f'Control mode request denied: MANUAL requires FSM state '
                f'IDLE or STOWED_LOCKED; actual={self.state.name}')
            return
        self.control_mode = requested
        self.get_logger().info(
            f'Control mode request accepted: mode={self.control_mode}, '
            f'fsm_state={self.state.name}')

    def _tool_ready(self):
        if (self.selected_tool_type != 'cleaner'
                and not self.tool_profile_valid and not self.validation_mode):
            return False
        if self.dry_run_mode:
            return True
        if self._tool_status is None or self._tool_status_stamp is None:
            return False
        age = (self.get_clock().now() - self._tool_status_stamp).nanoseconds * 1e-9
        return (age <= self.tool_status_timeout
                and (bool(self._tool_status.get('profile_valid'))
                     or (self.validation_mode
                         and bool(self._tool_status.get('validation_mode'))))
                and bool(self._tool_status.get('actuators_discovered'))
                and bool(self._tool_status.get('motion_allowed')))

    # ── FSM tick ───────────────────────────────

    def _tick(self):
        self.pub_fsm_state.publish(String(data=self.state.name))
        self.pub_control_mode.publish(String(data=self.control_mode))
        self.pub_contact_status.publish(Bool(data=self.sensors.contact_confirmed()))
        if self.control_mode == 'MANUAL':
            return
        self._check_chassis_mode_watchdog()
        if (self._motion_state == 'active' and self._arm_move_deadline is not None
                and self.get_clock().now() >= self._arm_move_deadline):
            self._motion_state = 'done'
            self._arm_move_deadline = None
        handler = getattr(self, f'_do_{self.state.name.lower()}', None)
        if handler:
            handler()

    def _check_chassis_mode_watchdog(self):
        """chassis_mode 수신 끊김 = default-deny(잠금 유지, §5.1)."""
        if self._last_chassis_recv_wall is None:
            return  # 아직 한 번도 못 받음 — IDLE 기본값(안 움직임)으로 이미 안전
        age = (self.get_clock().now() - self._last_chassis_recv_wall).nanoseconds * 1e-9
        if age > self.chassis_mode_timeout:
            self._mission_stop_active = False
            self._enter_locked()

    def _is_settled(self):
        """locked 하트비트(CARRYING_LOCKED/STOWED_LOCKED) 발행 전 실제 확인 — §5.1.

        tip pose(TF, base_frame←tip_link)가 연속 tick 사이 `locked_pos_tol` 이내로
        유지되고, 관절각 유한차분 속도가 `locked_vel_tol` 이내인 상태가 `locked_dwell`
        초 이상 지속돼야 True. TF 조회 실패·불안정 감지 시 dwell 타이머 리셋(안전 측
        기본값 = 미확인). controller fault(`/dynamixel/controller_fault`, 브릿지가
        Hardware Error Status 집계) 가 True 이면 즉시 미확인 처리(2026-07-15 추가 —
        이전엔 브릿지에 해당 필드가 없어 미포함이었음).
        """
        if self.dry_run_mode:
            return True
        if self._controller_fault:
            self._settle_start = None
            return False
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.tip_link, Time())
        except TransformException:
            self._settle_start = None
            return False
        t = tf.transform.translation
        pos = np.array([t.x, t.y, t.z])
        now = self.get_clock().now()

        stable = False
        if self._last_settle_pos is not None and self._last_settle_time is not None:
            dt = (now - self._last_settle_time).nanoseconds * 1e-9
            pos_delta = float(np.linalg.norm(pos - self._last_settle_pos))
            max_joint_vel = 0.0
            if dt > 1e-6:
                for name in ARM_JOINT_NAMES:
                    prev = self._last_settle_joints.get(name)
                    cur = self._joint_position.get(name)
                    if prev is not None and cur is not None:
                        max_joint_vel = max(max_joint_vel, abs(cur - prev) / dt)
            stable = (pos_delta <= self.locked_pos_tol
                      and max_joint_vel <= self.locked_vel_tol)

        self._last_settle_pos = pos
        self._last_settle_time = now
        self._last_settle_joints = dict(self._joint_position)

        if not stable:
            self._settle_start = None
            return False
        if self._settle_start is None:
            self._settle_start = now
        return (now - self._settle_start).nanoseconds * 1e-9 >= self.locked_dwell

    def _do_idle(self):
        # 계약 v2: 평상시(빈손) 주행 중 상시 하트비트 — WORK_READY 아님, STOWED_LOCKED.
        # WORK_READY는 MISSION_STOP+ArrivalStatus conjunction 수락 순간(_try_advance)의
        # 1회성 ack로 재배치됨. (STOWING에서 settle 확인 후 넘어온 상태라 여기선 재확인 안 함)
        self._set_status(ARM_STOWED_LOCKED)

    def _do_perceive(self):
        self._set_status(ARM_PERCEIVING)
        if self.pick_target is None:
            return
        if self.pick_target.pose.position.z == 0.0:   # depth 무효 (Phase 2 require_depth 기준)
            self.get_logger().warn('pick_target depth 무효 — 대기')
            return
        self._transition(State.PLAN)

    def _do_plan(self):
        """파지 목표로 이동 시작. 디스패치만 하고 DESCEND에서 대기."""
        self._set_status(ARM_PLANNING)
        if self.task_command in ('PICK', 'CLEAN') and not self._tool_ready():
            self.get_logger().error(
                f'{self.selected_tool_type} profile/backend not ready; arm motion blocked')
            self._publish_task_result(
                self.mission_id, False, 'PLAN', 'tool backend not ready')
            self._set_status(ARM_FAILED)
            self._transition(State.IDLE)
            return
        if not self.sensors.obstacle_clear():
            return
        if self.dry_run_mode:
            self._motion_ok = True
            self._motion_state = 'done'
            self._transition(State.APPROACH)
            return
        if self.ik_mode == 'moveit':
            return self._planned_grasp_pose is not None and self._planned_approach_pose is not None
        return self._planned_grasp_xyz is not None and self._planned_approach_xyz is not None

    def _clear_frozen_target(self):
        """얼린 타겟을 버린다 — 새 mission 진입 시 호출."""
        self._planned_grasp_pose = None
        self._planned_approach_pose = None
        self._planned_grasp_xyz = None
        self._planned_approach_xyz = None

    def _do_plan(self):
        """검출 결과 하나를 고정하고 접근 목표와 파지 목표를 각각 계산한다."""
        self._set_status(ARM_PLANNING)
        if self.freeze_target_on_retry and self._has_frozen_target():
            # 같은 mission 안의 재시도 — 팔이 시야에 들어와 오염됐을 수 있는 새 관측 대신
            # 처음 얼린 타겟을 그대로 쓴다(파라미터 주석 참고).
            self.get_logger().info('freeze_target_on_retry: 기존 타겟 재사용 (재인식 생략)')
            self._transition(State.APPROACH)
            return
        if self.ik_mode == 'moveit':
            self._planned_grasp_pose = self._grasp_pose_in_base()
            if self._planned_grasp_pose is None:
                self._fail('target pose transform failed')
                return
            self._begin_arm_move(grasp_pose)
            self._transition(State.APPROACH)
            return

        target = self._grasp_target_xyz()
        if target is None or not self._move_to_xyz(target):
            self._set_status(ARM_FAILED)
            self._transition(State.IDLE)
            return
        self._transition(State.APPROACH)

    def _do_approach(self):
        """MoveIt 모션 결과 대기 (저속 실행 = 하강 포함). TODO: 접촉 시 arm effort 감시."""
        self._set_status(ARM_EXECUTING)
        if not self.sensors.obstacle_clear():
            self.get_logger().warn('obstacle sensor blocked/stale → motion cancel')
            self._cancel_arm_motion()
            self._set_status(ARM_FAILED)
            self._transition(State.IDLE)
            return
        if self._motion_state == 'active':
            return
        ok = self._motion_ok
        self._motion_state = 'idle'
        self._transition(State.TOOL_ACTION if ok else State.IDLE)
        if not ok:
            self._set_status(ARM_FAILED)

    def _do_tool_action(self):
        """Dispatch to a backend without exposing actuator APIs to the VLA layer."""
        if self.task_command == 'VALIDATE_OPEN':
            self._do_validation_open()
        elif self.task_command == 'MOVE':
            self._transition(State.RETRACT)
        elif (self.task_command == 'PICK'
              and self.selected_tool_type in (
                  'spur_1motor_gripper', 'dual_motor_gripper')):
            self._transition(State.GRASP)
        elif (self.task_command == 'CLEAN'
              and self.selected_tool_type == 'cleaner'):
            self._transition(State.CLEAN_START)
        else:
            self._publish_task_result(
                self.mission_id, False, 'TOOL_ACTION', 'unsupported tool backend')
            self._transition(State.STOWING)

    def _do_validation_open(self):
        """Validation-only OPEN with real position action and simulated effort."""
        self._set_status(ARM_EXECUTING)
        if not self._tool_ready():
            self._fail_tool_action('validation gripper backend unavailable')
            return
        if self._gripper_command_state == 'idle':
            self._send_gripper(float(self.tool_profile.get('open_position', 1.0)))
            return
        if self._gripper_command_state == 'active':
            return
        ok = self._gripper_command_ok
        self._gripper_command_state = 'idle'
        self._publish_task_result(
            self.mission_id, bool(ok), 'TOOL_ACTION',
            'validation OPEN: effort result simulated' if ok
            else 'validation OPEN action failed')
        self._transition(State.IDLE)

    def _do_grasp(self):
        """Shared GRASP sub-FSM entry for either calibrated gripper backend."""
        self._set_status(ARM_EXECUTING)
        if not self._tool_ready():
            self._fail_tool_action('gripper backend became unavailable')
            return
        if self._gripper_command_state == 'idle':
            self._send_gripper(float(self.tool_profile.get('close_position', 0.0)))
            return
        if self._gripper_command_state == 'active':
            return
        ok = self._gripper_command_ok
        self._gripper_command_state = 'idle'
        if ok:
            self._transition(State.GRASP_CHECK)
        else:
            self._fail_tool_action('gripper action failed')

    def _do_grasp_check(self):
        self._set_status(ARM_EXECUTING)
        threshold = float(self.tool_profile.get('grasp_threshold', float('inf')))
        effort = self._tool_effort()
        if effort >= threshold:
            self._transition(State.RETRACT)
            return
        if self._elapsed() >= float(self.tool_profile.get('action_time', 0.0)) + 0.5:
            self._fail_tool_action(
                f'grasp effort {effort:.1f} below threshold {threshold:.1f}')

    def _fail_tool_action(self, reason):
        self.get_logger().error(reason)
        self._tool_stop_pub.publish(Bool(data=True))
        self._publish_task_result(
            self.mission_id, False, self.state.name, reason)
        self._set_status(ARM_FAILED)
        self._transition(State.STOWING)

    def _do_clean_start(self):
        """털털이 회전을 시작한다. 미설정 actuator에서는 안전하게 실패한다."""
        self._set_status(ARM_EXECUTING)
        if not self.cleaning_actuator_joint and not self.dry_run_mode:
            self.get_logger().error('cleaning_actuator_joint 미설정 — 털털이 구동 금지')
            self._set_status(ARM_FAILED)
            self._transition(State.RELEASE)
            return
        if not self._cleaning_command_sent:
            self._set_cleaning(True)
            self._cleaning_command_sent = True
            return
        if self._elapsed() < self.cleaning_start_time:
            return
        self._transition(State.CONTACT_CHECK)

    def _do_contact_check(self):
        """SensorManager의 contact backend fusion 결과를 확인한다."""
        self._set_status(ARM_EXECUTING)
        if self.sensors.contact_confirmed():
            self._transition(State.CLEAN)
        elif self._elapsed() >= self.contact_timeout:
            self.get_logger().warn('털털이 접촉 확인 실패 → CLEAN_STOP')
            self._set_status(ARM_FAILED)
            self._transition(State.RELEASE)

    def _do_clean(self):
        self._set_status(ARM_EXECUTING)
        if self._elapsed() >= self.clean_duration:
            self._transition(State.CLEAN_STOP)

    def _do_clean_stop(self):
        self._set_status(ARM_EXECUTING)
        if not self._cleaning_command_sent:
            self._set_cleaning(False)
            self._cleaning_command_sent = True
            return
        if self._elapsed() >= 0.2:
            self._transition(State.RETRACT)

    def _do_retract(self):
        """수직 리프트 → 운반 자세 (base_link +Z, 현재 tip TF 기준)."""
        self._set_status(ARM_EXECUTING)
        if self._motion_state == 'active':
            return
        if self._motion_state == 'done':
            ok = self._motion_ok
            self._motion_state = 'idle'
            self._transition(State.LOCK_CHECK)
            return

        # 'idle' — 리프트 모션 시작
        if self.ik_mode == 'moveit':
            lift_pose = self._carry_pose()
            if lift_pose is None:
                self.get_logger().warn('carry pose 미구현/TF 실패 — 스킵하고 CARRY 진입')
                self._transition(State.LOCK_CHECK)
                return
            self._begin_arm_move(lift_pose)
            return

        target = self._lift_target_xyz()
        if target is None or not self._move_to_xyz(target):
            self.get_logger().warn('LIFT 목표 계산/이동 실패 — 스킵하고 CARRY 진입')
            self._transition(State.LOCK_CHECK)

    def _do_lock_check(self):
        """리프트 후 기계식/전기식 lock confirmation을 fail-closed 확인한다."""
        self._set_status(ARM_EXECUTING)
        if self.sensors.lock_confirmed():
            self._transition(State.DONE if self.task_command else State.CARRY)
        elif self._elapsed() >= self.lock_check_timeout:
            self.get_logger().warn('lock confirmation missing/stale → RELEASE')
            self._set_status(ARM_FAILED)
            self._transition(State.RELEASE)

    def _do_carry(self):
        """청소 후 이동 가능한 안정 자세를 유지한다.

        `_is_settled()`(pose·관절속도·dwell 실측) 충족 전엔 `CARRYING_LOCKED` 대신
        `EXECUTING`을 발행 — 문자열만 바꾸지 않고 실제 정지 확인 후에만 locked 하트비트.
        """
        self._set_status(ARM_CARRYING_LOCKED if self._is_settled() else ARM_EXECUTING)
        # ARRIVED_DROP 은 _try_advance(MISSION_STOP conjunction)에서 RELEASE로 전이

    def _do_grip_lost(self):
        """supervisor-latched hold — 자동 재시도 없음(§5.1).

        새 MISSION_STOP+ArrivalStatus(ARRIVED_PICKUP) conjunction이 다시 와야만
        `_try_advance()`가 PERCEIVE로 재진입시킴. STOW_REQUEST 수신 시
        `_on_chassis_mode`에서 RELEASE로 강제 전이(운영자 포기, `STOW_ABORTABLE_STATES`).
        """
        self._set_status(ARM_GRIP_LOST)

    def _do_lower_release(self):
        """RELEASE(그리퍼 오픈) 전에 파지 높이까지 내린다 — 화물 낙하 방지(§5.1 잔여 합의 ①).

        `PAYLOAD_ALOFT_STATES`(LIFT/CARRY)에서 STOW_REQUEST로 중단됐을 때만 경유.
        `_carry_pose`/`_lift_target_xyz`가 더한 `lift_height`만큼 되돌려 내려
        grasp 당시 높이 근처로 되돌아간 뒤에만 RELEASE로 넘어간다. TF/IK 실패 시엔
        무한정 대기하지 않고 바로 RELEASE(기존 동작, 낙하 위험을 감수)로 폴백한다 —
        높이를 못 낮추는 것보다 그리퍼를 계속 닫아 매달아두는 쪽이 더 위험할 수 있다.
        """
        self._set_status(ARM_EXECUTING)
        if self._motion_state == 'active':
            return
        if self._motion_state == 'done':
            self._motion_state = 'idle'
            self._transition(State.RELEASE)
            return

        if self.ik_mode == 'moveit':
            pose = self._lower_pose()
            if pose is None:
                self.get_logger().warn('lower pose 계산 실패 — 바로 RELEASE(낙하 위험 감수)')
                self._transition(State.RELEASE)
                return
            self._begin_arm_move(pose)
            return

        target = self._lower_target_xyz()
        if target is None or not self._move_to_xyz(target):
            self.get_logger().warn('내림 이동 실패 — 바로 RELEASE(낙하 위험 감수)')
            self._transition(State.RELEASE)

    def _lower_pose(self):
        """현재 TCP(tip_link)를 base_link 기준 -Z(lift_height만큼) 내린 자세. `_carry_pose` 역."""
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.tip_link, Time())
        except TransformException as e:
            self.get_logger().warn(
                f'lower_pose TF 조회 실패 ({self.base_frame} <- {self.tip_link}): {e}')
            return None
        ps = PoseStamped()
        ps.header.frame_id = self.base_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        t = tf.transform.translation
        ps.pose.position.x = t.x
        ps.pose.position.y = t.y
        ps.pose.position.z = max(0.0, t.z - self.lift_height)
        ps.pose.orientation = tf.transform.rotation
        return ps

    def _lower_target_xyz(self):
        """현재 tip 위치(base_frame)에서 -Z lift_height 만큼 내린 목표. `_lift_target_xyz` 역."""
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.tip_link, Time())
        except TransformException as e:
            self.get_logger().warn(f'lower target TF 조회 실패: {e}')
            return None
        t = tf.transform.translation
        return (t.x, t.y, max(0.0, t.z - self.lift_height))

    def _do_release(self):
        if getattr(self, 'gripper_change_mode', False):
            self._transition(State.DESCEND_STOPPED)
            return
        if (self.mission_type != MISSION_PICK_PLACE
                or self.end_effector_kind != 'gripper'):
            self._fail('RELEASE is disabled for the selected end effector')
            return
        self._set_status(ARM_EXECUTING)
        self._set_cleaning(False)
        if self.tool_profile.get('backend') != 'gripper':
            self._transition(State.DONE)
            return
        if self._gripper_command_state == 'idle':
            if self.dry_run_mode or self._tool_ready():
                self._send_gripper(
                    float(self.tool_profile.get('open_position', 1.0)))
            else:
                # Never invent a motor command after a detach/fault.
                self._transition(State.DONE)
            return
        if self._gripper_command_state == 'active':
            return
        self._gripper_command_state = 'idle'
        self._transition(State.DONE)

    def _do_done(self):
        self._set_status(ARM_EXECUTING)
        if self.task_command and not self._task_result_sent:
            self._publish_task_result(self.mission_id, True, 'DONE', '')
            self._task_result_sent = True
        self._transition(State.STOWING)

    def _do_stowing(self):
        """접힘 자세로 이동 → `_is_settled()` 확인 후 `STOWED_LOCKED`.

        `stow_joint_positions`(파라미터, CAD 미검증 placeholder)로 직접 관절궤적을
        딱 한 번만(`_stow_move_sent`, 상태 진입당 1회) 발행한다. 모션 완료 전에
        `_is_settled()`를 확인하면 CARRY 종료 시점의 자세가 우연히 안정적이어서 실제로
        접히기도 전에 통과해버릴 수 있으므로, 반드시 `_motion_state=='active'`가 아닐
        때만 settle 게이트를 본다. ⚠️ `_motion_state`를 'idle'로 되돌리면 다음 tick에
        재발행 분기를 다시 타 dwell 누적 없이 궤적을 계속 재전송하는 버그가 있었음
        (2026-07-15 발견·수정) — `_stow_move_sent`는 상태를 벗어날 때(`_transition`)만
        리셋되므로 매 tick 재발행되지 않는다.
        """
        self._set_status(ARM_STOWING)
        if self.dry_run_mode:
            self._transition(State.STOWED_LOCKED)
            return
        if self.stow_joint_positions is None:
            # 목표 미설정(파라미터 길이 오류) — 접이 모션 없이 현재 자세 유지로 폴백.
            if self._is_settled():
                self._transition(State.STOWED_LOCKED)
            return
        if not self._stow_move_sent:
            self._begin_stow_move()
            self._stow_move_sent = True
            return
        if self._motion_state == 'active':
            return
        if self._is_settled():
            self._transition(State.STOWED_LOCKED)

    def _do_stowed_locked(self):
        """빈손으로 접혀 잠긴 평상시 상태이자 하역 완료 최종 권위.

        ⚠️ 이 상태 진입 경로는 `_do_stowing()`의 settle 확인된 전이뿐 아니라 **기동 시
        기본값**(__init__의 `self.state = State.STOWED_LOCKED`)도 있다 — 재시작 시 팔이
        실제로 어떤 자세든 있을 수 있는데 그 경우를 검증 없이 신뢰하면 안 된다. `_do_locked()`
        와 동일하게 매 tick `_near_stow_posture()`+`_is_settled()`로 실측 확인하고, 통과
        못 하면 `STOWED_LOCKED`(주행 허가) 대신 `EXECUTING`(미확인)을 발행한다.
        """
        self.pick_target = None
        self.task_command = ''
        self.tool_type = ''
        self._transition(State.IDLE)

    def _do_locked(self):
        """현재 자세 홀드: MoveIt에 새 goal 안 보냄 → 브릿지가 torque로 마지막 위치 유지.

        계약 v2 10Hz 하트비트 요구 — LOCKED 중에도 발행을 멈추면 안 됨. `_is_settled()`
        충족 전엔 `EXECUTING`(아직 정지 미확인). 충족 후 `_prev_state==LIFT`(파지 확정 뒤
        중단, 화물을 든 상태)면 `CARRYING_LOCKED`.

        그 외(PERCEIVE/PLAN/DESCEND/청소 중단, 빈손)는 예전엔 정지만 확인되면
        바로 `STOWED_LOCKED`로 근사했으나, LOCKED는 새 goal을 안 보내(모션 없이 그 자리에서
        홀드) 실제로는 임의의 안 접힌 자세일 수 있다 — 파워트레인은 `STOWED_LOCKED`를
        "차가 출발해도 되는" 근거로 그대로 신뢰하므로 이는 안전 gap이었다(§5.1 잔여 합의 ②).
        `_near_stow_posture()`로 관절각이 실제 `stow_joint_positions` 근처인지 확인한
        경우에만 `STOWED_LOCKED`를 발행하고, 아니면 정직하게 `EXECUTING`을 유지해 파워트레인
        쪽 motion hold를 받는다(거짓 주행 허가보다 안전).
        """
        if not self._is_settled():
            self._set_status(ARM_EXECUTING)
            return
        if self._prev_state == State.RETRACT:
            self._set_status(ARM_CARRYING_LOCKED)
            return
        self._set_status(ARM_STOWED_LOCKED if self._near_stow_posture() else ARM_EXECUTING)

    def _near_stow_posture(self):
        """현재 관절각이 `stow_joint_positions` 근처(±`stow_pos_tol_rad`)인지 확인.

        `stow_joint_positions`가 비활성(파라미터 길이 오류)이면 검증 불가하므로 기존
        동작(정지만 확인)으로 폴백한다 — 이미 그 경우엔 시작 시점에 에러 로그를 남긴다.
        """
        if self.stow_joint_positions is None:
            return True
        for name, target in zip(ARM_JOINT_NAMES, self.stow_joint_positions):
            cur = self._joint_position.get(name)
            if cur is None or abs(cur - target) > self.stow_pos_tol_rad:
                return False
        return True

    # ── MoveIt 팔 모션 ─────────────────────────

    def _begin_arm_move(self, pose_stamped):
        self._motion_state = 'active'
        self._motion_ok = False
        if not self._move.server_is_ready():
            self.get_logger().warn('move_action 서버 미준비 — MoveIt(move_group) 실행 확인')
        goal = self._build_move_group_goal(pose_stamped)
        self._move.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn('MoveGroup goal 거부됨')
            self._motion_state = 'done'
            self._motion_ok = False
            return
        self._arm_goal_handle = gh
        gh.get_result_async().add_done_callback(self._on_arm_result)

    def _on_arm_result(self, future):
        result = future.result().result
        self._motion_ok = (result.error_code.val == MOVEIT_SUCCESS)
        self._motion_state = 'done'
        self._arm_goal_handle = None

    def _cancel_arm_motion(self):
        if self._arm_goal_handle is not None:
            self._arm_goal_handle.cancel_goal_async()
            self._arm_goal_handle = None
        self._arm_move_deadline = None
        self._motion_state = 'idle'

    def _cancel_end_effector_motion(self):
        if self._rotate_goal_handle is not None:
            self._rotate_goal_handle.cancel_goal_async()
            self._rotate_goal_handle = None
        self._rotate_state = 'idle'

    def _cancel_arm_test_motion(self):
        if self._arm_test_goal_handle is not None:
            self._arm_test_goal_handle.cancel_goal_async()
            self._arm_test_goal_handle = None
        self._arm_test_state = 'idle'

    def _build_move_group_goal(self, pose_stamped):
        """목표 pose → MoveGroup goal (plan & execute). tip_link를 pose로 이동."""
        req = MotionPlanRequest()
        req.group_name = self.planning_group
        req.num_planning_attempts = 5
        req.allowed_planning_time = self.planning_time
        req.max_velocity_scaling_factor = self.vel_scale
        req.max_acceleration_scaling_factor = self.acc_scale

        pc = PositionConstraint()
        pc.header = pose_stamped.header
        pc.link_name = self.tip_link
        region = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.pos_tol]
        region.primitives.append(sphere)
        region.primitive_poses.append(pose_stamped.pose)
        pc.constraint_region = region
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header = pose_stamped.header
        oc.link_name = self.tip_link
        oc.orientation = pose_stamped.pose.orientation
        oc.absolute_x_axis_tolerance = self.orient_tol
        oc.absolute_y_axis_tolerance = self.orient_tol
        oc.absolute_z_axis_tolerance = self.orient_tol
        oc.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pc)
        # KDL/OMPL은 off-axis TCP의 position-only goal에서 유효한 goal state를
        # 샘플하지 못한다. single TCP에는 인식/TF로 변환된 자세를
        # 함께 제약해 IK sampler가 실제 TCP chain을 풀게 한다. dual은
        # 기존 4축 position-only 계약을 그대로 유지한다.
        if self.tip_link == 'single_gripper_grasp_frame':
            constraints.orientation_constraints.append(oc)
        req.goal_constraints.append(constraints)

        goal = MoveGroup.Goal()
        goal.request = req
        # planning_options 기본값: plan_only=False → 계획 후 컨트롤러로 실행
        return goal

    def _grasp_pose(self):
        """/pick_target(DetectedObject, frame 정보 없음) → PoseStamped(pick_frame_id)."""
        if self.pick_target is None:
            return None
        ps = PoseStamped()
        ps.header.frame_id = self.pick_frame_id    # DetectedObject엔 header 없음 → 파라미터 사용
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose = self.pick_target.pose
        return ps

    def _grasp_pose_in_base(self):
        """선택한 인식 자세를 MoveIt 계획 프레임으로 변환한다."""
        source = self._grasp_pose()
        if source is None:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, source.header.frame_id, Time())
        except TransformException as e:
            self.get_logger().warn(f'grasp pose TF lookup failed: {e}')
            return None
        result = PoseStamped()
        result.header.frame_id = self.base_frame
        result.header.stamp = self.get_clock().now().to_msg()
        result.pose = do_transform_pose(source.pose, tf)
        return result

    @staticmethod
    def _offset_pose_z(pose, offset):
        # ROS 메시지는 변경 가능하므로, 접근 오프셋이 DESCEND에서 사용할 고정 파지
        # 목표를 바꾸지 않도록 복사한다.
        result = deepcopy(pose)
        result.pose.position.z += offset
        return result

    def _carry_pose(self):
        """현재 TCP(tip_link)를 base_link 기준 +Z 로 들어올린 운반 자세.

        파지 직후의 실제 말단 자세를 TF(base_frame←tip_link)로 조회 → z 에 lift_height
        를 더하고 orientation 은 유지(박스 자세 보존). base_frame 이 planning frame 이라
        MoveIt 이 바로 계획 가능. TF 미가용 시 None → 호출부(_do_lift)가 LIFT 스킵.
        """
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.tip_link, Time())
        except TransformException as e:
            self.get_logger().warn(
                f'carry_pose TF 조회 실패 ({self.base_frame} <- {self.tip_link}): {e}')
            return None

        ps = PoseStamped()
        ps.header.frame_id = self.base_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        t = tf.transform.translation
        ps.pose.position.x = t.x
        ps.pose.position.y = t.y
        ps.pose.position.z = t.z + self.lift_height
        ps.pose.orientation = tf.transform.rotation
        return ps

    # ── analytic IK (ik_mode=='analytic', URDF 3관절 한정) ─────

    def _current_arm_joint_positions(self):
        return [self._joint_position.get(j, 0.0) for j in ARM_JOINT_NAMES]

    def _grasp_target_xyz(self):
        """/pick_target(카메라 프레임) → base_frame 기준 (x,y,z). 방향은 무시(3DOF 한계)."""
        if self.pick_target is None:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.pick_frame_id, Time())
        except TransformException as e:
            self.get_logger().warn(f'grasp target TF 조회 실패: {e}')
            return None
        out = do_transform_pose(self.pick_target.pose, tf)
        return (out.position.x, out.position.y, out.position.z)

    def _lift_target_xyz(self):
        """현재 tip 위치(base_frame)에서 +Z lift_height 만큼 든 목표."""
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.tip_link, Time())
        except TransformException as e:
            self.get_logger().warn(f'lift target TF 조회 실패: {e}')
            return None
        t = tf.transform.translation
        return (t.x, t.y, t.z + self.lift_height)

    def _fk_tip(self, q):
        """3관절 각도 q=[j1,j2,j3] → tip_link 위치(base_frame) np.array, 실패 시 None."""
        req = GetPositionFK.Request()
        req.header.frame_id = self.base_frame
        req.fk_link_names = [self.tip_link]
        req.robot_state = RobotState()
        req.robot_state.joint_state.name = list(ARM_JOINT_NAMES)
        req.robot_state.joint_state.position = [float(v) for v in q]
        if not self._fk_client.service_is_ready():
            return None
        future = self._fk_client.call_async(req)
        rclpy.spin_until_future_complete(self._fk_node, future, timeout_sec=1.0)
        res = future.result()
        if res is None or not res.pose_stamped:
            return None
        p = res.pose_stamped[0].pose.position
        return np.array([p.x, p.y, p.z])

    def _solve_position_ik(self, target_xyz, q_init):
        """FK + 수치 자코비안(finite-difference) 로 위치만 맞추는 IK.

        MoveIt 6DOF pose IK 대신 이 방식을 기본으로 씀(HW-7 실측 확인, compute_ik 가 현재
        실제 tip pose 에도 NO_IK_SOLUTION 반환하던 문제 회피). 방향은 포기하고 위치만
        댐핑 최소자승(Levenberg-Marquardt 유사)으로 반복 수렴.

        ⚠️ 관절 수는 `ARM_JOINT_NAMES`(=JOINT_CONFIG) 에서 온다 — **여기에 상수로 박지 말 것.**
        2026-08-08 수정: 예전엔 3 이 하드코딩돼 있었는데 실기 배선 확정으로 구동 관절이
        4개(arm_joint_2~5, id 14/13/12/16)가 되면서 `q + dq` 가 shape (4,)+(3,) 로 깨져
        APPROACH 에서 무조건 ValueError 로 죽었다(실기 dry-run 에서 발견).
        자코비안은 3×n(작업공간 3차원 × 관절 n)이고, 감쇠항 `np.eye(3)` 은 작업공간 쪽이라
        관절 수와 무관하게 3 이 맞다.
        """
        q = np.array(q_init, dtype=float)
        target = np.array(target_xyz, dtype=float)
        eps = 0.05
        lam = 0.01
        max_step = 0.4

        # 마지막 해의 위치 잔차 [m] — 호출부가 로그에 실어 "수렴(ik_tol)" 과 "겨우 수용
        # (ik_accept_tol)" 을 구분할 수 있게 한다. 이 둘은 3배 차이(1cm vs 3cm)인데
        # 로그가 같으면 파지가 빗나갈 때 IK 를 의심할지 캘리브를 의심할지 못 가린다.
        self._last_ik_residual = None

        p = self._fk_tip(q)
        if p is None:
            return None

        for _ in range(self.ik_max_iters):
            err = target - p
            if np.linalg.norm(err) < self.ik_tol:
                self._last_ik_residual = float(np.linalg.norm(err))
                return q.tolist()
            J = np.zeros((3, q.size))
            for i in range(q.size):
                dq = np.zeros(q.size)
                dq[i] = eps
                p2 = self._fk_tip(q + dq)
                if p2 is None:
                    return None
                J[:, i] = (p2 - p) / eps
            delta = J.T @ np.linalg.solve(J @ J.T + lam * np.eye(3), err)
            norm = np.linalg.norm(delta)
            if norm > max_step:
                delta *= max_step / norm
            q = q + delta
            p = self._fk_tip(q)
            if p is None:
                return None

        residual = float(np.linalg.norm(target - p))
        if residual < self.ik_accept_tol:
            self._last_ik_residual = residual
            return q.tolist()
        return None

    def _move_to_xyz(self, target_xyz):
        """target_xyz(base_frame) 로 analytic IK 계산 → /arm_controller/joint_trajectory 직접 발행."""
        q_current = self._current_arm_joint_positions()
        solution = self._solve_position_ik(target_xyz, q_current)
        if solution is None:
            self.get_logger().warn(f'analytic IK 실패 — 목표 도달 불가: {target_xyz}')
            # ⚠️ "도달 불가" 라고만 하면 목표가 먼 줄 오해한다. 실제로 자주 겪는 원인은
            # **출발 자세가 리밋 밖**인 것이다 — 토크가 꺼진 동안 팔이 처졌다가 그 자세로
            # 다시 잠기면(2026-08-12 실기) 야코비안 반복이 clamp 에 막혀 한 발도 못
            # 나간다. 목표가 예전에 성공한 지점보다 가까운데도 실패한다.
            # 원인을 여기서 갈라준다 — 이 경우 해법은 캘리브가 아니라 stow(home 복귀)다.
            out = []
            for name, q in zip(ARM_JOINT_NAMES, q_current):
                bounds = joint_limits.get_limits(name)
                if bounds is None:
                    continue
                lo, hi = bounds
                if q < lo - 1e-3:
                    out.append(f'{name}={q:+.3f}<하한{lo:+.3f}')
                elif q > hi + 1e-3:
                    out.append(f'{name}={q:+.3f}>상한{hi:+.3f}')
            if out:
                self.get_logger().error(
                    '출발 자세가 관절 리밋 밖입니다: ' + ', '.join(out)
                    + ' — 목표가 먼 게 아니라 여기서 IK 가 막힙니다. '
                      'stow(STOW_REQUEST)로 home 복귀 후 다시 시도하세요.')
            return False
        self._publish_joint_trajectory(solution, q_current)
        residual = getattr(self, '_last_ik_residual', None)
        # 잔차가 ik_tol 을 넘으면 "수렴 실패했지만 수용 범위" 라는 뜻 — 파지가 그만큼
        # 빗나가므로 눈에 띄게 남긴다.
        if residual is not None and residual >= self.ik_tol:
            self.get_logger().warn(
                f'analytic IK: {[round(v, 3) for v in solution]} rad 로 이동 '
                f'(잔차 {residual * 100:.1f}cm — ik_tol {self.ik_tol * 100:.0f}cm 미수렴, '
                f'ik_accept_tol {self.ik_accept_tol * 100:.0f}cm 로 수용)')
        else:
            self.get_logger().info(
                f'analytic IK: {[round(v, 3) for v in solution]} rad 로 이동 '
                f'(잔차 {residual * 100:.1f}cm)' if residual is not None
                else f'analytic IK: {[round(v, 3) for v in solution]} rad 로 이동')
        return True

    def _begin_stow_move(self):
        """`stow_joint_positions` 목표로 직접 관절궤적 발행 (MoveIt 미경유).

        접힘 자세는 충돌 회피가 필요 없는 known-safe 설정값이라 가정하므로(실측 후
        캘리브 전제), ik_mode와 무관하게 항상 `_arm_traj_pub`로 직접 명령한다 —
        analytic IK를 거칠 필요가 없다(목표가 이미 관절각이지 xyz가 아님).
        """
        q_current = self._current_arm_joint_positions()
        self._publish_joint_trajectory(self.stow_joint_positions, q_current)

    def _publish_joint_trajectory(self, target_positions, q_current):
        """목표 관절각으로 단일 포인트 궤적 발행 + 모션 진행 상태 갱신 (공통 헬퍼)."""
        delta = max(abs(a - b) for a, b in zip(target_positions, q_current))
        duration = max(1.0, min(5.0, delta / max(self.arm_move_speed, 0.05)))

        traj = JointTrajectory()
        traj.joint_names = list(ARM_JOINT_NAMES)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in target_positions]
        pt.time_from_start = Duration(sec=int(duration))
        traj.points.append(pt)
        self._arm_traj_pub.publish(traj)

        self._motion_state = 'active'
        self._motion_ok = True
        self._arm_move_deadline = self.get_clock().now() + RclpyDuration(
            seconds=duration + 0.5)

    # ── 털털이 actuator ─────────────────────────

    def _set_cleaning(self, enabled):
        """별도 Dynamixel velocity actuator에 start/stop 의도를 전달한다."""
        self._cleaning_pub.publish(Bool(data=bool(enabled)))

    def _on_arm_test_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._arm_test_ok = False
            self._arm_test_state = 'done'
            return
        self._arm_test_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(
            self._on_arm_test_result)

    def _on_arm_test_result(self, future):
        wrapped = future.result()
        self._arm_test_ok = bool(wrapped.result.success)
        self._arm_test_state = 'done'
        self._arm_test_goal_handle = None

    def _on_rotate_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._rotate_ok = False
            self._rotate_state = 'done'
            return
        self._rotate_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._on_rotate_result)

    def _on_rotate_result(self, future):
        wrapped = future.result()
        self._rotate_ok = bool(wrapped.result.success)
        self._rotate_state = 'done'
        self._rotate_goal_handle = None

    def _send_gripper(self, position):
        """Send only the logical joint command; the bridge owns raw hardware."""
        if self.dry_run_mode:
            self._gripper_command_ok = True
            self._gripper_command_state = 'done'
            return
        if not self._tool_ready():
            self._gripper_command_ok = False
            self._gripper_command_state = 'done'
            return
        trajectory = JointTrajectory()
        trajectory.joint_names = list(self.tool_profile.get('joint_names', []))
        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        duration = float(self.tool_profile.get('action_time', 1.0))
        point.time_from_start = Duration(
            sec=int(duration), nanosec=int((duration % 1.0) * 1e9))
        trajectory.points.append(point)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        self._gripper_command_state = 'active'
        self._gripper_command_ok = False
        self._gripper.send_goal_async(goal).add_done_callback(
            self._on_gripper_goal_response)

    def _on_gripper_goal_response(self, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self._gripper_command_state = 'done'
                return
            handle.get_result_async().add_done_callback(self._on_gripper_result)
        except Exception:
            self._gripper_command_state = 'done'

    def _on_gripper_result(self, future):
        try:
            wrapped = future.result()
            self._gripper_command_ok = (
                wrapped.result.error_code ==
                FollowJointTrajectory.Result.SUCCESSFUL)
        except Exception:
            self._gripper_command_ok = False
        self._gripper_command_state = 'done'

    def _tool_effort(self):
        joints = self.tool_profile.get('joint_names', [])
        return max((self._joint_effort.get(name, 0.0) for name in joints),
                   default=0.0)

    def _publish_task_result(self, mission_id, success, state, reason):
        msg = TaskResult()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mission_id = mission_id
        msg.success = success
        msg.state = state
        msg.reason = reason
        self.pub_task_result.publish(msg)

    # ── 공통 ───────────────────────────────────

    def _elapsed(self):
        return (self.get_clock().now() - self._state_enter_t).nanoseconds * 1e-9

    def _gripper_reported(self):
        """그리퍼 관절이 /joint_states 에 **존재하는가**.

        존재하지 않는 것과 effort 가 0 인 것은 원인이 완전히 다른데 `_gripper_effort()`
        는 둘 다 0.0 으로 뭉갠다. 2026-08-12 실기에서 브릿지 둘이 같은 시리얼 포트를
        두드려 그리퍼 초기화만 `result=-3002` 로 실패했고(팔 축은 성공), 그리퍼가
        SyncRead 에서 빠져 /joint_states 에 아예 안 실렸다. 그때 FSM 메시지가
        "grasp effort 0.0 below threshold" 라 **약하게 물었다**로 읽혀서 파지력·
        캘리브를 한참 의심했다. 둘을 갈라서 보고한다.
        """
        return self.gripper_joints[0] in self._joint_effort

    def _on_grasp_failure(self):
        """향후 REGRASP 정책/상태를 추가하기 위한 단일 확장 지점."""
        self._fail(
            f'grasp effort {self._gripper_effort():.1f} below threshold '
            f'{self.grasp_thresh:.1f}')

    def _fail(self, reason):
        self.get_logger().error(reason)
        self._cancel_arm_motion()
        self._cancel_arm_test_motion()
        self._cancel_end_effector_motion()
        self._transition(State.FAILED)

    def _transition(self, new_state):
        pending_status = (self._pending_arrival.status
                          if self._pending_arrival is not None else None)
        arrival_mission_id = (self._pending_arrival.mission_id
                              if self._pending_arrival is not None else None)
        self.get_logger().info(
            f'{self.state.name} → {new_state.name}; '
            f'current_state={self.state.name}, chassis_mode={self._chassis_mode!r}, '
            f'pending_arrival_status={pending_status!r}, '
            f'arrival_mission_id={arrival_mission_id!r}, '
            f'current_mission_id={self.mission_id}, '
            f'last_completed_mission_id={self._last_completed_mission_id!r}')
        if new_state == State.STOWED_LOCKED and self.state != State.STOWED_LOCKED:
            self._last_completed_mission_id = self.mission_id
        self.state = new_state
        self._state_enter_t = self.get_clock().now()
        self._cleaning_command_sent = False
        self._stow_move_sent = False
        if new_state not in (State.GRASP, State.RELEASE):
            self._gripper_command_state = 'idle'

    def _set_status(self, status):
        """발행할 현재 상태를 갱신한다. 실제 발행은 _publish_heartbeat 가 전담한다.

        여기서 직접 publish 하지 말 것 — 발행 경로가 둘이 되면 stamp 순서가 뒤집힐 수
        있고, 파워트레인은 stamp 역행을 영구 latch 로 처벌한다(contract.py 참고).
        """
        self._status = status

    def _publish_heartbeat(self):
        """계약 heartbeat — 현재 상태를 10Hz 로 발행. **유일한 /arm_status 발행 지점.**"""
        msg = ArmStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mission_id = self.mission_id
        msg.status = self._status
        self.pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmFsmNode()
    # heartbeat 타이머가 _tick(analytic IK 의 FK 블로킹 대기)에 굶지 않도록 멀티스레드.
    executor = MultiThreadedExecutor()
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


if __name__ == '__main__':
    main()
