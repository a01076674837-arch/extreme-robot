#!/usr/bin/env python3
"""원격조종 공통 코어 노드.

입력 장치(키보드/게임패드/네트워크 등)와 무관하게 동작하는 '두뇌'.
프론트엔드는 표준 control_msgs/JointJog 로 '어느 관절을 얼마나' 만 알려주고,
여기서 현재값 기준 적분·소프트리밋·데드맨·rad↔tick 변환을 모두 처리한 뒤
position_node 가 이해하는 /dynamixel/goal_position ([id, tick]) 로 내보낸다.

토픽
  구독 /arm/teleop_jog   (control_msgs/JointJog)   : 연속 이동 의도
         - displacements : 즉시 위치 증분 (단위: jog_step_rad 배수). 키보드가 쓴다.
         - velocities    : **rad/s**. 타이머가 적분한다. 조이스틱 같은 아날로그 입력이 쓴다.
  구독 /arm/teleop_cmd   (std_msgs/String)         : 이산 명령 "home" | "stop"
  구독 /joint_states     (sensor_msgs/JointState)  : 현재 각도(시작 튐 방지·stop 기준)
  발행 /dynamixel/goal_position (std_msgs/Int32MultiArray) [id, tick]

프론트엔드가 무엇이든 이 노드는 그대로 재사용된다.

⚠️ **velocity 프론트엔드가 지켜야 할 것** — on_jog 는 **메시지에 실린 관절만** velocity 를
   갱신한다. 움직이는 관절만 골라 보내면 **놓은 관절이 마지막 속도로 계속 돈다.**
   매 발행마다 **전 관절을 joint_names 에 싣고 velocity 를 0 포함해 채울 것.**
   (deadman_timeout_s 가 최후의 안전망이지만, 그건 입력이 완전히 끊겼을 때만 작동한다.)
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String
from sensor_msgs.msg import JointState
from control_msgs.msg import JointJog
from dynamixel_control.arm_hardware import (
    ARM_CENTERS, ARM_DIRECTIONS, ARM_JOINT_NAMES, ARM_LIMIT_ENABLED,
    ARM_MAX_RADS, ARM_MIN_RADS, ARM_MOTOR_IDS,
)


TICKS_PER_RAD = 4096.0 / (2.0 * math.pi)
DXL_MIN_TICK = 0
DXL_MAX_TICK = 4095

DEFAULT_JOINT_NAMES = ARM_JOINT_NAMES
DEFAULT_MOTOR_IDS = ARM_MOTOR_IDS
DEFAULT_CENTERS = ARM_CENTERS
DEFAULT_DIRECTIONS = ARM_DIRECTIONS
DEFAULT_LIMIT_ENABLED = ARM_LIMIT_ENABLED
DEFAULT_MIN_RADS = ARM_MIN_RADS
DEFAULT_MAX_RADS = ARM_MAX_RADS


class TeleopCore(Node):
    def __init__(self):
        super().__init__("teleop_core")

        # --- 관절 ↔ 모터 매핑 ---
        self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)
        self.declare_parameter("motor_ids", DEFAULT_MOTOR_IDS)
        self.declare_parameter("centers", DEFAULT_CENTERS)
        self.declare_parameter("directions", DEFAULT_DIRECTIONS)

        # --- 동작 파라미터 ---
        self.declare_parameter("jog_step_rad", 0.05)      # displacement=±1 당 이동량
        self.declare_parameter("joint_min_rad", -math.pi)  # 호환용 fallback 공통 리밋
        self.declare_parameter("joint_max_rad", math.pi)
        self.declare_parameter("joint_limit_enabled", DEFAULT_LIMIT_ENABLED)
        self.declare_parameter("joint_min_rads", DEFAULT_MIN_RADS)
        self.declare_parameter("joint_max_rads", DEFAULT_MAX_RADS)
        self.declare_parameter("deadman_timeout_s", 0.5)  # 이 시간 넘게 입력 없으면 velocity 적분 정지
        # velocity 적분 주기 = 목표 갱신 주기. 낮으면 한 번에 크게 뛰어 서보가
        # 매번 프로파일을 새로 그리며 틱틱 끊긴다 — 잘게 자주 보내는 편이 매끄럽다.
        self.declare_parameter("publish_rate_hz", 50.0)
        # JointJog.velocities 의 안전 상한 [rad/s]. 프론트엔드 버그로 큰 값이 들어와도
        # 팔이 튀지 않도록 on_jog 에서 clamp 한다.
        self.declare_parameter("max_vel_rad_s", 1.0)
        # 하드웨어 없이 RViz 디버그: 목표값을 그대로 /joint_states 로도 발행(open-loop).
        # 실서보 사용 시엔 position_node 가 실제 /joint_states 를 내므로 반드시 false.
        self.declare_parameter("publish_sim_joint_states", False)

        self.joint_names = list(self.get_parameter("joint_names").value)
        motor_ids = list(self.get_parameter("motor_ids").value)
        centers = list(self.get_parameter("centers").value)
        directions = list(self.get_parameter("directions").value)
        limit_enabled = list(self.get_parameter("joint_limit_enabled").value)
        min_rads = list(self.get_parameter("joint_min_rads").value)
        max_rads = list(self.get_parameter("joint_max_rads").value)

        if not (len(self.joint_names) == len(motor_ids) == len(centers) == len(directions)):
            raise RuntimeError(
                "joint_names/motor_ids/centers/directions 길이가 서로 다릅니다"
            )

        self.cfg = {
            name: {"id": int(motor_ids[i]), "center": int(centers[i]),
                   "direction": int(directions[i])}
            for i, name in enumerate(self.joint_names)
        }

        self.jog_step_rad = float(self.get_parameter("jog_step_rad").value)
        self.joint_min_rad = float(self.get_parameter("joint_min_rad").value)
        self.joint_max_rad = float(self.get_parameter("joint_max_rad").value)
        self.joint_limits = self._build_joint_limits(limit_enabled, min_rads, max_rads)
        self.deadman_timeout_s = float(self.get_parameter("deadman_timeout_s").value)
        self.publish_sim = bool(self.get_parameter("publish_sim_joint_states").value)
        self.max_vel_rad_s = abs(float(self.get_parameter("max_vel_rad_s").value))
        rate = float(self.get_parameter("publish_rate_hz").value)

        # 관절별 목표 각도(rad). None = 아직 초기화 안 됨(joint_states 대기).
        self.goal_rad = {name: None for name in self.joint_names}
        self.measured_rad = {name: None for name in self.joint_names}
        # velocity 모드용 상태
        self.velocity = {name: 0.0 for name in self.joint_names}
        self.last_input_time = self.get_clock().now()

        self.sub_jog = self.create_subscription(
            JointJog, "/arm/teleop_jog", self.on_jog, 10)
        self.sub_cmd = self.create_subscription(
            String, "/arm/teleop_cmd", self.on_cmd, 10)
        self.sub_js = self.create_subscription(
            JointState, "/joint_states", self.on_joint_states, 10)

        self.pub = self.create_publisher(
            Int32MultiArray, "/dynamixel/goal_position", 10)

        # 하드웨어 없이 RViz 디버그용 sim /joint_states 발행자
        self.js_pub = None
        if self.publish_sim:
            self.js_pub = self.create_publisher(JointState, "/joint_states", 10)

        self.timer = self.create_timer(1.0 / rate, self.on_timer)

        self.get_logger().info(
            f"teleop_core started (joints={self.joint_names}, "
            f"jog_step={self.jog_step_rad} rad, limits={self._limit_summary()}, "
            f"sim_joint_states={self.publish_sim})"
        )

    # ------------------------------------------------------------------ helpers
    def _build_joint_limits(self, enabled, min_rads, max_rads):
        n = len(self.joint_names)
        if not (len(enabled) == len(min_rads) == len(max_rads) == n):
            self.get_logger().warn(
                "joint_limit_enabled/joint_min_rads/joint_max_rads 길이가 "
                "joint_names와 맞지 않아 공통 fallback 리밋을 사용합니다"
            )
            return {
                name: (True, self.joint_min_rad, self.joint_max_rad)
                for name in self.joint_names
            }

        limits = {}
        for i, name in enumerate(self.joint_names):
            lower = float(min_rads[i])
            upper = float(max_rads[i])
            if lower > upper:
                lower, upper = upper, lower
            limits[name] = (bool(enabled[i]), lower, upper)
        return limits

    def _limit_summary(self):
        parts = []
        for name in self.joint_names:
            enabled, lower, upper = self.joint_limits[name]
            parts.append(f"{name}=[{lower:.3f},{upper:.3f}]" if enabled else f"{name}=off")
        return ", ".join(parts)

    def _clamp_rad(self, name, rad):
        enabled, lower, upper = self.joint_limits[name]
        if not enabled:
            return rad
        return max(lower, min(upper, rad))

    def _rad_to_tick(self, name, rad):
        c = self.cfg[name]
        tick = int(round(c["center"] + c["direction"] * rad * TICKS_PER_RAD))
        return max(DXL_MIN_TICK, min(DXL_MAX_TICK, tick))

    def _ensure_goal(self, name):
        """목표가 초기화 안 됐으면 측정값(없으면 0.0=center)으로 채운다 — 시작 튐 방지."""
        if self.goal_rad[name] is None:
            base = self.measured_rad[name]
            self.goal_rad[name] = self._clamp_rad(name, 0.0 if base is None else base)

    def _publish_goals(self, names):
        """지정 관절들의 현재 목표를 [id, tick] 로 발행."""
        for name in names:
            if self.goal_rad[name] is None:
                continue
            tick = self._rad_to_tick(name, self.goal_rad[name])
            msg = Int32MultiArray()
            msg.data = [self.cfg[name]["id"], tick]
            self.pub.publish(msg)

    # ------------------------------------------------------------------ callbacks
    def on_joint_states(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in self.measured_rad:
                self.measured_rad[name] = float(pos)

    def on_jog(self, msg):
        """연속 이동 의도. displacement 는 즉시 위치 증분, velocity 는 타이머가 적분."""
        self.last_input_time = self.get_clock().now()
        changed = []

        disp = list(msg.displacements)
        vel = list(msg.velocities)
        for i, name in enumerate(msg.joint_names):
            if name not in self.cfg:
                self.get_logger().warn(f"알 수 없는 관절: {name}")
                continue
            self._ensure_goal(name)
            if i < len(disp) and disp[i] != 0.0:
                self.goal_rad[name] = self._clamp_rad(
                    name, self.goal_rad[name] + disp[i] * self.jog_step_rad)
                changed.append(name)
            # velocity 는 타이머에서 적분 (deadman 적용). 단위는 **rad/s** (JointJog 표준).
            v = vel[i] if i < len(vel) else 0.0
            self.velocity[name] = max(-self.max_vel_rad_s, min(self.max_vel_rad_s, v))

        if changed:
            self._publish_goals(changed)

    def on_cmd(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == "home":
            for name in self.joint_names:
                self._ensure_goal(name)
                self.goal_rad[name] = self._clamp_rad(name, 0.0)
            self.velocity = {n: 0.0 for n in self.joint_names}
            self._publish_goals(self.joint_names)
            self.get_logger().info("home: 전 관절 center 복귀")
        elif cmd == "stop":
            # 현재 측정 위치로 목표 고정 → 그 자리에서 정지
            for name in self.joint_names:
                if self.measured_rad[name] is not None:
                    self.goal_rad[name] = self._clamp_rad(name, self.measured_rad[name])
                else:
                    self._ensure_goal(name)
            self.velocity = {n: 0.0 for n in self.joint_names}
            self._publish_goals(self.joint_names)
            self.get_logger().info("stop: 현재 위치에서 정지")
        else:
            self.get_logger().warn(f"알 수 없는 명령: {msg.data}")

    def _publish_sim_joint_states(self):
        """목표 각도를 그대로 /joint_states 로 발행(open-loop). RViz 디버그 전용."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for name in self.joint_names:
            rad = self.goal_rad[name]
            msg.name.append(name)
            msg.position.append(0.0 if rad is None else rad)
        self.js_pub.publish(msg)

    def on_timer(self):
        """sim joint_states 발행(옵션) + velocity 모드 적분 + 데드맨."""
        # RViz 디버그: 목표 포즈를 항상 내보내 TF 가 갱신되게 한다(입력 없어도).
        if self.js_pub is not None:
            self._publish_sim_joint_states()

        now = self.get_clock().now()
        dt = 1.0 / max(1e-3, float(self.get_parameter("publish_rate_hz").value))
        elapsed = (now - self.last_input_time).nanoseconds * 1e-9
        if elapsed > self.deadman_timeout_s:
            return  # 입력 끊김 → velocity 적분 중단(현재 목표 유지)

        moving = []
        for name in self.joint_names:
            v = self.velocity.get(name, 0.0)
            if v != 0.0:
                self._ensure_goal(name)
                # velocities 는 rad/s → dt 만 곱한다. (jog_step_rad 는 displacement 전용)
                self.goal_rad[name] = self._clamp_rad(name, self.goal_rad[name] + v * dt)
                moving.append(name)
        if moving:
            self._publish_goals(moving)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopCore()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
