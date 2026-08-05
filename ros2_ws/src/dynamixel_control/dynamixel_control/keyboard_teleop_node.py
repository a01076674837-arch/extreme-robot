#!/usr/bin/env python3
"""키보드 원격조종 프론트엔드.

터미널 키 입력을 표준 control_msgs/JointJog(+ 이산 명령 String)로 변환해
teleop_core 로 보낸다. 시리얼/모터 로직은 전혀 없다 — 순수 입력 어댑터.
게임패드/네트워크 프론트엔드도 같은 토픽으로 발행하면 그대로 대체된다.

키맵
  1 2 3 4 5   : 관절 선택 (설정된 관절만 유효)
  ↑ / w       : 선택 관절 + 방향 jog
  ↓ / s       : 선택 관절 - 방향 jog
  [ / ]       : jog 스텝 배율 감소 / 증가
  h           : 홈(전 관절 center) 복귀
  space       : 정지(현재 위치 고정)
  q / Ctrl-C  : 종료

* 반드시 자체 터미널에서 `ros2 run dynamixel_control keyboard_teleop` 로 실행할 것
  (launch 안에 넣으면 stdin 포커스를 못 받는다).
"""

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from control_msgs.msg import JointJog
from dynamixel_control.arm_hardware import ARM_JOINT_NAMES


HELP = __doc__


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_teleop")

        self.declare_parameter("joint_names", ARM_JOINT_NAMES)
        self.declare_parameter("step_scale", 1.0)   # displacement 크기 배율
        self.declare_parameter("step_scale_delta", 0.5)

        self.joint_names = list(self.get_parameter("joint_names").value)
        self.step_scale = float(self.get_parameter("step_scale").value)
        self.step_scale_delta = float(self.get_parameter("step_scale_delta").value)

        self.selected = 0  # 선택된 관절 인덱스

        self.jog_pub = self.create_publisher(JointJog, "/arm/teleop_jog", 10)
        self.cmd_pub = self.create_publisher(String, "/arm/teleop_cmd", 10)

    # ------------------------------------------------------------------ publish
    def _publish_jog(self, sign):
        name = self.joint_names[self.selected]
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = [name]
        msg.displacements = [sign * self.step_scale]
        self.jog_pub.publish(msg)
        self._status(f"jog {name} {'+' if sign > 0 else '-'} (scale {self.step_scale:.2f})")

    def _publish_cmd(self, cmd):
        self.cmd_pub.publish(String(data=cmd))
        self._status(f"cmd: {cmd}")

    def _status(self, extra=""):
        name = self.joint_names[self.selected]
        sys.stdout.write(
            f"\r[선택: {name} ({self.selected + 1}/{len(self.joint_names)})] "
            f"scale={self.step_scale:.2f}  {extra}      "
        )
        sys.stdout.flush()

    # ------------------------------------------------------------------ input
    def handle_key(self, key):
        """키 하나 처리. 종료하려면 False 반환."""
        if key in ("q", "\x03"):  # q or Ctrl-C
            return False
        elif key in ("\x1b[A", "w"):        # ↑ / w
            self._publish_jog(+1.0)
        elif key in ("\x1b[B", "s"):        # ↓ / s
            self._publish_jog(-1.0)
        elif key == "[":
            self.step_scale = max(0.1, self.step_scale - self.step_scale_delta)
            self._status("scale-")
        elif key == "]":
            self.step_scale = self.step_scale + self.step_scale_delta
            self._status("scale+")
        elif key == "h":
            self._publish_cmd("home")
        elif key == " ":
            self._publish_cmd("stop")
        elif key in ("1", "2", "3", "4", "5"):
            idx = int(key) - 1
            if idx < len(self.joint_names):
                self.selected = idx
                self._status("선택 변경")
            else:
                self._status(f"관절 {key} 없음")
        return True


def read_key(timeout=0.1):
    """블로킹 없이 키 한 번 읽기. 화살표(ESC 시퀀스)는 3바이트로 묶어 반환."""
    if select.select([sys.stdin], [], [], timeout)[0]:
        ch = sys.stdin.read(1)
        if ch == "\x1b":  # ESC 시퀀스 (화살표 등)
            if select.select([sys.stdin], [], [], 0.001)[0]:
                ch += sys.stdin.read(2)
        return ch
    return None


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()

    print(HELP)
    node._status("준비됨")

    old_attr = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = read_key(timeout=0.1)
            if key is not None:
                if not node.handle_key(key):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
        print()  # 개행 정리
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
