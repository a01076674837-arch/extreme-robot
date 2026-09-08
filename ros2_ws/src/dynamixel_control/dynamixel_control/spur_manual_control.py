"""Bounded ID5 manual routing, independent of dual gripper control."""
import time


class SpurManualControl:
    def __init__(self, bridge):
        self.bridge = bridge
        self.deadline = None

    def command(self, command, delta_deg=0):
        b = self.bridge
        if b.tool_type != 'spur_1motor_gripper' or b.tool_ids != [5] or b.read_only:
            raise RuntimeError('ID5 manual scope/read-only gate')
        if b.control_scope != 'END_EFFECTOR_ONLY':
            raise RuntimeError('ID5 requires end-effector scope')
        if command == 'manual_disable':
            self.deadline = None
            b.tool_fsm.disable()
            return
        if command == 'manual_enable':
            if (b.control_mode != 'MANUAL' or b.emergency_stop_active or b.tool_detached
                    or not b.tool_selection.valid or b.read_hardware_error(5) != 0):
                raise RuntimeError('ID5 enable readiness gate')
            b.tool_fsm.startup()
            if b.tool_fsm.state.name != 'READY':
                raise RuntimeError('ID5 FSM startup not READY')
            b.set_torque(5, True)
            return
        if command == 'manual_hold':
            self.deadline = None
            if b.read_torque(5) == 1:
                b.goal_position(5, b.read_position(5))
            return
        if (b.control_mode != 'MANUAL' or b.emergency_stop_active or b.tool_detached
                or not b.tool_selection.valid or not b.tool_profile.get('calibrated')
                or b.tool_fsm.state.name not in ('READY', 'OPEN', 'CLOSED')
                or b.read_hardware_error(5) != 0 or b.read_torque(5) != 1):
            raise RuntimeError('ID5 manual readiness gate')
        targets = b.tool_fsm._validated_targets()
        current = b.read_position(5)
        low, high = sorted(targets.values())
        if not low <= current <= high:
            raise RuntimeError('ID5 position outside calibrated endpoints')
        if command in ('manual_open', 'manual_close'):
            endpoint = targets['open' if command == 'manual_open' else 'close']
            target = current + max(-6, min(6, endpoint - current))
        elif command == 'manual_step':
            if float(delta_deg) not in (-1.0, -0.5, 0.5, 1.0):
                raise RuntimeError('unsupported ID5 manual step')
            target = current + round(float(delta_deg) * 4096 / 360)
        else:
            raise RuntimeError('unsupported ID5 manual command')
        b.goal_position(5, max(low, min(high, target)))
        self.deadline = time.monotonic() + 0.3

    def watchdog(self):
        if self.deadline is None or time.monotonic() < self.deadline:
            return
        self.deadline = None
        if self.bridge.tool_type == 'spur_1motor_gripper':
            self.command('manual_hold')
