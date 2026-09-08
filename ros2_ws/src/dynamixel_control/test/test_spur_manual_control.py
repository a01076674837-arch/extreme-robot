from types import SimpleNamespace
import pytest
from dynamixel_control.spur_manual_control import SpurManualControl


def make_control(open_tick=100, close_tick=200):
    b = SimpleNamespace(tool_type='spur_1motor_gripper', tool_ids=[5], read_only=False,
        control_scope='END_EFFECTOR_ONLY', control_mode='MANUAL', emergency_stop_active=False,
        tool_detached=False, tool_selection=SimpleNamespace(valid=True), tool_profile={'calibrated': True},
        tool_fsm=SimpleNamespace(state=SimpleNamespace(name='READY'),
            _validated_targets=lambda: {'open': open_tick, 'close': close_tick}), position=150, torque=1, error=0)
    b.read_position = lambda _: b.position
    b.read_torque = lambda _: b.torque
    b.read_hardware_error = lambda _: b.error
    b.goal_position = lambda _, tick: setattr(b, 'position', tick)
    return b, SpurManualControl(b)


@pytest.mark.parametrize('endpoints', [(100, 200), (200, 100)])
def test_endpoint_direction_clamp_release_and_watchdog(endpoints):
    b, c = make_control(*endpoints)
    for command, target in [('manual_open', endpoints[0]), ('manual_close', endpoints[1])]:
        for _ in range(30):
            c.command(command)
            assert 100 <= b.position <= 200
        assert b.position == target
    b.position = 155
    c.command('manual_hold')
    assert b.position == 155 and c.deadline is None
    c.command('manual_open')
    b.position = 153
    c.deadline = 0
    c.watchdog()
    assert b.position == 153 and c.deadline is None


@pytest.mark.parametrize('field,value', [('control_mode', 'FSM'), ('read_only', True),
    ('emergency_stop_active', True), ('tool_detached', True), ('torque', 0), ('error', 1),
    ('tool_type', 'dual_motor_gripper')])
def test_gate_blocks_without_writes(field, value):
    b, c = make_control()
    setattr(b, field, value)
    with pytest.raises(RuntimeError):
        c.command('manual_open')
    assert b.position == 150
