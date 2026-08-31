"""Static contract tests for the GUI ROS frontend."""

from pathlib import Path
import os
from types import SimpleNamespace

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')


ROOT = Path(__file__).parents[1] / 'robot_manual_gui'


def test_gui_never_imports_dynamixel_sdk():
    source = ''.join(path.read_text(encoding='utf-8')
                     for path in ROOT.glob('*.py'))
    assert 'dynamixel_sdk' not in source
    assert 'write1Byte' not in source
    assert 'write2Byte' not in source
    assert 'write4Byte' not in source


def test_gui_uses_existing_control_interfaces():
    source = (ROOT / 'ros_interface.py').read_text(encoding='utf-8')
    for interface in (
            '/arm_controller/joint_trajectory',
            '/gripper_controller/follow_joint_trajectory',
            '/cleaning/enable', '/tool/emergency_stop', '/tool/detached'):
        assert interface in source


def test_mode_status_does_not_overwrite_pending_operator_request():
    source = (ROOT / 'main_window.py').read_text(encoding='utf-8')
    assert 'self.mode_combo.setCurrentText(mode)' not in source


def test_end_effector_scope_blocks_arm_publish_path():
    ros_source = (ROOT / 'ros_interface.py').read_text(encoding='utf-8')
    window_source = (ROOT / 'main_window.py').read_text(encoding='utf-8')
    assert "self.control_scope == 'END_EFFECTOR_ONLY'" in ros_source
    assert 'manual and not end_effector_only' in window_source
    assert '제어 / 시험 범위:' in window_source


def _window(scope, single_motor=False):
    from PyQt5.QtWidgets import QApplication
    from robot_manual_gui.main_window import ManualMainWindow
    from robot_manual_gui.ros_interface import GuiSignals

    app = QApplication.instance() or QApplication([])
    goals = []
    stops = []
    ticks = []
    node = SimpleNamespace(
        control_scope=scope, selected_tool='dual_motor_gripper',
        dual_single_motor_test_mode=single_motor,
        single_motor_endpoints={'open_tick': None, 'close_tick': None},
        positions={}, efforts={}, gripper_busy=False,
        request_mode=lambda _mode: None, jog_arm=lambda *_args: None,
        command_arm=lambda *_args: None,
        command_gripper=lambda position: (goals.append(position) or True),
        command_single_motor_tick=lambda target: (ticks.append(target) or True),
        command_single_motor_preset=lambda _kind: True,
        stop_gripper=lambda: stops.append(True),
        command_cleaner=lambda *_args: None,
        emergency_stop=lambda: None, tool_detached=lambda: None)
    profile = {
        'calibrated': True, 'actuator_ids': [3, 4],
        'open_position': 1.0, 'close_position': 0.0,
        'safe_min_tick': -526, 'safe_max_tick': 2384,
        'motor_endpoints': {
            3: {'open': 1056, 'close': -526},
            4: {'open': 2384, 'close': 839}}}
    window = ManualMainWindow(node, GuiSignals(), profile, False)
    window._test_stops = stops
    window._test_ticks = ticks
    return app, window, goals


def _ready_status(scope):
    return {
        'control_scope': scope, 'tool_type': 'dual_motor_gripper',
        'profile_valid': True, 'calibrated': True,
        'actuators_discovered': True, 'motion_allowed': True,
        'read_only': False, 'emergency_stop': False, 'tool_detached': False,
        'bridge_connected': True,
        'actuators': [
            {'id': 3, 'online': True, 'position': 265, 'effort': 10},
            {'id': 4, 'online': True, 'position': 1612, 'effort': 10}]}


def test_end_effector_scope_enables_only_tool_controls():
    _app, window, _goals = _window('END_EFFECTOR_ONLY')
    window._update_tool_status(_ready_status('END_EFFECTOR_ONLY'))
    window._update_mode('MANUAL')
    assert window.open_button.isEnabled()
    assert window.close_button.isEnabled()
    assert window.tool_stop.isEnabled()
    assert not any(widget.isEnabled() for widget in window.arm_buttons)
    window.close()


def test_full_robot_preserves_arm_feedback_gate():
    _app, window, _goals = _window('FULL_ROBOT')
    window._update_tool_status(_ready_status('FULL_ROBOT'))
    window._update_mode('MANUAL')
    assert not any(widget.isEnabled() for widget in window.arm_buttons)
    window.seen_arm_joints.add('arm_joint_1')
    window._refresh_buttons()
    assert all(widget.isEnabled() for widget in window.arm_widgets['arm_joint_1'])
    assert not any(widget.isEnabled()
                   for widget in window.arm_widgets['arm_joint_2'])
    window.close()


def test_jog_interpolates_both_motors_and_busy_blocks_queue():
    _app, window, goals = _window('END_EFFECTOR_ONLY')
    window._update_tool_status(_ready_status('END_EFFECTOR_ONLY'))
    window._update_mode('MANUAL')
    window._jog_gripper(1)
    assert len(goals) == 1
    assert window.gripper_target_ticks == {3: 270, 4: 1617}
    window.gripper_busy = True
    window._jog_gripper(1)
    assert len(goals) == 1
    window.close()


def test_keyboard_controls_gripper_even_when_a_button_has_focus():
    from PyQt5.QtCore import Qt
    from PyQt5.QtTest import QTest

    _app, window, goals = _window('END_EFFECTOR_ONLY')
    window._update_tool_status(_ready_status('END_EFFECTOR_ONLY'))
    window._update_mode('MANUAL')
    window.open_button.setFocus()

    QTest.keyClick(window.open_button, Qt.Key_Right)
    assert len(goals) == 1
    assert window.gripper_target_ticks == {3: 270, 4: 1617}

    QTest.keyClick(window.open_button, Qt.Key_Space)
    assert window._test_stops == [True]
    window.close()


def test_single_motor_key_hold_repeats_id3_and_release_stops():
    from PyQt5.QtCore import Qt
    from PyQt5.QtTest import QTest

    _app, window, _goals = _window('END_EFFECTOR_ONLY', single_motor=True)
    status = _ready_status('END_EFFECTOR_ONLY')
    status.update({
        'dual_single_motor_test_mode': True,
        'torque_enabled_ids': [3],
    })
    window._update_tool_status(status)
    window._update_mode('MANUAL')
    window.open_button.setFocus()

    QTest.keyPress(window.open_button, Qt.Key_W)
    QTest.qWait(340)
    QTest.keyRelease(window.open_button, Qt.Key_W)
    sent_at_release = len(window._test_ticks)
    assert sent_at_release >= 3
    assert window._test_ticks[:3] == [270, 270, 270]
    QTest.qWait(220)
    assert len(window._test_ticks) == sent_at_release
    assert not window.motor_repeat.isActive()
    QTest.keyClick(window.open_button, Qt.Key_Right)
    assert len(window._test_ticks) == sent_at_release
    window.close()


def test_single_motor_qw_are_application_global_and_ignore_qt_auto_repeat():
    from PyQt5.QtCore import QEvent, Qt
    from PyQt5.QtGui import QKeyEvent
    from PyQt5.QtTest import QTest

    app, window, _goals = _window('END_EFFECTOR_ONLY', single_motor=True)
    status = _ready_status('END_EFFECTOR_ONLY')
    status.update({
        'dual_single_motor_test_mode': True,
        'torque_enabled_ids': [3],
    })
    window._update_tool_status(status)
    window._update_mode('MANUAL')
    # Q/W must work even while an editing widget owns focus.
    window.tool_combo.setFocus()
    QTest.keyPress(window.tool_combo, Qt.Key_Q)
    assert window._test_ticks == [260]
    assert window.motor_repeat.isActive()

    # Qt's synthetic repeat press/release pair must neither add an immediate
    # command nor stop the application-owned 150 ms timer.
    count = len(window._test_ticks)
    app.sendEvent(window.tool_combo, QKeyEvent(
        QEvent.KeyPress, Qt.Key_Q, Qt.NoModifier, '', True, 1))
    app.sendEvent(window.tool_combo, QKeyEvent(
        QEvent.KeyRelease, Qt.Key_Q, Qt.NoModifier, '', True, 1))
    assert len(window._test_ticks) == count
    assert window.motor_repeat.isActive()

    QTest.qWait(170)
    assert len(window._test_ticks) > count
    QTest.keyRelease(window.tool_combo, Qt.Key_Q)
    assert not window.motor_repeat.isActive()
    window.close()


def test_single_motor_mouse_button_uses_same_hold_jog_path():
    from PyQt5.QtCore import Qt
    from PyQt5.QtTest import QTest

    _app, window, _goals = _window('END_EFFECTOR_ONLY', single_motor=True)
    status = _ready_status('END_EFFECTOR_ONLY')
    status.update({
        'dual_single_motor_test_mode': True,
        'torque_enabled_ids': [3],
    })
    window._update_tool_status(status)
    window._update_mode('MANUAL')

    QTest.mousePress(window.jog_open, Qt.LeftButton)
    assert window._test_ticks == [270]
    assert window.motor_repeat.isActive()
    QTest.qWait(170)
    QTest.mouseRelease(window.jog_open, Qt.LeftButton)
    assert len(window._test_ticks) >= 2
    assert not window.motor_repeat.isActive()
    window.close()


def test_single_motor_outside_profile_only_recovers_in_small_steps():
    _app, window, _goals = _window('END_EFFECTOR_ONLY', single_motor=True)
    window.node.single_motor_endpoints = {
        'open_tick': 1056, 'close_tick': -526}
    status = _ready_status('END_EFFECTOR_ONLY')
    status['actuators'][0]['position'] = -810
    status.update({
        'dual_single_motor_test_mode': True,
        'torque_enabled_ids': [3],
    })
    window._update_tool_status(status)
    window._update_mode('MANUAL')

    window._jog_single_id3(-1)
    assert window._test_ticks == []
    window._jog_single_id3(1)
    assert window._test_ticks == [-805]
    window.close()


def test_single_motor_mode_ignores_id4_normalized_position_divergence():
    _app, window, _goals = _window('END_EFFECTOR_ONLY', single_motor=True)
    status = _ready_status('END_EFFECTOR_ONLY')
    status['actuators'][0]['position'] = 0
    status['actuators'][1]['position'] = 2384
    status.update({
        'dual_single_motor_test_mode': True,
        'torque_enabled_ids': [3],
    })
    window._update_tool_status(status)
    window._update_mode('MANUAL')

    assert window._gripper_positions_synchronized()
    assert window.jog_close.isEnabled()
    assert window.jog_open.isEnabled()
    assert '정규화 편차 검사 미적용' in window.gripper_feedback_label.text()
    assert '차단' not in window.gripper_busy_label.text()
    window.close()
