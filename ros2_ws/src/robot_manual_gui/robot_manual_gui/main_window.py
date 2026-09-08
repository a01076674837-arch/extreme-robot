"""PyQt5 widgets for manual robot validation."""

import time
import json

from PyQt5.QtCore import QEvent, QLibraryInfo, QProcess, QTimer, QTranslator, Qt
from PyQt5.QtWidgets import (
    QAbstractSpinBox, QApplication, QComboBox, QDoubleSpinBox, QFormLayout,
    QGridLayout,
    QGroupBox, QHBoxLayout, QLabel, QMainWindow, QMessageBox, QPushButton,
    QLineEdit, QTableWidget, QTableWidgetItem, QTextEdit, QVBoxLayout, QWidget)

from robot_manual_gui.ros_interface import ARM_JOINTS
from robot_manual_gui.korean_text import ko
from dynamixel_control.tool_manager import ToolManager


TRUE_STYLE = 'color: #0b7a25; font-weight: bold;'
FALSE_STYLE = 'color: #b00020; font-weight: bold;'
ESTOP_STYLE = 'background: #b00020; color: white; font-size: 20px; font-weight: bold;'


class ManualMainWindow(QMainWindow):
    """Hardware-test dashboard backed exclusively by ROS interfaces."""

    def __init__(self, ros_node, signals, profile, mock_mode=False):
        super().__init__()
        self._qt_korean = QTranslator(self)
        self._qt_korean.load('qtbase_ko', QLibraryInfo.location(QLibraryInfo.TranslationsPath))
        QApplication.instance().installTranslator(self._qt_korean)
        self.node = ros_node
        self.signals = signals
        self.profile = profile
        self.mock_mode = mock_mode
        self.tool_status = {}
        self.pending_tool_change = None
        self.spur_hold_command = None
        self.spur_hold_timer = QTimer(self)
        self.spur_hold_timer.setInterval(100)
        self.spur_hold_timer.timeout.connect(self._repeat_spur_hold)
        self.fsm_state = 'UNKNOWN'
        self.control_mode = 'FSM'
        self.last_status_time = 0.0
        self.processes = []
        self.joint_rows = {}
        self.seen_arm_joints = set()
        self.arm_widgets = {}
        self.gripper_busy = False
        self.dual_hold_jog_active = False
        self.dual_hold_jog_direction = None
        self.gripper_target_ticks = {}
        self.spur_torque_enabled = False
        self.spur_torque_state = 'UNKNOWN'
        self.spur_endpoints = {}
        self.spur_zero_tick = None
        self.dual_calibration_buttons = []
        self.dual_calibration_step = None
        self.dual_calibration_state = None
        self.dual_start_calibration = None
        self.dual_capture_open = None
        self.dual_capture_close = None
        self.dual_validate_calibration = None
        self.dual_save_calibration = None
        self.dual_capture_label = None
        # External spur gears reverse rotation.  This is deliberately shown in
        # the GUI instead of being hidden in a raw-tick jog control.
        self.spur_output_direction = -1
        self.spur_gear_ratio = 1.0
        self.temporary_jog_safe_min = getattr(
            self.node, 'temporary_jog_safe_min', 2867)
        self.temporary_jog_safe_max = getattr(
            self.node, 'temporary_jog_safe_max', 3807)
        get_param = getattr(self.node, 'get_parameter', None)
        self.temporary_jog_mechanical_open = (
            get_param('temporary_jog_mechanical_open_tick').value
            if get_param else 2817)
        self.temporary_jog_mechanical_close = (
            get_param('temporary_jog_mechanical_close_tick').value
            if get_param else 3857)
        self.setWindowTitle(ko('Extreme Robot Manual Hardware Validation'))
        self.resize(1180, 850)
        self._build_ui()
        self._connect_signals()
        # Child widgets normally consume arrow keys for focus navigation.
        # Observe them before dispatch so hold-to-run works anywhere in this
        # window, while preserving arrow editing in input widgets.
        QApplication.instance().installEventFilter(self)
        self.watchdog = QTimer(self)
        self.watchdog.timeout.connect(self._refresh_connection)
        self.watchdog.start(500)
        self.dual_key_jog_timer = QTimer(self)
        self.dual_key_jog_timer.setInterval(100)
        self.dual_key_jog_timer.timeout.connect(self._dual_key_jog_tick)
        self.dual_key_jog_direction = 0

    def _build_ui(self):
        root = QWidget()
        outer = QVBoxLayout(root)

        self.scope_banner = QLabel(ko(f'CONTROL / TEST SCOPE: {self.node.control_scope}'))
        self.scope_banner.setAlignment(Qt.AlignCenter)
        self.scope_banner.setStyleSheet(
            'font-size: 22px; font-weight: bold; padding: 8px; '
            'background: #ffe08a; color: #202020;')
        outer.addWidget(self.scope_banner)

        safety = QHBoxLayout()
        self.estop = QPushButton(ko('EMERGENCY STOP'))
        self.estop.setMinimumHeight(62)
        self.estop.setStyleSheet(ESTOP_STYLE)
        self.estop.clicked.connect(self._estop)
        self.detach = QPushButton(ko('TOOL DETACHED'))
        self.detach.clicked.connect(self._detach)
        self.reset = QPushButton(ko('RESET E-STOP (restart required)'))
        self.reset.setEnabled(False)
        self.estop_state = QLabel(ko('E-STOP: FALSE'))
        self.estop_state.setStyleSheet(TRUE_STYLE)
        safety.addWidget(self.estop, 3)
        safety.addWidget(self.detach)
        safety.addWidget(self.reset)
        safety.addWidget(self.estop_state)
        outer.addLayout(safety)

        columns = QHBoxLayout()
        left = QVBoxLayout()
        right = QVBoxLayout()
        self.right_layout = right
        left.addWidget(self._status_group())
        left.addWidget(self._arm_group())
        right.addWidget(self._tool_selection_group())
        self.tool_control_box = self._tool_control_group()
        right.addWidget(self.tool_control_box)
        columns.addLayout(left, 3)
        columns.addLayout(right, 2)
        outer.addLayout(columns)

        self.diag = QTableWidget(0, 5)
        self.diag.setHorizontalHeaderLabels(
            [ko('ID'), ko('Joint'), ko('Position'), ko('Current/Load'), ko('Online')])
        outer.addWidget(self.diag)
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumHeight(120)
        outer.addWidget(self.log)
        self.setCentralWidget(root)

    def _status_group(self):
        box = QGroupBox(ko('Connection / Status'))
        form = QFormLayout(box)
        self.status_labels = {}
        self.status_titles = {}
        for key, title in (
                ('connection', 'Bridge connection'),
                ('u2d2', 'U2D2 / serial'), ('tool_type', 'Tool type'),
                ('profile_valid', 'Profile valid'),
                ('calibration_valid', 'Calibration / endpoints valid'),
                ('actuators_discovered', 'Actuators discovered'),
                ('motion_allowed', 'Motion allowed'), ('fsm', 'FSM state'),
                ('arm_status', 'Arm contract state'), ('mode', 'Control mode'),
                ('dual_online', 'ID3 / ID4 online'),
                ('dual_positions', 'ID3 / ID4 positions'),
                ('dual_torque', 'ID3 / ID4 torque'),
                ('dual_hw_error', 'ID3 / ID4 hardware error'),
                ('dual_sync', 'Dual synchronization'),
                ('spur_online', 'ID5 연결'),
                ('spur_position', 'ID5 위치 (틱)'),
                ('spur_torque', 'ID5 토크'),
                ('spur_hw_error', 'ID5 하드웨어 오류'),
                ('spur_operating_mode', 'ID5 운전 모드'),
                ('contact', 'Contact sensor')):
            label = QLabel(ko('UNKNOWN'))
            self.status_labels[key] = label
            title_label = QLabel(ko(title))
            self.status_titles[key] = title_label
            form.addRow(title_label, label)
        self._update_status_panel({})
        return box

    def _update_status_panel(self, status):
        """Display-only selection; never publish a motor or ownership command."""
        tool = self.node.selected_tool
        for key, label in self.status_labels.items():
            visible = (tool == 'dual_motor_gripper' if key.startswith('dual_')
                       else tool == 'spur_1motor_gripper' if key.startswith('spur_')
                       else True)
            if tool == 'spur_1motor_gripper':
                visible = key.startswith('spur_') or key == 'fsm'
            label.setVisible(visible)
            self.status_titles[key].setVisible(visible)
        if hasattr(self, 'diag'):
            self.diag.setVisible(tool != 'spur_1motor_gripper')
        sample = next((item for item in status.get('actuators', [])
                       if item.get('id') == 5), {}) if tool == 'spur_1motor_gripper' else {}
        for key, field in (('spur_online', 'online'), ('spur_position', 'position'),
                           ('spur_torque', 'torque_state'), ('spur_hw_error', 'hardware_error'),
                           ('spur_operating_mode', 'operating_mode')):
            value = sample.get(field)
            if value is None:
                text = 'UNKNOWN'
            elif field == 'online':
                text = '연결됨' if value else '연결 끊김'
            elif field == 'operating_mode':
                text = {1: '1 (속도 제어)', 3: '3 (위치 제어)',
                        4: '4 (확장 위치 제어)', 5: '5 (전류 기반 위치 제어)'}.get(value, str(value))
            else:
                text = str(value)
            self.status_labels[key].setText(ko(text))

    def _arm_group(self):
        box = QGroupBox(ko('Arm Manual Control'))
        layout = QGridLayout(box)
        layout.addWidget(QLabel(ko('Joint')), 0, 0)
        layout.addWidget(QLabel(ko('Current rad')), 0, 1)
        layout.addWidget(QLabel(ko('Jog')), 0, 2, 1, 2)
        layout.addWidget(QLabel(ko('Target rad')), 0, 4)
        self.arm_buttons = []
        self.arm_position_labels = {}
        self.arm_targets = {}
        for row, joint in enumerate(ARM_JOINTS, 1):
            label = QLabel(ko('0.0000'))
            minus = QPushButton(ko('−'))
            plus = QPushButton(ko('+'))
            target = QDoubleSpinBox()
            target.setRange(-6.283, 6.283)
            target.setDecimals(4)
            send = QPushButton(ko('GO'))
            minus.clicked.connect(
                lambda _checked=False, name=joint: self._jog(name, -1))
            plus.clicked.connect(
                lambda _checked=False, name=joint: self._jog(name, 1))
            send.clicked.connect(
                lambda _checked=False, name=joint: self._arm_target(name))
            layout.addWidget(QLabel(ko(joint)), row, 0)
            layout.addWidget(label, row, 1)
            layout.addWidget(minus, row, 2)
            layout.addWidget(plus, row, 3)
            layout.addWidget(target, row, 4)
            layout.addWidget(send, row, 5)
            self.arm_position_labels[joint] = label
            self.arm_targets[joint] = target
            self.arm_buttons.extend([minus, plus, target, send])
            self.arm_widgets[joint] = [minus, plus, target, send]
        self.jog_step = QComboBox()
        self.jog_step.addItems(['0.5', '1.0', '5.0'])
        layout.addWidget(QLabel(ko('Jog step (deg)')), 6, 0)
        layout.addWidget(self.jog_step, 6, 1)
        return box

    def _tool_selection_group(self):
        box = QGroupBox(ko('Tool Selection / Ownership'))
        form = QFormLayout(box)
        self.tool_combo = QComboBox()
        for tool in ('dual_motor_gripper', 'spur_1motor_gripper', 'cleaner'):
            self.tool_combo.addItem(ko(tool), tool)
        self.tool_combo.setCurrentIndex(self.tool_combo.findData(self.node.selected_tool))
        request = QPushButton(ko('REQUEST TOOL CHANGE'))
        request.clicked.connect(self._request_tool_change)
        self.mode_combo = QComboBox()
        for mode in ('FSM', 'MANUAL'):
            self.mode_combo.addItem(ko(mode), mode)
        mode_request = QPushButton(ko('REQUEST MODE'))
        mode_request.clicked.connect(self._request_mode)
        form.addRow(ko('Selected tool'), self.tool_combo)
        form.addRow(ko(''), request)
        form.addRow(ko('Ownership'), self.mode_combo)
        form.addRow(ko(''), mode_request)
        return box

    def _tool_control_group(self):
        # This panel is rebuilt when the active runtime tool changes. Clear
        # references to widgets belonging to the previous tool first.
        self.dual_recovery_buttons = []
        self.dual_calibration_buttons = []
        for name in (
                'dual_start_calibration', 'dual_calibration_state',
                'dual_calibration_step', 'dual_capture_open',
                'dual_capture_close', 'dual_capture_label',
                'dual_validate_calibration', 'dual_save_calibration',
                'spur_actual_state', 'capture_open', 'capture_close',
                'captured_endpoints_label', 'validate_calibration',
                'save_calibration', 'spur_mapping', 'spur_minus_5',
                'spur_zero', 'spur_plus_5', 'motor_minus_half', 'motor_plus_half',
                'motor_minus_one', 'motor_plus_one'):
            setattr(self, name, None)
        box = QGroupBox(ko('End Effector'))
        layout = QVBoxLayout(box)
        self.profile_text = QLabel(ko(self._profile_summary()))
        self.profile_text.setWordWrap(True)
        layout.addWidget(self.profile_text)
        if not hasattr(self, 'common_enable'):
            self.open_button = QPushButton(ko('OPEN'))
            self.close_button = QPushButton(ko('CLOSE'))
            self.tool_stop = QPushButton(ko('STOP'))
            self.hold_open_button = QPushButton(ko('HOLD TO OPEN'))
            self.hold_close_button = QPushButton(ko('HOLD TO CLOSE'))
            self.common_enable = QPushButton('활성화')
            self.common_disable = QPushButton('비활성화')
            self.open_button.clicked.connect(lambda: self._command_tool('OPEN'))
            self.close_button.clicked.connect(lambda: self._command_tool('CLOSE'))
            self.tool_stop.clicked.connect(self._stop_tool)
            self.common_enable.clicked.connect(lambda: self._common_torque(True))
            self.common_disable.clicked.connect(lambda: self._common_torque(False))
            self.hold_open_button.pressed.connect(lambda: self._common_hold('OPEN'))
            self.hold_close_button.pressed.connect(lambda: self._common_hold('CLOSE'))
            self.hold_open_button.released.connect(self._common_release)
            self.hold_close_button.released.connect(self._common_release)
            self.spur_enable = self.dual_enable = self.common_enable
            self.spur_disable = self.dual_disable = self.common_disable
        for widgets in ((self.open_button, self.close_button, self.tool_stop),
                        (self.hold_open_button, self.hold_close_button),
                        (self.common_enable, self.common_disable)):
            row = QHBoxLayout()
            for widget in widgets:
                row.addWidget(widget)
            layout.addLayout(row)
        self.dual_recovery_buttons = []
        if self.node.selected_tool == 'dual_motor_gripper':
            recovery = QGroupBox(ko('MANUAL DUAL MOTOR RECOVERY (one click only)'))
            recovery_layout = QGridLayout(recovery)
            for row, dxl_id in enumerate((3, 4)):
                recovery_layout.addWidget(QLabel(ko(f'ID{dxl_id}')), row, 0)
                for column, delta in enumerate((-0.5, 0.5), 1):
                    button = QPushButton(ko(f'ID{dxl_id} {delta:+.1f}°'))
                    button.setAutoRepeat(False)
                    button.clicked.connect(
                        lambda _checked=False, motor=dxl_id, step=delta:
                        self._manual_dual_recovery_jog(motor, step))
                    recovery_layout.addWidget(button, row, column)
                    self.dual_recovery_buttons.append((dxl_id, button))
            layout.addWidget(recovery)
            calibration = QGroupBox(ko('DUAL ENDPOINT CALIBRATION'))
            calibration_layout = QGridLayout(calibration)
            self.dual_start_calibration = QPushButton(ko('START DUAL CALIBRATION'))
            self.dual_start_calibration.clicked.connect(self._start_dual_calibration)
            self.dual_calibration_state = QLabel(ko('RECALIBRATION_REQUIRED'))
            calibration_layout.addWidget(self.dual_start_calibration, 0, 0, 1, 2)
            calibration_layout.addWidget(self.dual_calibration_state, 0, 2, 1, 2)
            calibration_layout.addWidget(QLabel(ko('Calibration step (motor degree)')), 1, 0, 1, 2)
            self.dual_calibration_step = QComboBox()
            self.dual_calibration_step.addItems(['0.5', '1', '2', '5'])
            calibration_layout.addWidget(self.dual_calibration_step, 1, 2, 1, 2)
            for row, dxl_id in enumerate((3, 4), 2):
                calibration_layout.addWidget(QLabel(ko(f'ID{dxl_id}')), row, 0)
                for column, direction in enumerate((-1.0, 1.0), 1):
                    button = QPushButton(ko(f'ID{dxl_id} {"−" if direction < 0 else "+"} step'))
                    button.setAutoRepeat(False)
                    button.clicked.connect(
                        lambda _checked=False, motor=dxl_id, sign=direction:
                        self._jog_dual_calibration_motor(motor, sign))
                    calibration_layout.addWidget(button, row, column)
                    self.dual_calibration_buttons.append((dxl_id, button))
            self.dual_capture_open = QPushButton(ko('CAPTURE OPEN'))
            self.dual_capture_close = QPushButton(ko('CAPTURE CLOSE'))
            self.dual_capture_open.clicked.connect(
                lambda: self._command_dual_calibration('capture_open'))
            self.dual_capture_close.clicked.connect(
                lambda: self._command_dual_calibration('capture_close'))
            calibration_layout.addWidget(self.dual_capture_open, 4, 0, 1, 2)
            calibration_layout.addWidget(self.dual_capture_close, 4, 2, 1, 2)
            self.dual_capture_label = QLabel(ko('Captured OPEN: — | CLOSE: —'))
            calibration_layout.addWidget(self.dual_capture_label, 5, 0, 1, 4)
            self.dual_validate_calibration = QPushButton(ko('VALIDATE DUAL CALIBRATION'))
            self.dual_save_calibration = QPushButton(ko('SAVE DUAL CALIBRATION'))
            self.dual_validate_calibration.clicked.connect(
                lambda: self._command_dual_calibration('validate'))
            self.dual_save_calibration.clicked.connect(
                lambda: self._command_dual_calibration('save'))
            calibration_layout.addWidget(self.dual_validate_calibration, 6, 0, 1, 2)
            calibration_layout.addWidget(self.dual_save_calibration, 6, 2, 1, 2)
            calibration_layout.addWidget(QLabel(
                ko('Captured endpoint pairs become the only OPEN/CLOSE targets. '
                'Capture itself performs reads only.')), 7, 0, 1, 4)
            bypass = QLabel(
                ko('CALIBRATION JOG: spread protection bypassed\n'
                'Normal OPEN/CLOSE and legacy gripper JOG remain blocked until READY.'))
            bypass.setStyleSheet(FALSE_STYLE)
            bypass.setWordWrap(True)
            calibration_layout.addWidget(bypass, 8, 0, 1, 4)
            layout.addWidget(calibration)
        jog = QGroupBox(ko('GRIPPER JOG'))
        jog_layout = QGridLayout(jog)
        self.spur_minus_5 = self.spur_zero = self.spur_plus_5 = None
        if self.node.selected_tool == 'spur_1motor_gripper':
            left_label, right_label = '누르는 동안 열기 (열림 끝점 방향)', '누르는 동안 닫기 (닫힘 끝점 방향)'
        else:
            left_label, right_label = 'LEFT / +  (OPEN)', 'RIGHT / −  (CLOSE)'
        self.jog_close = QPushButton(ko(left_label))
        self.jog_open = QPushButton(ko(right_label))
        self.gripper_jog_step = QComboBox()
        self.gripper_jog_step.addItems(['5', '10', '25', '50'])
        if self.node.selected_tool == 'spur_1motor_gripper':
            self.gripper_jog_step.clear()
            self.gripper_jog_step.addItem('0.5°')
        self.gripper_busy_label = QLabel(ko('READY'))
        self.gripper_position_label = QLabel(ko('Gripper position: UNKNOWN'))
        self.gripper_feedback_label = QLabel(ko('ID3: UNKNOWN\nID4: UNKNOWN'))
        self.gripper_feedback_label.setWordWrap(True)
        shortcut = QLabel(
            ko('Shortcuts: Left=OPEN jog, Right=CLOSE jog, Space=STOP\n'
            '(disabled while editing a field; key auto-repeat ignored)'))
        if self.node.selected_tool == 'spur_1motor_gripper':
            shortcut.setText(ko('Left: ID5 −0.5° / Right: ID5 +0.5° / Space: STOP'))
        shortcut.setWordWrap(True)
        if self.node.selected_tool == 'spur_1motor_gripper':
            self.jog_close.pressed.connect(lambda: self._start_spur_hold('manual_open'))
            self.jog_open.pressed.connect(lambda: self._start_spur_hold('manual_close'))
            self.jog_close.released.connect(self._release_spur_hold)
            self.jog_open.released.connect(self._release_spur_hold)
        else:
            self.jog_close.clicked.connect(lambda: self._jog_gripper(-1))
            self.jog_open.clicked.connect(lambda: self._jog_gripper(1))
        jog_layout.addWidget(self.jog_close, 0, 0)
        jog_layout.addWidget(self.jog_open, 0, 1)
        step_label = ('Motor step' if self.node.selected_tool == 'spur_1motor_gripper'
                      else 'Step (tick equivalent)')
        jog_layout.addWidget(QLabel(ko(step_label)), 1, 0)
        jog_layout.addWidget(self.gripper_jog_step, 1, 1)
        jog_layout.addWidget(self.gripper_busy_label, 2, 0, 1, 2)
        jog_layout.addWidget(self.gripper_position_label, 3, 0, 1, 2)
        jog_layout.addWidget(self.gripper_feedback_label, 4, 0, 1, 2)
        shortcut_row = 5
        if self.node.selected_tool == 'spur_1motor_gripper':
            self.spur_actual_state = QLabel(ko('ID5: position=UNKNOWN torque=UNKNOWN load=UNKNOWN'))
            jog_layout.addWidget(self.spur_actual_state, 5, 0, 1, 2)
            self.capture_open = QPushButton(ko('SET CURRENT AS OPEN'))
            self.capture_close = QPushButton(ko('SET CURRENT AS CLOSE'))
            self.capture_open.clicked.connect(lambda: self._capture_spur_endpoint('open'))
            self.capture_close.clicked.connect(lambda: self._capture_spur_endpoint('close'))
            jog_layout.addWidget(self.capture_open, 6, 0)
            jog_layout.addWidget(self.capture_close, 6, 1)
            self.captured_endpoints_label = QLabel(ko('Captured OPEN: — | CLOSE: —'))
            jog_layout.addWidget(self.captured_endpoints_label, 7, 0, 1, 2)
            self.validate_calibration = QPushButton(ko('VALIDATE CALIBRATION'))
            self.save_calibration = QPushButton(ko('SAVE CALIBRATION'))
            self.validate_calibration.clicked.connect(self._validate_spur_calibration)
            self.save_calibration.clicked.connect(self._save_spur_calibration)
            jog_layout.addWidget(self.validate_calibration, 8, 0)
            jog_layout.addWidget(self.save_calibration, 8, 1)
            self.motor_minus_half = QPushButton(ko('MOTOR −0.5°'))
            self.motor_plus_half = QPushButton(ko('MOTOR +0.5°'))
            self.motor_minus_one = QPushButton(ko('MOTOR −1°'))
            self.motor_plus_one = QPushButton(ko('MOTOR +1°'))
            for button, degrees in ((self.motor_minus_half, -0.5),
                                    (self.motor_plus_half, 0.5),
                                    (self.motor_minus_one, -1.0),
                                    (self.motor_plus_one, 1.0)):
                button.clicked.connect(
                    lambda _checked=False, delta=degrees: self._jog_spur_motor(delta))
            jog_layout.addWidget(self.motor_minus_half, 9, 0)
            jog_layout.addWidget(self.motor_plus_half, 9, 1)
            jog_layout.addWidget(self.motor_minus_one, 10, 0)
            jog_layout.addWidget(self.motor_plus_one, 10, 1)
            jog_layout.addWidget(QLabel(
                ko('Safety policy: captured OPEN/CLOSE are the command limits; '
                'no hidden endpoint or temporary range is used.')), 11, 0, 1, 2)
            self.spur_mapping = QLabel(ko('Output mapping: waiting for ID5 feedback'))
            self.spur_mapping.setWordWrap(True)
            jog_layout.addWidget(self.spur_mapping, 12, 0, 1, 2)
            self.spur_minus_5 = QPushButton(ko('OUTPUT −5°'))
            self.spur_zero = QPushButton(ko('OUTPUT 0°'))
            self.spur_plus_5 = QPushButton(ko('OUTPUT +5°'))
            self.spur_minus_5.clicked.connect(lambda: self._command_spur_output_deg(-5.0))
            self.spur_zero.clicked.connect(lambda: self._command_spur_output_deg(0.0))
            self.spur_plus_5.clicked.connect(lambda: self._command_spur_output_deg(5.0))
            jog_layout.addWidget(self.spur_minus_5, 13, 0)
            jog_layout.addWidget(self.spur_zero, 13, 1)
            jog_layout.addWidget(self.spur_plus_5, 14, 0, 1, 2)
            shortcut_row = 15
        jog_layout.addWidget(shortcut, shortcut_row, 0, 1, 2)
        layout.addWidget(jog)
        cleaner = QHBoxLayout()
        self.clean_start = QPushButton(ko('CLEANER START'))
        self.clean_stop = QPushButton(ko('CLEANER STOP'))
        self.clean_start.clicked.connect(lambda: self.node.command_cleaner(True))
        self.clean_stop.clicked.connect(lambda: self.node.command_cleaner(False))
        cleaner.addWidget(self.clean_start)
        cleaner.addWidget(self.clean_stop)
        layout.addLayout(cleaner)
        calibration = QHBoxLayout()
        self.read_diag = QPushButton(ko('READ ONLY DIAGNOSTIC'))
        self.start_cal = QPushButton(ko('START CALIBRATION'))
        self.read_diag.clicked.connect(self._read_only_diagnostic)
        self.start_cal.clicked.connect(self._start_calibration)
        calibration.addWidget(self.read_diag)
        calibration.addWidget(self.start_cal)
        layout.addLayout(calibration)
        dual = self.node.selected_tool == 'dual_motor_gripper'
        spur = self.node.selected_tool == 'spur_1motor_gripper'
        for widget in (self.hold_open_button, self.hold_close_button,
                       self.dual_enable, self.dual_disable):
            widget.setVisible(dual)
        for widget in (self.spur_enable, self.spur_disable, self.read_diag, self.start_cal):
            widget.setVisible(spur)
        for widget in (self.open_button, self.close_button, self.tool_stop, jog):
            widget.setVisible(dual or spur)
        self.clean_start.setVisible(not (dual or spur))
        self.clean_stop.setVisible(not (dual or spur))
        return box

    def _common_buttons(self):
        return (self.open_button, self.close_button, self.tool_stop,
                self.common_enable, self.common_disable,
                self.hold_open_button, self.hold_close_button)

    def _common_torque(self, enabled):
        if self.node.selected_tool == 'dual_motor_gripper':
            (self._enable_dual_motors if enabled else self._disable_dual_motors)()
        elif self.node.selected_tool == 'spur_1motor_gripper':
            self.node.command_calibration('manual_enable' if enabled else 'manual_disable')

    def _common_hold(self, direction):
        if self.node.selected_tool == 'dual_motor_gripper':
            self._start_dual_hold_jog(direction)
        elif self.node.selected_tool == 'spur_1motor_gripper':
            self._start_spur_hold('manual_open' if direction == 'OPEN' else 'manual_close')

    def _common_release(self):
        if self.node.selected_tool == 'dual_motor_gripper':
            self._release_dual_hold_jog()
        elif self.node.selected_tool == 'spur_1motor_gripper':
            self._release_spur_hold()

    def _spur_enable_ready(self):
        status = self.tool_status
        sample = self._gripper_samples().get(5, {})
        return bool(self.node.selected_tool == 'spur_1motor_gripper'
                    and status.get('tool_type') == 'spur_1motor_gripper'
                    and self.pending_tool_change is None
                    and self.control_mode == 'MANUAL'
                    and self.node.control_scope == 'END_EFFECTOR_ONLY'
                    and time.monotonic() - self.last_status_time < 1.5
                    and not status.get('read_only') and not getattr(self.node, 'read_only', False)
                    and not status.get('emergency_stop') and not status.get('tool_detached')
                    and sample.get('online') and sample.get('hardware_error') == 0
                    and self.fsm_state in ('READY', 'OPEN', 'CLOSED'))

    def _refresh_common_buttons(self):
        tool = self.node.selected_tool
        for widget in self._common_buttons():
            widget.setVisible(tool in ('dual_motor_gripper', 'spur_1motor_gripper'))
        if tool != 'spur_1motor_gripper':
            # The existing dual gates above are authoritative, unchanged.
            return
        sample = self._gripper_samples().get(5, {})
        ready = self._spur_enable_ready()
        torque_on = sample.get('torque_state') == 'ON'
        # Torque-off readiness must never depend on motion_allowed or calibration-session state.
        self.common_enable.setEnabled(ready and not torque_on)
        motion = ready and torque_on
        for widget in (self.open_button, self.close_button,
                       self.hold_open_button, self.hold_close_button):
            widget.setEnabled(motion)
        writable = (not self.tool_status.get('read_only')
                    and not getattr(self.node, 'read_only', False))
        self.common_disable.setEnabled(bool(sample.get('online') and writable and torque_on))
        self.tool_stop.setEnabled(bool(sample.get('online') and writable))
        self._log_spur_enable_diagnostics()

    def _log_spur_enable_diagnostics(self):
        status = self.tool_status
        sample = self._gripper_samples().get(5, {})
        gates = {
            'selected_spur': self.node.selected_tool == 'spur_1motor_gripper',
            'reported_spur': status.get('tool_type') == 'spur_1motor_gripper',
            'no_pending_change': self.pending_tool_change is None,
            'manual': self.control_mode == 'MANUAL',
            'scope': self.node.control_scope == 'END_EFFECTOR_ONLY',
            'fresh': time.monotonic() - self.last_status_time < 1.5,
            'bridge_writable': not bool(status.get('read_only')),
            'gui_writable': not getattr(self.node, 'read_only', False),
            'no_estop': not bool(status.get('emergency_stop')),
            'attached': not bool(status.get('tool_detached')),
            'online': bool(sample.get('online')),
            'hw_error_zero': sample.get('hardware_error') == 0,
            'fsm_ready': self.fsm_state in ('READY', 'OPEN', 'CLOSED'),
            'torque_not_on': sample.get('torque_state') != 'ON',
        }
        parents = []
        parent = self.common_enable.parentWidget()
        while parent is not None:
            parents.append({'class': type(parent).__name__, 'enabled': parent.isEnabled()})
            parent = parent.parentWidget()
        record = {
            'selected_tool': self.node.selected_tool, 'control_mode': self.control_mode,
            'node_control_mode': getattr(self.node, 'control_mode', None), 'fsm_state': self.fsm_state,
            'id5': sample, 'mode_combo': self.mode_combo.currentData(),
            'tool_status': {k: status.get(k) for k in ('tool_type', 'control_mode',
                'fsm_state', 'tool_enable_allowed', 'motion_allowed', 'profile_valid', 'calibrated')},
            'enable_conditions': gates, 'setEnabled_argument': all(gates.values()),
            'button_isEnabled': self.common_enable.isEnabled(), 'parents': parents,
        }
        encoded = json.dumps(record, sort_keys=True, ensure_ascii=False)
        if encoded != getattr(self, '_last_enable_diagnostics', None):
            self._last_enable_diagnostics = encoded
            logger = getattr(self.node, 'get_logger', None)
            if logger:
                logger().info('SPUR_ENABLE_DIAGNOSTICS ' + encoded)

    def _profile_summary(self):
        keys = ('calibrated', 'actuator_ids', 'safe_min_tick', 'safe_max_tick',
                'open_tick', 'close_tick', 'profile_velocity',
                'profile_acceleration')
        return '\n'.join(f'{key}: {self.profile.get(key)}' for key in keys)

    def _rebuild_tool_control_group(self):
        old = getattr(self, 'tool_control_box', None)
        if old is not None:
            for widget in self._common_buttons():
                widget.setParent(None)
            self.right_layout.removeWidget(old)
            old.setEnabled(False)
            old.hide()
            old.deleteLater()
        self.tool_control_box = self._tool_control_group()
        self.right_layout.addWidget(self.tool_control_box)

    def _connect_signals(self):
        self.signals.joint_states.connect(self._update_joints)
        self.signals.tool_status.connect(self._update_tool_status)
        self.signals.fsm_state.connect(self._update_mission_fsm)
        self.signals.control_mode.connect(self._update_mode)
        self.signals.arm_status.connect(
            lambda value: self.status_labels['arm_status'].setText(ko(value)))
        self.signals.contact_status.connect(
            lambda value: self._set_bool(self.status_labels['contact'], value))
        self.signals.log.connect(self._append_log)
        self.signals.gripper_state.connect(self._update_gripper_state)

    def _set_bool(self, label, value):
        label.setText(ko('TRUE' if value else 'FALSE'))
        label.setStyleSheet(TRUE_STYLE if value else FALSE_STYLE)

    def _refresh_connection(self):
        connected = time.monotonic() - self.last_status_time < 1.5
        self._set_bool(self.status_labels['connection'], connected)
        if not connected:
            self._set_bool(self.status_labels['motion_allowed'], False)
        self._refresh_buttons()

    def _update_tool_status(self, status):
        self.tool_status = status
        self.last_status_time = time.monotonic()
        previous_tool = self.node.selected_tool
        reported_tool = status.get('tool_type')
        runtime_profile = status.get('tool_profile')
        if isinstance(runtime_profile, dict):
            self.profile = runtime_profile
        change = status.get('tool_change') or {}
        if self.pending_tool_change:
            if reported_tool == self.pending_tool_change:
                self.node.selected_tool = reported_tool
                self.pending_tool_change = None
                self._append_log(ko(f'도구 런타임 전환 완료: {reported_tool}'))
            elif change.get('error'):
                self.pending_tool_change = None
                self.tool_combo.setCurrentIndex(self.tool_combo.findData(reported_tool))
                self._append_log(ko(f'도구 런타임 전환 거부: {change["error"]}'))
        elif (reported_tool in ('spur_1motor_gripper', 'dual_motor_gripper', 'cleaner')
              and reported_tool != self.node.selected_tool):
            # The bridge's active tool is separate from the user's pending
            # combo selection. Sync the combo only on an active-tool change.
            self.node.selected_tool = reported_tool
        if self.node.selected_tool != previous_tool:
            # The bridge already stopped the old tool. Do not send HOLD to
            # the newly selected tool from an old timer/release callback.
            self.spur_hold_timer.stop()
            self.spur_hold_command = None
            self.dual_key_jog_timer.stop()
            self.dual_key_jog_direction = 0
            self.dual_hold_jog_active = False
            self.dual_hold_jog_direction = None
            self.gripper_busy = self.node.gripper_busy = False
            self.node.last_gripper_goal = None
            self.node.tool_context_generation = getattr(self.node, 'tool_context_generation', 0) + 1
            self.gripper_target_ticks = {}
            self.spur_endpoints = {}
            self.spur_zero_tick = None
            self.spur_torque_enabled = False
            self.spur_torque_state = 'UNKNOWN'
            for key in ('dual_online', 'dual_positions', 'dual_torque', 'dual_hw_error', 'dual_sync'):
                self.status_labels[key].setText('—')
            self.tool_combo.setCurrentIndex(self.tool_combo.findData(self.node.selected_tool))
            self._rebuild_tool_control_group()
        if self.node.selected_tool == 'spur_1motor_gripper':
            # Mode-status is event-only; recover the current mode from periodic tool context.
            mode = status.get('control_mode')
            if mode in ('MANUAL', 'FSM'):
                self.control_mode = self.node.control_mode = mode
                self.status_labels['mode'].setText(ko(mode))
        self._update_status_panel(status)
        self.node.tool_profile = self.profile
        self.node.actuator_ids = list(self.profile.get('actuator_ids', []))
        self.profile_text.setText(ko(self._profile_summary()))
        self.status_labels['tool_type'].setText(ko(status.get('tool_type', 'UNKNOWN')))
        if (self.node.selected_tool != 'cleaner' or previous_tool != 'cleaner'
                or status.get('fsm_state') in ('READY', 'CLEANING', 'STOPPED',
                                               'CALIBRATION_REQUIRED')):
            self._update_fsm(status.get('fsm_state') or 'UNKNOWN')
        self._set_bool(
            self.status_labels['u2d2'], bool(status.get('u2d2_connected')))
        for key in ('profile_valid', 'actuators_discovered', 'motion_allowed'):
            self._set_bool(self.status_labels[key], bool(status.get(key)))
        self._set_bool(
            self.status_labels['calibration_valid'],
            bool(status.get('calibrated')) and (
                self.node.selected_tool != 'dual_motor_gripper'
                or bool(status.get('endpoint_calibration_verified'))))
        estop = bool(status.get('emergency_stop'))
        self.estop_state.setText(ko(f'E-STOP: {str(estop).upper()}'))
        self.estop_state.setStyleSheet(FALSE_STYLE if estop else TRUE_STYLE)
        self._rebuild_diagnostics(status.get('actuators', []))
        self._update_gripper_feedback()
        self._refresh_buttons()
        samples = self._gripper_samples()
        if self.node.selected_tool == 'dual_motor_gripper':
            self.status_labels['dual_online'].setText(
                ko(' / '.join(f'ID{i}={bool(samples.get(i, {}).get("online"))}'
                           for i in (3, 4))))
            self.status_labels['dual_positions'].setText(
                ko(' / '.join(f'ID{i}={samples.get(i, {}).get("position")}'
                           for i in (3, 4))))
            self.status_labels['dual_torque'].setText(
                ko(' / '.join(f'ID{i}={samples.get(i, {}).get("torque_state", "UNKNOWN")}'
                           for i in (3, 4))))
            self.status_labels['dual_hw_error'].setText(
                ko(' / '.join(f'ID{i}={samples.get(i, {}).get("hardware_error")}'
                           for i in (3, 4))))
            sync = status.get('synchronization') or {}
            self.status_labels['dual_sync'].setText(
                ko(f'{sync.get("state", "UNKNOWN")} spread={sync.get("spread")} '
                f'(limit={sync.get("limit", 0.05)})'))

    def _update_joints(self, values):
        for joint, sample in values.items():
            if joint in self.arm_position_labels and sample['position'] is not None:
                self.seen_arm_joints.add(joint)
                self.arm_position_labels[joint].setText(ko(f'{sample["position"]:.4f}'))
                self.arm_targets[joint].setValue(float(sample['position']))
        self._refresh_buttons()
        self._rebuild_diagnostics(self.tool_status.get('actuators', []), values)

    def _update_mission_fsm(self, state):
        if self.node.selected_tool == 'cleaner':
            self._update_fsm(state)

    def _update_fsm(self, state):
        self.fsm_state = state
        self.status_labels['fsm'].setText(ko(state))

    def _update_mode(self, mode):
        self.control_mode = mode
        self.status_labels['mode'].setText(ko(mode))
        self._refresh_buttons()

    def _refresh_buttons(self):
        self.tool_control_box.setEnabled(self.pending_tool_change is None)
        manual = self.control_mode == 'MANUAL'
        end_effector_only = self.node.control_scope == 'END_EFFECTOR_ONLY'
        for widget in self.arm_buttons:
            widget.setEnabled(manual and not end_effector_only)
        if not self.mock_mode:
            for joint, widgets in self.arm_widgets.items():
                for widget in widgets:
                    widget.setEnabled(
                        manual and not end_effector_only
                        and joint in self.seen_arm_joints)
        profile_ok = bool(self.tool_status.get('profile_valid'))
        motion = self._tool_motion_ready()
        gripper = self.node.selected_tool.endswith('gripper')
        spur = self.node.selected_tool == 'spur_1motor_gripper'
        dual = self.node.selected_tool == 'dual_motor_gripper'
        calibrated = bool(self.tool_status.get('calibrated')) or self.mock_mode
        captured = (set(self.spur_endpoints) == {'open', 'close'}
                    and self.spur_endpoints['open'] != self.spur_endpoints['close'])
        dual_calibration = self.tool_status.get('dual_calibration') or {}
        dual_ready = dual_calibration.get('state') == 'READY'
        fsm_commandable = self.fsm_state in ('READY', 'OPEN', 'CLOSED')
        preset_ready = (manual and gripper and profile_ok and motion
                        and calibrated and not self.gripper_busy
                        and (not spur or fsm_commandable)
                        and (not dual or (dual_ready and fsm_commandable)))
        # Captures are only a candidate.  They never silently turn an
        # uncalibrated live profile into a normal-motion profile.
        self.open_button.setEnabled(preset_ready)
        self.close_button.setEnabled(preset_ready)
        self.spur_enable.setVisible(spur)
        self.spur_disable.setVisible(spur)
        self.dual_enable.setVisible(dual)
        self.dual_disable.setVisible(dual)
        calibration = self.tool_status.get('calibration') or {}
        self.spur_enable.setEnabled(
            spur and manual and calibration.get('active', False)
            and self._tool_enable_ready() and not calibration.get('enabled', False))
        self.spur_disable.setEnabled(
            spur and calibration.get('active', False)
            and self.spur_torque_state == 'ON')
        dual_samples = self._gripper_samples()
        dual_online = all(dual_samples.get(dxl_id, {}).get('online')
                          for dxl_id in (3, 4))
        dual_healthy = all(dual_samples.get(dxl_id, {}).get('hardware_error') == 0
                           for dxl_id in (3, 4))
        dual_torque_on = all(dual_samples.get(dxl_id, {}).get('torque_state') == 'ON'
                             for dxl_id in (3, 4))
        dual_profile_verified = bool(
            self.tool_status.get('endpoint_calibration_verified'))
        dual_synchronized = ((self.tool_status.get('synchronization') or {}).get(
            'state') == 'SYNCHRONIZED')
        if dual:
            self.open_button.setEnabled(
                preset_ready and dual_online and dual_healthy
                and dual_torque_on and dual_profile_verified
                and dual_synchronized)
            self.close_button.setEnabled(self.open_button.isEnabled())
        hold_jog_ready = dual and self.open_button.isEnabled()
        self.hold_open_button.setVisible(dual)
        self.hold_close_button.setVisible(dual)
        self.hold_open_button.setEnabled(
            (self.dual_hold_jog_active
             and self.dual_hold_jog_direction == 'OPEN')
            or (hold_jog_ready and not self.dual_hold_jog_active))
        self.hold_close_button.setEnabled(
            (self.dual_hold_jog_active
             and self.dual_hold_jog_direction == 'CLOSE')
            or (hold_jog_ready and not self.dual_hold_jog_active))
        self.tool_stop.setEnabled(
            (spur and not bool(self.tool_status.get('read_only'))
             and bool(self.tool_status.get('online')))
            or (dual and not bool(self.tool_status.get('read_only'))))
        self.dual_enable.setEnabled(
            dual and manual and dual_online and dual_healthy
            and not dual_torque_on and not bool(self.tool_status.get('read_only')))
        self.dual_disable.setEnabled(
            dual and dual_online and not bool(self.tool_status.get('read_only')))
        recovery_base = (
            dual and manual and self.node.control_scope == 'END_EFFECTOR_ONLY'
            and not bool(self.tool_status.get('read_only'))
            and not bool(self.tool_status.get('emergency_stop'))
            and not bool(self.tool_status.get('tool_detached')))
        for dxl_id, button in self.dual_recovery_buttons:
            sample = dual_samples.get(dxl_id, {})
            button.setEnabled(
                recovery_base and bool(sample.get('online'))
                and sample.get('hardware_error') == 0
                and sample.get('torque_state') == 'ON')
        dual_calibration_active = bool(dual_calibration.get('active'))
        dual_calibration_capture_ready = (
            dual and manual and end_effector_only
            and not bool(self.tool_status.get('read_only'))
            and not bool(self.tool_status.get('emergency_stop'))
            and not bool(self.tool_status.get('tool_detached'))
            and dual_online and dual_healthy)
        dual_calibration_jog_ready = (
            dual_calibration_capture_ready and dual_torque_on)
        if self.dual_calibration_state is not None:
            self.dual_calibration_state.setText(
                ko(dual_calibration.get('state', 'RECALIBRATION_REQUIRED')))
        if self.dual_capture_label is not None:
            captures = dual_calibration.get('captures') or {}
            self.dual_capture_label.setText(
                ko(f'Captured OPEN: {captures.get("open", "—")} | '
                f'CLOSE: {captures.get("close", "—")}'))
        if self.dual_start_calibration is not None:
            self.dual_start_calibration.setEnabled(
                dual and manual and end_effector_only
                and not bool(self.tool_status.get('read_only'))
                and not bool(self.tool_status.get('emergency_stop'))
                and not bool(self.tool_status.get('tool_detached'))
                and not dual_calibration_active)
        for _dxl_id, button in self.dual_calibration_buttons:
            button.setEnabled(dual_calibration_active and dual_calibration_jog_ready)
        if self.dual_calibration_step is not None:
            self.dual_calibration_step.setEnabled(
                dual_calibration_active and dual_calibration_jog_ready)
        if self.dual_capture_open is not None:
            self.dual_capture_open.setEnabled(
                dual_calibration_active and dual_calibration_capture_ready)
            self.dual_capture_close.setEnabled(
                dual_calibration_active and dual_calibration_capture_ready)
            captured_pairs = dual_calibration.get('captures') or {}
            both_pairs = set(captured_pairs) == {'open', 'close'}
            self.dual_validate_calibration.setEnabled(
                dual_calibration_active and dual_calibration_capture_ready and both_pairs)
            self.dual_save_calibration.setEnabled(
                dual_calibration_active and bool(dual_calibration.get('validated')))
        jog_ready = (manual and not self.gripper_busy
                     and self.node.control_scope == 'END_EFFECTOR_ONLY'
                     and self.node.selected_tool in (
                         'dual_motor_gripper', 'spur_1motor_gripper')
                     and self._tool_motion_ready()
                     and self._gripper_positions_synchronized()
                     and (not dual or dual_ready))
        # When the measured position is outside the temporary range, expose
        # only the inward recovery direction.  This prevents a disabled
        # direction from being retried by either a click or a key shortcut.
        spur_open_allowed = True   # LEFT / '-' decreases ticks (opens)
        spur_close_allowed = True  # RIGHT / '+' increases ticks (closes)
        if self.node.selected_tool == 'spur_1motor_gripper':
            sample = self._gripper_samples().get(5, {})
            current = sample.get('position')
            if current is not None:
                if current > self.temporary_jog_safe_max:
                    spur_close_allowed = False
                elif current < self.temporary_jog_safe_min:
                    spur_open_allowed = False
        self.jog_close.setEnabled(jog_ready and spur_open_allowed)
        self.jog_open.setEnabled(jog_ready and spur_close_allowed)
        self.gripper_jog_step.setEnabled(not self.gripper_busy)
        cleaner = self.node.selected_tool == 'cleaner'
        configured = bool(self.tool_status.get('actuators_discovered'))
        self.clean_start.setEnabled(manual and cleaner and profile_ok
                                    and motion and configured)
        self.clean_stop.setEnabled(manual and cleaner and profile_ok and motion)
        for widget in (self.spur_minus_5, self.spur_zero, self.spur_plus_5):
            if widget is not None:
                widget.setEnabled(False)
        if spur:
            calibration_ready = (manual and calibration.get('active', False)
                                 and self.spur_torque_state == 'ON'
                                 and calibration.get('enabled', False)
                                 and bool(self.tool_status.get('calibration_jog_enabled'))
                                 and self._gripper_positions_synchronized())
            for widget in (self.motor_minus_half, self.motor_plus_half,
                           self.motor_minus_one, self.motor_plus_one,
                           self.capture_open, self.capture_close):
                widget.setEnabled(calibration_ready)
            manual_ready = self._spur_manual_ready()
            opening = self.profile.get('open_tick')
            closing = self.profile.get('close_tick')
            if isinstance(opening, (int, float)) and isinstance(closing, (int, float)):
                self.jog_close.setText(f'누르는 동안 열기 (틱 {"감소" if opening < closing else "증가"} → {opening})')
                self.jog_open.setText(f'누르는 동안 닫기 (틱 {"증가" if opening < closing else "감소"} → {closing})')
            self.open_button.setEnabled(manual_ready)
            self.close_button.setEnabled(manual_ready)
            self.jog_close.setEnabled(manual_ready)
            self.jog_open.setEnabled(manual_ready)
            for widget in (self.motor_minus_half, self.motor_plus_half,
                           self.motor_minus_one, self.motor_plus_one):
                widget.setEnabled(manual_ready)
            self.gripper_jog_step.setEnabled(False)
            captures = calibration.get('captures', {})
            both_captured = (set(captures) == {'open', 'close'}
                             and captures['open'] != captures['close'])
            self.validate_calibration.setEnabled(
                manual and calibration.get('active', False) and both_captured)
            self.save_calibration.setEnabled(
                manual and calibration.get('active', False)
                and calibration.get('validated', False))
        self.read_diag.setEnabled(spur and not self.mock_mode)
        self.start_cal.setEnabled(
            spur and manual and bool(self.tool_status.get('calibration_jog_enabled'))
            and not calibration.get('active', False))
        self._refresh_common_buttons()

    def _spur_manual_ready(self):
        return (self._spur_enable_ready()
                and self._gripper_samples().get(5, {}).get('torque_state') == 'ON')

    def _common_motion_ready(self):
        status = self.tool_status
        ids = self.profile.get('actuator_ids', [])
        samples = self._gripper_samples()
        return bool(self.node.selected_tool in ('dual_motor_gripper', 'spur_1motor_gripper')
                    and self.pending_tool_change is None
                    and self.control_mode == 'MANUAL'
                    and self.node.control_scope == 'END_EFFECTOR_ONLY'
                    and time.monotonic() - self.last_status_time < 1.5
                    and status.get('tool_type') == self.node.selected_tool
                    and status.get('profile_valid') and status.get('calibrated')
                    and not status.get('read_only') and not getattr(self.node, 'read_only', False)
                    and not status.get('emergency_stop') and not status.get('tool_detached')
                    and ids and all(samples.get(i, {}).get('online')
                        and samples[i].get('hardware_error') == 0
                        and samples[i].get('torque_state') == 'ON' for i in ids)
                    and self.fsm_state in ('READY', 'OPEN', 'CLOSED'))

    def _start_spur_hold(self, command):
        if not self._spur_manual_ready():
            return
        self.spur_hold_command = command
        self._repeat_spur_hold()
        self.spur_hold_timer.start()

    def _repeat_spur_hold(self):
        if not self._spur_manual_ready():
            self._release_spur_hold()
            return
        if self.spur_hold_command:
            self.node.command_calibration(self.spur_hold_command)

    def _release_spur_hold(self):
        self.spur_hold_timer.stop()
        active = self.spur_hold_command is not None
        self.spur_hold_command = None
        if active and self.node.selected_tool == 'spur_1motor_gripper':
            self.node.command_calibration('manual_hold')

    def _tool_motion_ready(self):
        fresh = time.monotonic() - self.last_status_time < 1.5
        scope_ok = self.tool_status.get('control_scope') == self.node.control_scope
        tool_type_ok = self.tool_status.get('tool_type') == self.node.selected_tool
        expected_ids = set(self.profile.get('actuator_ids', []))
        samples = self.tool_status.get('actuators', [])
        online_ids = {sample.get('id') for sample in samples
                      if sample.get('online')}
        actuators_ok = bool(expected_ids) and online_ids == expected_ids
        if self.node.selected_tool == 'cleaner' and self.mock_mode:
            actuators_ok = online_ids == expected_ids
        profile_ready = bool(self.tool_status.get('profile_valid')) \
            and (bool(self.tool_status.get('calibrated')) or self.mock_mode)
        temporary_ready = bool(self.tool_status.get('temporary_jog_ready')) \
            and self.node.temporary_jog_mode
        return (fresh and bool(self.tool_status.get('bridge_connected'))
                and bool(self.tool_status.get('motion_allowed')) and scope_ok
                and tool_type_ok
                and actuators_ok and (profile_ready or temporary_ready)
                and not bool(self.tool_status.get('read_only'))
                and not bool(self.tool_status.get('emergency_stop'))
                and not bool(self.tool_status.get('tool_detached')))

    def _tool_enable_ready(self):
        """Readiness before torque is enabled; used only by ENABLE ID5."""
        fresh = time.monotonic() - self.last_status_time < 1.5
        return (fresh and bool(self.tool_status.get('bridge_connected'))
                and bool(self.tool_status.get('online'))
                and self.tool_status.get('position') is not None
                and self.tool_status.get('hardware_error') == 0
                and not bool(self.tool_status.get('read_only'))
                and not bool(self.tool_status.get('emergency_stop'))
                and not bool(self.tool_status.get('tool_detached')))

    def _update_gripper_state(self, busy, state):
        self.gripper_busy = bool(busy)
        self.gripper_busy_label.setText(
            ko(f'BUSY: {state}' if busy else f'READY: {state}'))
        self.gripper_busy_label.setStyleSheet(
            FALSE_STYLE if busy else TRUE_STYLE)
        self._refresh_buttons()

    def _motor_endpoints(self):
        endpoints = self.profile.get('motor_endpoints', {})
        return {
            dxl_id: endpoints.get(dxl_id, endpoints.get(str(dxl_id)))
            for dxl_id in self.profile.get('actuator_ids', [])}

    def _gripper_samples(self):
        return {sample.get('id'): sample
                for sample in self.tool_status.get('actuators', [])}

    def _normalized_positions(self):
        samples = self._gripper_samples()
        fractions = {}
        for dxl_id, endpoint in self._motor_endpoints().items():
            sample = samples.get(dxl_id)
            if not endpoint or not sample or sample.get('position') is None:
                return {}
            span = endpoint['open'] - endpoint['close']
            if span == 0:
                return {}
            fractions[dxl_id] = (
                (float(sample['position']) - endpoint['close']) / span)
        return fractions

    def _gripper_positions_synchronized(self):
        if self.node.selected_tool == 'spur_1motor_gripper':
            sample = self._gripper_samples().get(5, {})
            return sample.get('position') is not None and bool(sample.get('online'))
        fractions = self._normalized_positions()
        return (len(fractions) == len(self.profile.get('actuator_ids', []))
                and max(fractions.values()) - min(fractions.values()) <= 0.05)

    def _update_gripper_feedback(self):
        if self.node.selected_tool == 'cleaner':
            return
        samples = self._gripper_samples()
        if self.node.selected_tool == 'spur_1motor_gripper':
            sample = samples.get(5, {})
            current = sample.get('position')
            target = self.gripper_target_ticks.get(5)
            error = None if current is None or target is None else target - current
            self.gripper_position_label.setText(
                ko(f'Spur Gripper | Current: {current} | Target: {target} '
                f'| Error: {error}'))
            self.gripper_feedback_label.setText(
                ko(f'ID5: current={current}, target={target}, error={error}, '
                f'current/load={sample.get("effort")}, '
                f'online={sample.get("online", False)}\n'
                f'Safe range: {self.temporary_jog_safe_min} ~ '
                f'{self.temporary_jog_safe_max}\n'
                f'Mechanical range: {self.temporary_jog_mechanical_open} ~ '
                f'{self.temporary_jog_mechanical_close}'))
            self.spur_torque_state = self.tool_status.get(
                'tool_torque_state', 'UNKNOWN')
            self.spur_torque_enabled = self.spur_torque_state == 'ON'
            self.spur_endpoints = dict((self.tool_status.get('calibration') or {}).get(
                'captures', self.spur_endpoints))
            self.captured_endpoints_label.setText(
                ko(f'Captured OPEN: {self.spur_endpoints.get("open", "—")} | '
                f'CLOSE: {self.spur_endpoints.get("close", "—")}'))
            self.spur_actual_state.setText(
                ko(f'ID5: position={current} torque={self.spur_torque_state} '
                f'load={sample.get("effort")} mode={sample.get("operating_mode")} '
                f'velocity={sample.get("profile_velocity")} '
                f'acceleration={sample.get("profile_acceleration")} '
                f'hardware_error={self.tool_status.get("hardware_error")} '
                f'model={self.tool_status.get("model")} '
                f'fsm={self.fsm_state} calibrated={self.tool_status.get("calibrated")}'))
            zero = 'UNSET' if self.spur_zero_tick is None else str(self.spur_zero_tick)
            self.spur_mapping.setText(
                ko('Output mapping: zero offset/reference tick=' + zero + '\n'
                'GUI output angle → motor tick: motor_deg = '
                f'output_deg × {self.spur_gear_ratio:.3f} / '
                f'({self.spur_output_direction:+d}); '
                'tick = zero + motor_deg × 4096 / 360.\n'
                'External spur pair: output direction is the inverse of motor direction.'))
            return
        fractions = self._normalized_positions()
        if fractions:
            normalized = sum(fractions.values()) / len(fractions)
            spread = max(fractions.values()) - min(fractions.values())
            self.gripper_position_label.setText(
                ko(f'Gripper position: {normalized:.4f} '
                f'(0.0=closed, 1.0=open, motor spread={spread:.4f})'))
            if not self.gripper_busy and spread > 0.05:
                self.gripper_busy_label.setText(
                    ko(f'BLOCKED: motor normalized spread {spread:.4f} > 0.0500'))
                self.gripper_busy_label.setStyleSheet(FALSE_STYLE)
        else:
            self.gripper_position_label.setText(ko('Gripper position: UNKNOWN'))
        lines = []
        for dxl_id in self.profile.get('actuator_ids', []):
            sample = samples.get(dxl_id, {})
            current = sample.get('position')
            target = self.gripper_target_ticks.get(dxl_id)
            error = None if current is None or target is None else target - current
            lines.append(
                f'ID{dxl_id}: current={current}, target={target}, '
                f'error={error}, current/load={sample.get("effort")}, '
                f'online={sample.get("online", False)}, '
                f'actual torque={sample.get("torque_state", "UNKNOWN")}, '
                f'hardware error={sample.get("hardware_error")}')
        self.gripper_feedback_label.setText(ko('\n'.join(lines) or 'No actuator data'))

    def _jog_gripper(self, direction):
        if self.pending_tool_change:
            return
        if self.node.selected_tool == 'spur_1motor_gripper':
            button = self.motor_minus_half if direction < 0 else self.motor_plus_half
            if button.isEnabled():
                self._jog_spur_motor(direction * 0.5)
            return
        reason = self._gripper_jog_block_reason()
        if reason:
            self._append_log(f'Gripper jog blocked: {reason}')
            return
        if self.node.selected_tool == 'spur_1motor_gripper':
            self._jog_spur(direction)
            return
        endpoints = self._motor_endpoints()
        fractions = self._normalized_positions()
        current = sum(fractions.values()) / len(fractions)
        spread = max(fractions.values()) - min(fractions.values())
        if spread > 0.05:
            self._append_log(
                f'Gripper jog blocked: motor normalized positions disagree '
                f'({fractions}, spread={spread:.4f})')
            return
        max_span = max(abs(ep['open'] - ep['close'])
                       for ep in endpoints.values())
        step = int(self.gripper_jog_step.currentText())
        target_fraction = min(1.0, max(
            0.0, current + direction * step / max_span))
        if abs(target_fraction - current) < 1e-9:
            self._append_log('Gripper jog blocked: already at profile boundary')
            return
        low = int(self.profile['safe_min_tick'])
        high = int(self.profile['safe_max_tick'])
        targets = {
            dxl_id: int(round(ep['close'] + target_fraction
                              * (ep['open'] - ep['close'])))
            for dxl_id, ep in endpoints.items()}
        outside = {dxl_id: target for dxl_id, target in targets.items()
                   if not low <= target <= high}
        if outside:
            self._append_log(
                f'Gripper jog blocked: targets outside [{low}, {high}]: '
                f'{outside}')
            return
        close_position = float(self.profile.get('close_position', 0.0))
        open_position = float(self.profile.get('open_position', 1.0))
        logical = close_position + target_fraction * (
            open_position - close_position)
        self._append_log(
            f'Gripper jog request: normalized={target_fraction:.6f}, '
            f'targets={targets}, step={step}')
        if self.node.command_gripper(logical):
            self.gripper_target_ticks = targets
            self._update_gripper_feedback()

    def _gripper_jog_block_reason(self):
        if (self.node.selected_tool == 'dual_motor_gripper'
                and (self.tool_status.get('dual_calibration') or {}).get('state')
                != 'READY'):
            return 'dual endpoint recalibration is required'
        if self.node.control_scope != 'END_EFFECTOR_ONLY':
            return 'control scope is not END_EFFECTOR_ONLY'
        if self.node.selected_tool not in (
                'dual_motor_gripper', 'spur_1motor_gripper'):
            return 'selected tool is not a supported gripper'
        if self.control_mode != 'MANUAL':
            return 'ownership is not MANUAL'
        if self.gripper_busy or self.node.gripper_busy:
            return 'BUSY'
        if not self._tool_motion_ready():
            return 'bridge/tool safety status is not ready or fresh'
        if self.node.selected_tool == 'spur_1motor_gripper':
            sample = self._gripper_samples().get(5, {})
            if sample.get('position') is None or not sample.get('online'):
                return 'ID5 position/online feedback unavailable'
            return ''
        if not self._normalized_positions():
            return 'current actuator positions are unavailable'
        if not self._gripper_positions_synchronized():
            return 'motor normalized positions are not synchronized'
        return ''

    def _jog_spur(self, direction):
        sample = self._gripper_samples().get(5, {})
        current = sample.get('position')
        step = int(self.gripper_jog_step.currentText())
        # Spur mapping: decreasing ticks opens, increasing ticks closes.
        target = int(current) + direction * step
        in_safe = self.temporary_jog_safe_min <= target <= self.temporary_jog_safe_max
        recovery = current < self.temporary_jog_safe_min or current > self.temporary_jog_safe_max
        inward = ((current > self.temporary_jog_safe_max and direction < 0)
                  or (current < self.temporary_jog_safe_min and direction > 0))
        if (not in_safe and not (recovery and inward)):
            self._append_log(
                f'Spur jog blocked: target={target} outside safe range '
                f'[{self.temporary_jog_safe_min}, {self.temporary_jog_safe_max}]')
            return
        if self.node.command_gripper(target):
            self.gripper_target_ticks = {5: target}
            self._update_gripper_feedback()

    def _jog_spur_motor(self, degrees):
        sample = self._gripper_samples().get(5, {})
        current = sample.get('position')
        if current is None or self.spur_torque_state != 'ON':
            self._append_log('Motor jog blocked: ID5 position/actual torque unavailable')
            return
        if self._spur_manual_ready() and self.node.command_calibration('manual_step', delta_deg=float(degrees)):
            self._append_log(f'ID5 CalibrationSession jog {degrees:+.1f}° requested')

    def _capture_spur_endpoint(self, label):
        sample = self._gripper_samples().get(5, {})
        current = sample.get('position')
        if current is None:
            self._append_log(f'Capture {label} blocked: ID5 position unavailable')
            return
        if self.node.command_calibration(f'capture_{label}'):
            self._append_log(f'CalibrationSession capture {label.upper()} requested (read only)')
            self._refresh_buttons()

    def _validate_spur_calibration(self):
        if self.node.command_calibration('validate'):
            self._append_log('CalibrationSession validation requested (no motor write)')

    def _save_spur_calibration(self):
        if self.node.command_calibration('save'):
            self._append_log(
                'Calibration save requested; bridge will atomically reload and require READY')

    def _command_tool(self, command):
        if self.pending_tool_change:
            return
        if self.node.selected_tool == 'dual_motor_gripper':
            self.node.command_tool_fsm(command)
            return
        if self.node.selected_tool == 'spur_1motor_gripper':
            self.node.command_tool_fsm(command)
            return

    def _start_dual_hold_jog(self, direction):
        if self.node.selected_tool != 'dual_motor_gripper':
            return
        if not self.open_button.isEnabled():
            self._append_log('Hold-to-run jog blocked by dual safety gate')
            return
        self.dual_hold_jog_active = True
        self.dual_hold_jog_direction = direction
        self._start_dual_key_jog(self._dual_jog_sign(direction))
        self._append_log(
            f'Dual hold-to-run {direction}: endpoint-ratio jog while held')
        self._refresh_buttons()

    def _release_dual_hold_jog(self):
        if not self.dual_hold_jog_active:
            return
        self._stop_dual_key_jog()
        self.dual_hold_jog_active = False
        self.dual_hold_jog_direction = None
        self._append_log('Dual hold-to-run released: current-position HOLD requested')
        self._refresh_buttons()

    def _stop_tool(self):
        if self.dual_key_jog_timer.isActive():
            self._stop_dual_key_jog()
        if self.node.selected_tool in ('spur_1motor_gripper', 'dual_motor_gripper'):
            self.node.command_tool_fsm('STOP')
            return
        if self.node.selected_tool == 'cleaner':
            self.node.command_cleaner(False)

    def _enable_spur_motor(self):
        sample = self._gripper_samples().get(5, {})
        current = sample.get('position')
        if current is None:
            self._append_log('ID5 Enable blocked: current tick feedback unavailable')
            return
        self.node.command_calibration('enable')
        self._append_log('CalibrationSession ENABLE ID5 requested')

    def _disable_spur_motor(self):
        self.node.command_calibration('disable')
        self._append_log('CalibrationSession DISABLE ID5 requested')

    def _enable_dual_motors(self):
        if self.node.set_dual_motor_enabled(True, self.profile.get('actuator_ids', [])):
            self._append_log('Operator requested dual torque enable for IDs [3, 4]')

    def _disable_dual_motors(self):
        if self.node.set_dual_motor_enabled(False, self.profile.get('actuator_ids', [])):
            self._append_log('Operator requested dual torque disable for IDs [3, 4]')

    def _manual_dual_recovery_jog(self, actuator_id, delta_deg):
        if self.node.manual_dual_recovery_jog(actuator_id, delta_deg):
            self._append_log(
                f'Operator requested one-click recovery jog: ID{actuator_id} '
                f'{delta_deg:+.1f}° (bridge re-reads actual position)')

    def _start_dual_calibration(self):
        if self.node.command_dual_calibration('start'):
            self._append_log('Dual endpoint calibration started (no motor write)')

    def _jog_dual_calibration_motor(self, actuator_id, direction):
        if self.dual_calibration_step is None:
            return
        degrees = float(self.dual_calibration_step.currentText()) * float(direction)
        if self.node.command_dual_calibration(
                'jog_motor_degrees', actuator_id=int(actuator_id),
                delta_deg=degrees):
            self._append_log(
                f'Dual calibration one-click jog requested: ID{actuator_id} '
                f'{degrees:+.1f}°')

    def _command_dual_calibration(self, command):
        if self.node.command_dual_calibration(command):
            self._append_log(f'Dual calibration command requested: {command}')

    def _command_spur_output_deg(self, output_deg):
        self._append_log('Output-angle command is unavailable during ID5 calibration')

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            event.ignore()
            return
        focus = self.focusWidget()
        editing = isinstance(
            focus, (QAbstractSpinBox, QLineEdit, QTextEdit, QComboBox))
        enabled = (self.pending_tool_change is None
                   and self.node.control_scope == 'END_EFFECTOR_ONLY'
                   and self.control_mode == 'MANUAL')
        if enabled and event.key() == Qt.Key_Space:
            self._stop_tool()
            event.accept()
            return
        if self.node.selected_tool == 'spur_1motor_gripper':
            if enabled and not editing and event.key() in (Qt.Key_Left, Qt.Key_Right):
                self._jog_gripper(-1 if event.key() == Qt.Key_Left else 1)
                event.accept()
                return
            event.ignore()
            return
        if (enabled and not editing
                and self.node.selected_tool == 'dual_motor_gripper'
                and event.key() in (Qt.Key_Left, Qt.Key_Right)):
            direction = (self._dual_jog_sign('OPEN') if event.key() == Qt.Key_Left
                         else self._dual_jog_sign('CLOSE'))
            self._start_dual_key_jog(direction)
            event.accept()
            return
        super().keyPressEvent(event)

    def eventFilter(self, watched, event):
        if event.type() == QEvent.WindowDeactivate and watched is self:
            self._release_spur_hold()
        if (event.type() == QEvent.WindowDeactivate
                and watched is self and self.dual_key_jog_timer.isActive()):
            self._stop_dual_key_jog()
        if event.type() not in (QEvent.KeyPress, QEvent.KeyRelease):
            return super().eventFilter(watched, event)
        if event.key() not in (Qt.Key_Left, Qt.Key_Right):
            return super().eventFilter(watched, event)
        if not self.isVisible():
            return super().eventFilter(watched, event)
        if (self.node.selected_tool != 'dual_motor_gripper'
                or self.node.control_scope != 'END_EFFECTOR_ONLY'
                or self.control_mode != 'MANUAL'):
            return super().eventFilter(watched, event)
        if event.isAutoRepeat():
            return True
        if event.type() == QEvent.KeyPress:
            direction = (self._dual_jog_sign('OPEN') if event.key() == Qt.Key_Left
                         else self._dual_jog_sign('CLOSE'))
            self._log_key_trace(
                f'arrow keyPress: key={event.key()} direction={direction}')
            self._start_dual_key_jog(direction)
        elif self.dual_key_jog_timer.isActive():
            self._log_key_trace(
                f'arrow keyRelease: key={event.key()} -> HOLD')
            self._stop_dual_key_jog()
        return True

    def keyReleaseEvent(self, event):
        if (not event.isAutoRepeat()
                and event.key() in (Qt.Key_Left, Qt.Key_Right)
                and self.dual_key_jog_timer.isActive()):
            self._stop_dual_key_jog()
            event.accept()
            return
        super().keyReleaseEvent(event)

    def _start_dual_key_jog(self, direction):
        if self.pending_tool_change or self.node.selected_tool != 'dual_motor_gripper':
            return
        if not self.open_button.isEnabled():
            return
        if self.dual_key_jog_timer.isActive():
            return
        self.dual_key_jog_direction = int(direction)
        self.dual_key_jog_timer.start()
        self._dual_key_jog_tick()

    @staticmethod
    def _dual_jog_sign(direction):
        # Shared physical convention for the hold buttons and arrow keys.
        return {'OPEN': 1, 'CLOSE': -1}[direction]

    def _dual_key_jog_tick(self):
        if self.dual_key_jog_direction not in (-1, 1):
            return
        if (self.control_mode != 'MANUAL'
                or self.node.selected_tool != 'dual_motor_gripper'
                or self.fsm_state in ('FAULT', 'STOPPED')
                or self.tool_status.get('emergency_stop')
                or self.tool_status.get('tool_detached')):
            self._stop_dual_key_jog()
            return
        command = 'JOG_OPEN' if self.dual_key_jog_direction > 0 else 'JOG_CLOSE'
        if not self.node.command_tool_fsm(command):
            self._stop_dual_key_jog()

    def _stop_dual_key_jog(self):
        self.dual_key_jog_timer.stop()
        self.dual_key_jog_direction = 0
        self.node.command_tool_fsm('HOLD')
        self.dual_hold_jog_active = False
        self.dual_hold_jog_direction = None
        self._append_log('Dual jog released: current-position HOLD requested')

    def closeEvent(self, event):
        self._release_spur_hold()
        if self.dual_key_jog_timer.isActive():
            self._stop_dual_key_jog()
        super().closeEvent(event)

    def _log_key_trace(self, message):
        get_logger = getattr(self.node, 'get_logger', None)
        if get_logger is not None:
            get_logger().info(message)

    def _jog(self, joint, sign):
        self.node.jog_arm(joint, sign * float(self.jog_step.currentText()))

    def _arm_target(self, joint):
        self.node.command_arm(joint, self.arm_targets[joint].value())

    def _request_mode(self):
        requested = self.mode_combo.currentData()
        self._append_log(
            f'Mode request clicked: requested={requested}, '
            f'approved={self.control_mode}')
        if (not self.mock_mode and requested == 'MANUAL'
                and self.fsm_state not in ToolManager.SAFE_CHANGE_STATES
                and not (self.node.selected_tool == 'spur_1motor_gripper'
                         and self.fsm_state in ('CALIBRATION_REQUIRED', 'STOPPED', 'READY'))
                and not (self.node.selected_tool == 'dual_motor_gripper'
                         and self.node.control_scope == 'END_EFFECTOR_ONLY')):
            QMessageBox.warning(
                self, ko('Ownership denied'),
                ko(f'MANUAL is allowed only in IDLE/STOWED; current={self.fsm_state}'))
            return
        self.node.request_mode(requested)

    def _request_tool_change(self):
        requested = self.tool_combo.currentData()
        current = self.tool_status.get('tool_type', self.node.selected_tool)
        if requested == current:
            self._append_log(f'{requested} is already selected')
            return
        if requested not in ('spur_1motor_gripper', 'dual_motor_gripper', 'cleaner'):
            QMessageBox.warning(self, ko('도구 변경 거부'),
                                ko('지원하지 않는 도구입니다.'))
            self.tool_combo.setCurrentIndex(self.tool_combo.findData(current))
            return
        if self.dual_key_jog_timer.isActive():
            self._stop_dual_key_jog()
        self.pending_tool_change = requested
        if not self.node.request_tool_change(requested):
            self.pending_tool_change = None
            self.tool_combo.setCurrentIndex(self.tool_combo.findData(current))
            return
        self._refresh_buttons()
        self._append_log(ko(f'도구 런타임 전환 요청: {requested}'))

    def _estop(self):
        self.node.emergency_stop()
        self.estop_state.setText(ko('E-STOP: REQUESTED'))
        self.estop_state.setStyleSheet(FALSE_STYLE)

    def _detach(self):
        answer = QMessageBox.question(
            self, ko('Confirm detach'), ko('Mark the current tool as DETACHED and stop it?'))
        if answer == QMessageBox.Yes:
            self.node.tool_detached()

    def _run_process(self, program, args):
        process = QProcess(self)
        process.setProgram(program)
        process.setArguments(args)
        process.readyReadStandardOutput.connect(
            lambda: self._append_log(bytes(
                process.readAllStandardOutput()).decode(errors='replace')))
        process.readyReadStandardError.connect(
            lambda: self._append_log(bytes(
                process.readAllStandardError()).decode(errors='replace')))
        process.finished.connect(lambda: self._append_log('Diagnostic process finished'))
        self.processes.append(process)
        process.start()

    def _read_only_diagnostic(self):
        if time.monotonic() - self.last_status_time < 1.5:
            self._append_log(
                'Bridge already owns the serial bus; using /tool/status read-only '
                f'diagnostics: {self.tool_status}')
            return
        ids = self.profile.get('actuator_ids', [5])
        self._run_process('ros2', [
            'run', 'dynamixel_control', 'spur_gripper_calibration',
            '--actuator-id', str(ids[0]), '--read-only'])

    def _start_calibration(self):
        if self.node.command_calibration('start'):
            self._append_log('CalibrationSession started (no register write)')

    def _rebuild_diagnostics(self, actuators, joint_values=None):
        joint_values = joint_values or {}
        rows = []
        for index, joint in enumerate(ARM_JOINTS):
            sample = joint_values.get(joint, {})
            position = sample.get('position', self.node.positions.get(joint))
            effort = sample.get('effort', self.node.efforts.get(joint))
            rows.append((index, joint, position, effort, position is not None))
        for sample in actuators:
            rows.append((sample.get('id'), sample.get('joint'),
                         sample.get('position'), sample.get('effort'),
                         sample.get('online', False)))
        self.diag.setRowCount(len(rows))
        for row, values in enumerate(rows):
            for column, value in enumerate(values):
                text = '—' if value is None else str(value)
                item = QTableWidgetItem(ko(text))
                if column == 4:
                    item.setForeground(Qt.darkGreen if value else Qt.red)
                self.diag.setItem(row, column, item)

    def _append_log(self, text):
        self.log.append(ko(str(text).strip()))
