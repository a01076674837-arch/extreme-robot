"""PyQt5 widgets for manual robot validation."""

import shlex
import time

from PyQt5.QtCore import QProcess, QTimer, Qt
from PyQt5.QtWidgets import (
    QAbstractSpinBox, QComboBox, QDoubleSpinBox, QFormLayout, QGridLayout,
    QGroupBox, QHBoxLayout, QLabel, QMainWindow, QMessageBox, QPushButton,
    QLineEdit, QTableWidget, QTableWidgetItem, QTextEdit, QVBoxLayout, QWidget)

from robot_manual_gui.ros_interface import ARM_JOINTS
from dynamixel_control.tool_manager import ToolManager


TRUE_STYLE = 'color: #0b7a25; font-weight: bold;'
FALSE_STYLE = 'color: #b00020; font-weight: bold;'
ESTOP_STYLE = 'background: #b00020; color: white; font-size: 20px; font-weight: bold;'


class ManualMainWindow(QMainWindow):
    """Hardware-test dashboard backed exclusively by ROS interfaces."""

    def __init__(self, ros_node, signals, profile, mock_mode=False):
        super().__init__()
        self.node = ros_node
        self.signals = signals
        self.profile = profile
        self.mock_mode = mock_mode
        self.tool_status = {}
        self.fsm_state = 'UNKNOWN'
        self.control_mode = 'FSM'
        self.last_status_time = 0.0
        self.processes = []
        self.joint_rows = {}
        self.seen_arm_joints = set()
        self.arm_widgets = {}
        self.gripper_busy = False
        self.gripper_target_ticks = {}
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
        self.setWindowTitle('Extreme Robot Manual Hardware Validation')
        self.resize(1180, 850)
        self._build_ui()
        self._connect_signals()
        self.watchdog = QTimer(self)
        self.watchdog.timeout.connect(self._refresh_connection)
        self.watchdog.start(500)

    def _build_ui(self):
        root = QWidget()
        outer = QVBoxLayout(root)

        self.scope_banner = QLabel(f'CONTROL / TEST SCOPE: {self.node.control_scope}')
        self.scope_banner.setAlignment(Qt.AlignCenter)
        self.scope_banner.setStyleSheet(
            'font-size: 22px; font-weight: bold; padding: 8px; '
            'background: #ffe08a; color: #202020;')
        outer.addWidget(self.scope_banner)

        safety = QHBoxLayout()
        self.estop = QPushButton('EMERGENCY STOP')
        self.estop.setMinimumHeight(62)
        self.estop.setStyleSheet(ESTOP_STYLE)
        self.estop.clicked.connect(self._estop)
        self.detach = QPushButton('TOOL DETACHED')
        self.detach.clicked.connect(self._detach)
        self.reset = QPushButton('RESET E-STOP (restart required)')
        self.reset.setEnabled(False)
        self.estop_state = QLabel('E-STOP: FALSE')
        self.estop_state.setStyleSheet(TRUE_STYLE)
        safety.addWidget(self.estop, 3)
        safety.addWidget(self.detach)
        safety.addWidget(self.reset)
        safety.addWidget(self.estop_state)
        outer.addLayout(safety)

        columns = QHBoxLayout()
        left = QVBoxLayout()
        right = QVBoxLayout()
        left.addWidget(self._status_group())
        left.addWidget(self._arm_group())
        right.addWidget(self._tool_selection_group())
        right.addWidget(self._tool_control_group())
        columns.addLayout(left, 3)
        columns.addLayout(right, 2)
        outer.addLayout(columns)

        self.diag = QTableWidget(0, 5)
        self.diag.setHorizontalHeaderLabels(
            ['ID', 'Joint', 'Position', 'Current/Load', 'Online'])
        outer.addWidget(self.diag)
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumHeight(120)
        outer.addWidget(self.log)
        self.setCentralWidget(root)

    def _status_group(self):
        box = QGroupBox('Connection / Status')
        form = QFormLayout(box)
        self.status_labels = {}
        for key, title in (
                ('connection', 'Bridge connection'),
                ('u2d2', 'U2D2 / serial'), ('tool_type', 'Tool type'),
                ('profile_valid', 'Profile valid'),
                ('actuators_discovered', 'Actuators discovered'),
                ('motion_allowed', 'Motion allowed'), ('fsm', 'FSM state'),
                ('arm_status', 'Arm contract state'), ('mode', 'Control mode'),
                ('contact', 'Contact sensor')):
            label = QLabel('UNKNOWN')
            self.status_labels[key] = label
            form.addRow(title, label)
        return box

    def _arm_group(self):
        box = QGroupBox('Arm Manual Control')
        layout = QGridLayout(box)
        layout.addWidget(QLabel('Joint'), 0, 0)
        layout.addWidget(QLabel('Current rad'), 0, 1)
        layout.addWidget(QLabel('Jog'), 0, 2, 1, 2)
        layout.addWidget(QLabel('Target rad'), 0, 4)
        self.arm_buttons = []
        self.arm_position_labels = {}
        self.arm_targets = {}
        for row, joint in enumerate(ARM_JOINTS, 1):
            label = QLabel('0.0000')
            minus = QPushButton('−')
            plus = QPushButton('+')
            target = QDoubleSpinBox()
            target.setRange(-6.283, 6.283)
            target.setDecimals(4)
            send = QPushButton('GO')
            minus.clicked.connect(
                lambda _checked=False, name=joint: self._jog(name, -1))
            plus.clicked.connect(
                lambda _checked=False, name=joint: self._jog(name, 1))
            send.clicked.connect(
                lambda _checked=False, name=joint: self._arm_target(name))
            layout.addWidget(QLabel(joint), row, 0)
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
        layout.addWidget(QLabel('Jog step (deg)'), 6, 0)
        layout.addWidget(self.jog_step, 6, 1)
        return box

    def _tool_selection_group(self):
        box = QGroupBox('Tool Selection / Ownership')
        form = QFormLayout(box)
        self.tool_combo = QComboBox()
        self.tool_combo.addItems([
            'dual_motor_gripper', 'spur_1motor_gripper', 'cleaner'])
        self.tool_combo.setCurrentText(self.node.selected_tool)
        request = QPushButton('REQUEST TOOL CHANGE')
        request.clicked.connect(self._request_tool_change)
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(['FSM', 'MANUAL'])
        mode_request = QPushButton('REQUEST MODE')
        mode_request.clicked.connect(self._request_mode)
        form.addRow('Selected tool', self.tool_combo)
        form.addRow('', request)
        form.addRow('Ownership', self.mode_combo)
        form.addRow('', mode_request)
        return box

    def _tool_control_group(self):
        box = QGroupBox('End Effector')
        layout = QVBoxLayout(box)
        self.profile_text = QLabel(self._profile_summary())
        self.profile_text.setWordWrap(True)
        layout.addWidget(self.profile_text)
        row = QHBoxLayout()
        self.open_button = QPushButton('OPEN')
        self.close_button = QPushButton('CLOSE')
        self.tool_stop = QPushButton('STOP')
        self.open_button.clicked.connect(lambda: self.node.command_gripper(
            float(self.profile.get('open_position', 1.0))))
        self.close_button.clicked.connect(lambda: self.node.command_gripper(
            float(self.profile.get('close_position', 0.0))))
        self.tool_stop.clicked.connect(self.node.stop_gripper)
        row.addWidget(self.open_button)
        row.addWidget(self.close_button)
        row.addWidget(self.tool_stop)
        layout.addLayout(row)
        jog = QGroupBox('GRIPPER JOG')
        jog_layout = QGridLayout(jog)
        if self.node.selected_tool == 'spur_1motor_gripper':
            left_label, right_label = 'LEFT / −  (OPEN)', 'RIGHT / +  (CLOSE)'
        else:
            left_label, right_label = 'LEFT / −  (CLOSE)', 'RIGHT / +  (OPEN)'
        self.jog_close = QPushButton(left_label)
        self.jog_open = QPushButton(right_label)
        self.gripper_jog_step = QComboBox()
        self.gripper_jog_step.addItems(['5', '10', '25', '50'])
        self.gripper_busy_label = QLabel('READY')
        self.gripper_position_label = QLabel('Gripper position: UNKNOWN')
        self.gripper_feedback_label = QLabel('ID3: UNKNOWN\nID4: UNKNOWN')
        self.gripper_feedback_label.setWordWrap(True)
        shortcut = QLabel(
            'Shortcuts: Left=CLOSE jog, Right=OPEN jog, Space=STOP\n'
            '(disabled while editing a field; key auto-repeat ignored)')
        shortcut.setWordWrap(True)
        self.jog_close.clicked.connect(lambda: self._jog_gripper(-1))
        self.jog_open.clicked.connect(lambda: self._jog_gripper(1))
        jog_layout.addWidget(self.jog_close, 0, 0)
        jog_layout.addWidget(self.jog_open, 0, 1)
        jog_layout.addWidget(QLabel('Step (tick equivalent)'), 1, 0)
        jog_layout.addWidget(self.gripper_jog_step, 1, 1)
        jog_layout.addWidget(self.gripper_busy_label, 2, 0, 1, 2)
        jog_layout.addWidget(self.gripper_position_label, 3, 0, 1, 2)
        jog_layout.addWidget(self.gripper_feedback_label, 4, 0, 1, 2)
        shortcut_row = 5
        if self.node.selected_tool == 'spur_1motor_gripper':
            jog_layout.addWidget(QLabel(
                f'Safe range: {self.temporary_jog_safe_min} ~ '
                f'{self.temporary_jog_safe_max}\n'
                f'Mechanical range: {self.temporary_jog_mechanical_open} ~ '
                f'{self.temporary_jog_mechanical_close}\n'
                'Direction: OPEN ← [−]   [+] → CLOSE'), 5, 0, 1, 2)
            shortcut_row = 6
        jog_layout.addWidget(shortcut, shortcut_row, 0, 1, 2)
        layout.addWidget(jog)
        cleaner = QHBoxLayout()
        self.clean_start = QPushButton('CLEANER START')
        self.clean_stop = QPushButton('CLEANER STOP')
        self.clean_start.clicked.connect(lambda: self.node.command_cleaner(True))
        self.clean_stop.clicked.connect(lambda: self.node.command_cleaner(False))
        cleaner.addWidget(self.clean_start)
        cleaner.addWidget(self.clean_stop)
        layout.addLayout(cleaner)
        calibration = QHBoxLayout()
        self.read_diag = QPushButton('READ ONLY DIAGNOSTIC')
        self.start_cal = QPushButton('START CALIBRATION')
        self.read_diag.clicked.connect(self._read_only_diagnostic)
        self.start_cal.clicked.connect(self._start_calibration)
        calibration.addWidget(self.read_diag)
        calibration.addWidget(self.start_cal)
        layout.addLayout(calibration)
        return box

    def _profile_summary(self):
        keys = ('calibrated', 'actuator_ids', 'safe_min_tick', 'safe_max_tick',
                'open_tick', 'close_tick', 'profile_velocity',
                'profile_acceleration')
        return '\n'.join(f'{key}: {self.profile.get(key)}' for key in keys)

    def _connect_signals(self):
        self.signals.joint_states.connect(self._update_joints)
        self.signals.tool_status.connect(self._update_tool_status)
        self.signals.fsm_state.connect(self._update_fsm)
        self.signals.control_mode.connect(self._update_mode)
        self.signals.arm_status.connect(
            lambda value: self.status_labels['arm_status'].setText(value))
        self.signals.contact_status.connect(
            lambda value: self._set_bool(self.status_labels['contact'], value))
        self.signals.log.connect(self._append_log)
        self.signals.gripper_state.connect(self._update_gripper_state)

    def _set_bool(self, label, value):
        label.setText('TRUE' if value else 'FALSE')
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
        self.status_labels['tool_type'].setText(status.get('tool_type', 'UNKNOWN'))
        self._set_bool(
            self.status_labels['u2d2'], bool(status.get('u2d2_connected')))
        for key in ('profile_valid', 'actuators_discovered', 'motion_allowed'):
            self._set_bool(self.status_labels[key], bool(status.get(key)))
        estop = bool(status.get('emergency_stop'))
        self.estop_state.setText(f'E-STOP: {str(estop).upper()}')
        self.estop_state.setStyleSheet(FALSE_STYLE if estop else TRUE_STYLE)
        self._refresh_buttons()
        self._rebuild_diagnostics(status.get('actuators', []))
        self._update_gripper_feedback()

    def _update_joints(self, values):
        for joint, sample in values.items():
            if joint in self.arm_position_labels and sample['position'] is not None:
                self.seen_arm_joints.add(joint)
                self.arm_position_labels[joint].setText(f'{sample["position"]:.4f}')
                self.arm_targets[joint].setValue(float(sample['position']))
        self._refresh_buttons()
        self._rebuild_diagnostics(self.tool_status.get('actuators', []), values)

    def _update_fsm(self, state):
        self.fsm_state = state
        self.status_labels['fsm'].setText(state)

    def _update_mode(self, mode):
        self.control_mode = mode
        self.status_labels['mode'].setText(mode)
        self._refresh_buttons()

    def _refresh_buttons(self):
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
        calibrated = bool(self.profile.get('calibrated')) or self.mock_mode
        preset_ready = (manual and gripper and profile_ok and motion
                        and calibrated and not self.gripper_busy)
        self.open_button.setEnabled(preset_ready)
        self.close_button.setEnabled(preset_ready)
        self.tool_stop.setEnabled(
            manual and gripper and profile_ok
            and (self.gripper_busy or motion))
        jog_ready = (manual and not self.gripper_busy
                     and self.node.control_scope == 'END_EFFECTOR_ONLY'
                     and self.node.selected_tool in (
                         'dual_motor_gripper', 'spur_1motor_gripper')
                     and self._tool_motion_ready()
                     and self._gripper_positions_synchronized())
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
        spur = self.node.selected_tool == 'spur_1motor_gripper'
        self.read_diag.setEnabled(spur and not self.mock_mode)
        self.start_cal.setEnabled(spur and not self.mock_mode)

    def _tool_motion_ready(self):
        fresh = time.monotonic() - self.last_status_time < 1.5
        scope_ok = self.tool_status.get('control_scope') == self.node.control_scope
        expected_ids = set(self.profile.get('actuator_ids', []))
        samples = self.tool_status.get('actuators', [])
        online_ids = {sample.get('id') for sample in samples
                      if sample.get('online')}
        actuators_ok = bool(expected_ids) and online_ids == expected_ids
        profile_ready = bool(self.tool_status.get('profile_valid')) \
            and bool(self.tool_status.get('calibrated'))
        temporary_ready = bool(self.tool_status.get('temporary_jog_ready')) \
            and self.node.temporary_jog_mode
        return (fresh and bool(self.tool_status.get('bridge_connected'))
                and bool(self.tool_status.get('motion_allowed')) and scope_ok
                and actuators_ok and (profile_ready or temporary_ready)
                and not bool(self.tool_status.get('read_only'))
                and not bool(self.tool_status.get('emergency_stop'))
                and not bool(self.tool_status.get('tool_detached')))

    def _update_gripper_state(self, busy, state):
        self.gripper_busy = bool(busy)
        self.gripper_busy_label.setText(
            f'BUSY: {state}' if busy else f'READY: {state}')
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
        samples = self._gripper_samples()
        if self.node.selected_tool == 'spur_1motor_gripper':
            sample = samples.get(5, {})
            current = sample.get('position')
            target = self.gripper_target_ticks.get(5)
            error = None if current is None or target is None else target - current
            self.gripper_position_label.setText(
                f'Spur Gripper | Current: {current} | Target: {target} '
                f'| Error: {error}')
            self.gripper_feedback_label.setText(
                f'ID5: current={current}, target={target}, error={error}, '
                f'current/load={sample.get("effort")}, '
                f'online={sample.get("online", False)}\n'
                f'Safe range: {self.temporary_jog_safe_min} ~ '
                f'{self.temporary_jog_safe_max}\n'
                f'Mechanical range: {self.temporary_jog_mechanical_open} ~ '
                f'{self.temporary_jog_mechanical_close}')
            return
        fractions = self._normalized_positions()
        if fractions:
            normalized = sum(fractions.values()) / len(fractions)
            spread = max(fractions.values()) - min(fractions.values())
            self.gripper_position_label.setText(
                f'Gripper position: {normalized:.4f} '
                f'(0.0=closed, 1.0=open, motor spread={spread:.4f})')
            if not self.gripper_busy and spread > 0.05:
                self.gripper_busy_label.setText(
                    f'BLOCKED: motor normalized spread {spread:.4f} > 0.0500')
                self.gripper_busy_label.setStyleSheet(FALSE_STYLE)
        else:
            self.gripper_position_label.setText('Gripper position: UNKNOWN')
        lines = []
        for dxl_id in self.profile.get('actuator_ids', []):
            sample = samples.get(dxl_id, {})
            current = sample.get('position')
            target = self.gripper_target_ticks.get(dxl_id)
            error = None if current is None or target is None else target - current
            lines.append(
                f'ID{dxl_id}: current={current}, target={target}, '
                f'error={error}, current/load={sample.get("effort")}, '
                f'online={sample.get("online", False)}')
        self.gripper_feedback_label.setText('\n'.join(lines) or 'No actuator data')

    def _jog_gripper(self, direction):
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

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            event.ignore()
            return
        focus = self.focusWidget()
        editing = isinstance(
            focus, (QAbstractSpinBox, QLineEdit, QTextEdit, QComboBox))
        enabled = (self.node.control_scope == 'END_EFFECTOR_ONLY'
                   and self.control_mode == 'MANUAL')
        if enabled and event.key() == Qt.Key_Space:
            self.node.stop_gripper()
            event.accept()
            return
        if enabled and not editing and event.key() == Qt.Key_Left:
            self._jog_gripper(-1)
            event.accept()
            return
        if enabled and not editing and event.key() == Qt.Key_Right:
            self._jog_gripper(1)
            event.accept()
            return
        super().keyPressEvent(event)

    def _jog(self, joint, sign):
        self.node.jog_arm(joint, sign * float(self.jog_step.currentText()))

    def _arm_target(self, joint):
        self.node.command_arm(joint, self.arm_targets[joint].value())

    def _request_mode(self):
        requested = self.mode_combo.currentText()
        self._append_log(
            f'Mode request clicked: requested={requested}, '
            f'approved={self.control_mode}')
        if (requested == 'MANUAL'
                and self.fsm_state not in ToolManager.SAFE_CHANGE_STATES):
            QMessageBox.warning(
                self, 'Ownership denied',
                f'MANUAL is allowed only in IDLE/STOWED; current={self.fsm_state}')
            return
        self.node.request_mode(requested)

    def _request_tool_change(self):
        requested = self.tool_combo.currentText()
        current = self.tool_status.get('tool_type', self.node.selected_tool)
        if requested == current:
            self._append_log(f'{requested} is already selected')
            return
        if self.fsm_state not in ToolManager.SAFE_CHANGE_STATES:
            QMessageBox.warning(
                self, 'Tool change denied',
                f'ToolManager policy denies changes in {self.fsm_state}')
            self.tool_combo.setCurrentText(current)
            return
        QMessageBox.information(
            self, 'Restart required',
            'Runtime hardware reprovisioning is not implemented. Stop the launch, '
            f'detach safely, then restart with tool_type:={requested}.')
        self.tool_combo.setCurrentText(current)

    def _estop(self):
        self.node.emergency_stop()
        self.estop_state.setText('E-STOP: REQUESTED')
        self.estop_state.setStyleSheet(FALSE_STYLE)

    def _detach(self):
        answer = QMessageBox.question(
            self, 'Confirm detach', 'Mark the current tool as DETACHED and stop it?')
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
        answer = QMessageBox.warning(
            self, 'Powered calibration confirmation',
            'Calibration can move the gripper. Stop the bridge first, clear the '
            'mechanism, prepare emergency power-off, and continue in a terminal. '
            'Launch the guarded calibration terminal now?',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if answer != QMessageBox.Yes:
            return
        if time.monotonic() - self.last_status_time < 1.5:
            QMessageBox.critical(
                self, 'Serial bus still owned',
                'The bridge is still running. Stop the bridge/stack first; calibration '
                'will not be launched while another serial owner is active.')
            return
        actuator_id = self.profile.get('actuator_ids', [5])[0]
        command = (
            'source /opt/ros/humble/setup.bash; '
            'source /home/asd/extreme-robot/ros2_ws/install/setup.bash; '
            'ros2 run dynamixel_control spur_gripper_calibration '
            f'--actuator-id {shlex.quote(str(actuator_id))} --armed')
        self._run_process('x-terminal-emulator', ['-e', 'bash', '-lc', command])

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
                item = QTableWidgetItem(text)
                if column == 4:
                    item.setForeground(Qt.darkGreen if value else Qt.red)
                self.diag.setItem(row, column, item)

    def _append_log(self, text):
        self.log.append(str(text).strip())
