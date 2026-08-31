"""PyQt5 widgets for manual robot validation."""

import shlex
import time

from PyQt5.QtCore import QEvent, QProcess, QTimer, Qt
from PyQt5.QtWidgets import (
    QApplication, QAbstractSpinBox, QComboBox, QDoubleSpinBox, QFormLayout, QGridLayout,
    QGroupBox, QHBoxLayout, QLabel, QMainWindow, QMessageBox, QPushButton,
    QLineEdit, QTableWidget, QTableWidgetItem, QTextEdit, QVBoxLayout, QWidget)

from robot_manual_gui.ros_interface import ARM_JOINTS
from dynamixel_control.tool_manager import ToolManager
from dynamixel_control.single_motor_endpoints import jog_limits as single_jog_limits


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
        self.dual_single_motor_test_mode = bool(getattr(
            self.node, 'dual_single_motor_test_mode', False))
        self.dual_manual_test_mode = bool(getattr(
            self.node, 'dual_manual_test_mode', False))
        self.held_motor_direction = 0
        self.held_motor_key = None
        self.single_preset_target = None
        self.motor_repeat = QTimer(self)
        self.motor_repeat.setInterval(150)
        self.motor_repeat.timeout.connect(self._repeat_single_motor_jog)
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
        self.setWindowTitle('익스트림 로봇 수동 하드웨어 검증')
        self.resize(1180, 850)
        self._build_ui()
        # Filter the QApplication itself so Q/W reaches the jog controller
        # regardless of which current or subsequently-created widget has focus.
        self.application = QApplication.instance()
        if self.application is not None:
            self.application.installEventFilter(self)
        self._connect_signals()
        self.watchdog = QTimer(self)
        self.watchdog.timeout.connect(self._refresh_connection)
        self.watchdog.start(500)

    def _build_ui(self):
        root = QWidget()
        outer = QVBoxLayout(root)

        scope_names = {
            'END_EFFECTOR_ONLY': '엔드 이펙터만',
            'FULL_ROBOT': '전체 로봇',
        }
        scope = scope_names.get(self.node.control_scope, self.node.control_scope)
        self.scope_banner = QLabel(f'제어 / 시험 범위: {scope}')
        self.scope_banner.setAlignment(Qt.AlignCenter)
        self.scope_banner.setStyleSheet(
            'font-size: 22px; font-weight: bold; padding: 8px; '
            'background: #ffe08a; color: #202020;')
        outer.addWidget(self.scope_banner)

        safety = QHBoxLayout()
        self.estop = QPushButton('긴급 정지')
        self.estop.setMinimumHeight(62)
        self.estop.setStyleSheet(ESTOP_STYLE)
        self.estop.clicked.connect(self._estop)
        self.detach = QPushButton('도구 분리')
        self.detach.clicked.connect(self._detach)
        self.reset = QPushButton('긴급 정지 해제 (재시작 필요)')
        self.reset.setEnabled(False)
        self.estop_state = QLabel('긴급 정지: 해제')
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
            ['ID', '관절', '위치', '전류/부하', '연결'])
        outer.addWidget(self.diag)
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumHeight(120)
        outer.addWidget(self.log)
        self.setCentralWidget(root)

    def _status_group(self):
        box = QGroupBox('연결 / 상태')
        form = QFormLayout(box)
        self.status_labels = {}
        for key, title in (
                ('connection', '브리지 연결'),
                ('u2d2', 'U2D2 / 직렬 통신'), ('tool_type', '도구 유형'),
                ('profile_valid', '프로파일 유효'),
                ('actuators_discovered', '구동기 감지'),
                ('motion_allowed', '동작 허용'), ('fsm', 'FSM 상태'),
                ('arm_status', '로봇팔 계약 상태'), ('mode', '제어 모드'),
                ('contact', '접촉 센서')):
            label = QLabel('확인 중')
            self.status_labels[key] = label
            form.addRow(title, label)
        return box

    def _arm_group(self):
        box = QGroupBox('로봇팔 수동 제어')
        layout = QGridLayout(box)
        layout.addWidget(QLabel('관절'), 0, 0)
        layout.addWidget(QLabel('현재값(rad)'), 0, 1)
        layout.addWidget(QLabel('조그'), 0, 2, 1, 2)
        layout.addWidget(QLabel('목표값(rad)'), 0, 4)
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
            send = QPushButton('이동')
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
        layout.addWidget(QLabel('조그 간격(도)'), 6, 0)
        layout.addWidget(self.jog_step, 6, 1)
        return box

    def _tool_selection_group(self):
        box = QGroupBox('도구 선택 / 제어권')
        form = QFormLayout(box)
        self.tool_combo = QComboBox()
        self.tool_combo.addItems([
            'dual_motor_gripper', 'spur_1motor_gripper', 'cleaner'])
        self.tool_combo.setCurrentText(self.node.selected_tool)
        request = QPushButton('도구 변경 요청')
        request.clicked.connect(self._request_tool_change)
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(['FSM', 'MANUAL'])
        mode_request = QPushButton('모드 변경 요청')
        mode_request.clicked.connect(self._request_mode)
        form.addRow('선택한 도구', self.tool_combo)
        form.addRow('', request)
        form.addRow('제어권', self.mode_combo)
        form.addRow('', mode_request)
        return box

    def _tool_control_group(self):
        box = QGroupBox('엔드 이펙터')
        layout = QVBoxLayout(box)
        self.profile_text = QLabel(self._profile_summary())
        self.profile_text.setWordWrap(True)
        layout.addWidget(self.profile_text)
        row = QHBoxLayout()
        self.open_button = QPushButton('열기')
        self.close_button = QPushButton('닫기')
        self.tool_stop = QPushButton('정지')
        if self.dual_single_motor_test_mode:
            self.open_button.clicked.connect(
                lambda: self._start_single_motor_preset('open'))
            self.close_button.clicked.connect(
                lambda: self._start_single_motor_preset('close'))
        else:
            self.open_button.clicked.connect(lambda: self.node.command_gripper(
                float(self.profile.get('open_position', 1.0))))
            self.close_button.clicked.connect(lambda: self.node.command_gripper(
                float(self.profile.get('close_position', 0.0))))
        self.tool_stop.clicked.connect(self.node.stop_gripper)
        row.addWidget(self.open_button)
        row.addWidget(self.close_button)
        row.addWidget(self.tool_stop)
        layout.addLayout(row)
        jog = QGroupBox('그리퍼 조그')
        jog_layout = QGridLayout(jog)
        if self.node.selected_tool == 'spur_1motor_gripper':
            left_label, right_label = '← / −  (열기)', '→ / +  (닫기)'
        elif self.dual_manual_test_mode:
            left_label, right_label = 'Q  열림 (두 모터)', 'W  닫힘 (두 모터)'
        else:
            left_label, right_label = '← / −  (닫기)', '→ / +  (열기)'
        self.jog_close = QPushButton(left_label)
        self.jog_open = QPushButton(right_label)
        self.gripper_jog_step = QComboBox()
        self.gripper_jog_step.addItems(['5', '10', '25', '50'])
        self.gripper_busy_label = QLabel('준비')
        self.gripper_position_label = QLabel('그리퍼 위치: 확인 중')
        self.gripper_feedback_label = QLabel('ID3: 확인 중\nID4: 확인 중')
        self.gripper_feedback_label.setWordWrap(True)
        shortcut = QLabel(
            (('키보드: Q=열림, W=닫힘 (두 모터 normalized 동기화)\n'
              if self.dual_manual_test_mode else
              '키보드: Q=ID3 tick 5 감소, W=ID3 tick 5 증가\n')
             + '(누르는 동안 150ms 간격 반복, 키를 떼면 즉시 정지)'))
        shortcut.setWordWrap(True)
        if self.dual_single_motor_test_mode or self.dual_manual_test_mode:
            # Mouse holds use exactly the same press/repeat/release path as Q/W.
            close_direction = 1 if self.dual_manual_test_mode else -1
            open_direction = -1 if self.dual_manual_test_mode else 1
            close_key = 'Q' if self.dual_manual_test_mode else 'Q'
            open_key = 'W' if self.dual_manual_test_mode else 'W'
            self.jog_close.pressed.connect(
                lambda: self._start_motor_jog(close_direction, close_key))
            self.jog_close.released.connect(
                lambda: self._release_motor_jog(close_direction, close_key))
            self.jog_open.pressed.connect(
                lambda: self._start_motor_jog(open_direction, open_key))
            self.jog_open.released.connect(
                lambda: self._release_motor_jog(open_direction, open_key))
        else:
            self.jog_close.clicked.connect(lambda: self._jog_gripper(-1))
            self.jog_open.clicked.connect(lambda: self._jog_gripper(1))
        jog_layout.addWidget(self.jog_close, 0, 0)
        jog_layout.addWidget(self.jog_open, 0, 1)
        jog_layout.addWidget(QLabel('간격(틱 기준)'), 1, 0)
        jog_layout.addWidget(self.gripper_jog_step, 1, 1)
        jog_layout.addWidget(self.gripper_busy_label, 2, 0, 1, 2)
        jog_layout.addWidget(self.gripper_position_label, 3, 0, 1, 2)
        jog_layout.addWidget(self.gripper_feedback_label, 4, 0, 1, 2)
        shortcut_row = 5
        if self.node.selected_tool == 'spur_1motor_gripper':
            jog_layout.addWidget(QLabel(
                f'안전 범위: {self.temporary_jog_safe_min} ~ '
                f'{self.temporary_jog_safe_max}\n'
                f'기계적 범위: {self.temporary_jog_mechanical_open} ~ '
                f'{self.temporary_jog_mechanical_close}\n'
                '방향: 열기 ← [−]   [+] → 닫기'), 5, 0, 1, 2)
            shortcut_row = 6
        jog_layout.addWidget(shortcut, shortcut_row, 0, 1, 2)
        self.endpoint_label = QLabel()
        self.save_open_endpoint = QPushButton('현재 ID3 위치를 열림 최대로 저장')
        self.save_close_endpoint = QPushButton('현재 ID3 위치를 닫힘 최대로 저장')
        self.save_open_endpoint.clicked.connect(
            lambda: self._save_single_endpoint('open'))
        self.save_close_endpoint.clicked.connect(
            lambda: self._save_single_endpoint('close'))
        endpoint_row = shortcut_row + 1
        jog_layout.addWidget(self.endpoint_label, endpoint_row, 0, 1, 2)
        jog_layout.addWidget(self.save_open_endpoint, endpoint_row + 1, 0, 1, 2)
        jog_layout.addWidget(self.save_close_endpoint, endpoint_row + 2, 0, 1, 2)
        for widget in (self.endpoint_label, self.save_open_endpoint,
                       self.save_close_endpoint):
            widget.setVisible(self.dual_single_motor_test_mode)
        self._refresh_endpoint_label()
        layout.addWidget(jog)
        cleaner = QHBoxLayout()
        self.clean_start = QPushButton('클리너 시작')
        self.clean_stop = QPushButton('클리너 정지')
        self.clean_start.clicked.connect(lambda: self.node.command_cleaner(True))
        self.clean_stop.clicked.connect(lambda: self.node.command_cleaner(False))
        cleaner.addWidget(self.clean_start)
        cleaner.addWidget(self.clean_stop)
        layout.addLayout(cleaner)
        calibration = QHBoxLayout()
        self.read_diag = QPushButton('읽기 전용 진단')
        self.start_cal = QPushButton('캘리브레이션 시작')
        self.read_diag.clicked.connect(self._read_only_diagnostic)
        self.start_cal.clicked.connect(self._start_calibration)
        calibration.addWidget(self.read_diag)
        calibration.addWidget(self.start_cal)
        layout.addLayout(calibration)
        return box

    def _profile_summary(self):
        fields = (
            ('calibrated', '캘리브레이션 완료'),
            ('actuator_ids', '구동기 ID'),
            ('safe_min_tick', '안전 최소 틱'),
            ('safe_max_tick', '안전 최대 틱'),
            ('open_tick', '열림 틱'),
            ('close_tick', '닫힘 틱'),
            ('profile_velocity', '프로파일 속도'),
            ('profile_acceleration', '프로파일 가속도'),
        )
        return '\n'.join(
            f'{label}: {self.profile.get(key)}' for key, label in fields)

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
        label.setText('정상' if value else '아님')
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
        self.status_labels['tool_type'].setText(status.get('tool_type', '확인 중'))
        self._set_bool(
            self.status_labels['u2d2'], bool(status.get('u2d2_connected')))
        for key in ('profile_valid', 'actuators_discovered', 'motion_allowed'):
            self._set_bool(self.status_labels[key], bool(status.get(key)))
        estop = bool(status.get('emergency_stop'))
        self.estop_state.setText(
            f'긴급 정지: {"작동" if estop else "해제"}')
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
        if mode != 'MANUAL':
            self._stop_motor_repeat()
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
        endpoints = getattr(self.node, 'single_motor_endpoints', {})
        self.open_button.setEnabled(
            preset_ready and (not self.dual_single_motor_test_mode
                              or endpoints.get('open_tick') is not None))
        self.close_button.setEnabled(
            preset_ready and (not self.dual_single_motor_test_mode
                              or endpoints.get('close_tick') is not None))
        self.tool_stop.setEnabled(
            manual and gripper and profile_ok
            and (self.gripper_busy or motion))
        jog_ready = (manual and not self.gripper_busy
                     and self.node.control_scope == 'END_EFFECTOR_ONLY'
                     and self.node.selected_tool in (
                         'dual_motor_gripper', 'spur_1motor_gripper')
                     and self._tool_motion_ready()
                     and (self.dual_single_motor_test_mode
                          or self._gripper_positions_synchronized()))
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
        endpoint_save_ready = self.dual_single_motor_test_mode \
            and self._single_motor_test_ready()
        self.save_open_endpoint.setEnabled(endpoint_save_ready)
        self.save_close_endpoint.setEnabled(endpoint_save_ready)
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
        single_motor_torque_ok = (
            not self.dual_single_motor_test_mode
            or (self.tool_status.get('dual_single_motor_test_mode')
                and set(self.tool_status.get('torque_enabled_ids', [])) == {3}))
        return (fresh and bool(self.tool_status.get('bridge_connected'))
                and bool(self.tool_status.get('motion_allowed')) and scope_ok
                and actuators_ok and (profile_ready or temporary_ready)
                and not bool(self.tool_status.get('read_only'))
                and not bool(self.tool_status.get('emergency_stop'))
                and not bool(self.tool_status.get('tool_detached'))
                and single_motor_torque_ok)

    def _update_gripper_state(self, busy, state):
        self.gripper_busy = bool(busy)
        self.gripper_busy_label.setText(
            f'동작 중: {state}' if busy else f'준비: {state}')
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
        # ID4 is deliberately torque-free in this diagnostic mode.  Its
        # position is expected to diverge and must never gate ID3 jogging.
        if self.dual_single_motor_test_mode or self.dual_manual_test_mode:
            sample = self._gripper_samples().get(3, {})
            return sample.get('position') is not None and bool(sample.get('online'))
        if self.node.selected_tool == 'spur_1motor_gripper':
            sample = self._gripper_samples().get(5, {})
            return sample.get('position') is not None and bool(sample.get('online'))
        fractions = self._normalized_positions()
        return (len(fractions) == len(self.profile.get('actuator_ids', []))
                and max(fractions.values()) - min(fractions.values()) <= 0.05)

    def _update_gripper_feedback(self):
        samples = self._gripper_samples()
        if self.dual_single_motor_test_mode:
            id3 = samples.get(3, {})
            id4 = samples.get(4, {})
            current = id3.get('position')
            target = self.gripper_target_ticks.get(3)
            error = None if current is None or target is None else target - current
            self.gripper_position_label.setText(
                f'ID3 단독 제어 | 현재: {current} | 목표: {target} '
                f'| 오차: {error}')
            self.gripper_feedback_label.setText(
                f'ID3: Torque ON, 현재={current}, 목표={target}, '
                f'전류/부하={id3.get("effort")}, '
                f'연결={id3.get("online", False)}\n'
                f'ID4: Torque OFF / 수동 회전, 현재={id4.get("position")}, '
                f'연결={id4.get("online", False)}\n'
                '단일모터 시험: ID3/ID4 정규화 편차 검사 미적용')
            return
        if self.node.selected_tool == 'spur_1motor_gripper':
            sample = samples.get(5, {})
            current = sample.get('position')
            target = self.gripper_target_ticks.get(5)
            error = None if current is None or target is None else target - current
            self.gripper_position_label.setText(
                f'스퍼 그리퍼 | 현재: {current} | 목표: {target} '
                f'| 오차: {error}')
            self.gripper_feedback_label.setText(
                f'ID5: 현재={current}, 목표={target}, 오차={error}, '
                f'전류/부하={sample.get("effort")}, '
                f'연결={sample.get("online", False)}\n'
                f'안전 범위: {self.temporary_jog_safe_min} ~ '
                f'{self.temporary_jog_safe_max}\n'
                f'기계적 범위: {self.temporary_jog_mechanical_open} ~ '
                f'{self.temporary_jog_mechanical_close}')
            return
        fractions = self._normalized_positions()
        if fractions:
            normalized = sum(fractions.values()) / len(fractions)
            spread = max(fractions.values()) - min(fractions.values())
            self.gripper_position_label.setText(
                f'그리퍼 위치: {normalized:.4f} '
                f'(0.0=닫힘, 1.0=열림, 모터 편차={spread:.4f})')
            if not self.gripper_busy and spread > 0.05:
                self.gripper_busy_label.setText(
                    f'차단: 모터 정규화 편차 {spread:.4f} > 0.0500')
                self.gripper_busy_label.setStyleSheet(FALSE_STYLE)
        else:
            self.gripper_position_label.setText('그리퍼 위치: 확인 중')
        lines = []
        for dxl_id in self.profile.get('actuator_ids', []):
            sample = samples.get(dxl_id, {})
            current = sample.get('position')
            target = self.gripper_target_ticks.get(dxl_id)
            error = None if current is None or target is None else target - current
            normalized = fractions.get(dxl_id)
            lines.append(
                f'ID{dxl_id}: Present={current}, Goal={target}, 오차={error}, '
                f'normalized={normalized}, Velocity={sample.get("velocity")}, '
                f'Current/Load={sample.get("effort")}, '
                f'Torque={sample.get("torque", sample.get("torque_state"))}, '
                f'Mode={sample.get("operating_mode")}, '
                f'HardwareError={sample.get("hardware_error")}, '
                f'연결={sample.get("online", False)}')
        self.gripper_feedback_label.setText(
            '\n'.join(lines) or '구동기 데이터 없음')

    def _jog_gripper(self, direction):
        if self.dual_single_motor_test_mode:
            self._jog_single_id3(direction)
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

    def _jog_single_id3(self, direction, final_target=None):
        if not self._single_motor_test_ready():
            self._append_log(
                'ID3 조그 차단: ID3 ON / ID4 OFF 상태가 준비되지 않음')
            self._stop_motor_repeat()
            return
        sample = self._gripper_samples().get(3, {})
        current = sample.get('position')
        if current is None:
            self._stop_motor_repeat()
            return
        saved = getattr(self.node, 'single_motor_endpoints', {})
        low, high, _calibrated = single_jog_limits(saved)
        # Always step from fresh measured feedback.  Building on the previous
        # requested target can outrun a slow motor during a long key hold.
        base = int(current)
        step = int(self.gripper_jog_step.currentText())
        distance = step if final_target is None else min(
            step, abs(int(final_target) - base))
        target = base + direction * distance
        if ((base < low and direction < 0)
                or (base > high and direction > 0)
                or (low <= base <= high and not low <= target <= high)):
            self._append_log(
                f'ID3 조그 차단: target={target}, 안전 범위=[{low}, {high}]')
            self._stop_motor_repeat()
            return
        if target == base:
            self._stop_motor_repeat()
            return
        if self.node.command_single_motor_tick(target):
            self.gripper_target_ticks[3] = target
            self._update_gripper_feedback()

    def _single_motor_test_ready(self):
        return bool(
            self.dual_single_motor_test_mode
            and self.control_mode == 'MANUAL'
            and self._tool_motion_ready()
            and self.tool_status.get('dual_single_motor_test_mode')
            and set(self.tool_status.get('torque_enabled_ids', [])) == {3}
            and self._gripper_samples().get(3, {}).get('online')
            and self._gripper_samples().get(4, {}).get('online'))

    def _repeat_single_motor_jog(self):
        if self.dual_manual_test_mode:
            if self.held_motor_direction:
                self._jog_gripper(self.held_motor_direction)
            return
        if self.single_preset_target is not None:
            current = self._gripper_samples().get(3, {}).get('position')
            if current is None or int(current) == self.single_preset_target:
                self._stop_motor_repeat()
                return
            direction = 1 if self.single_preset_target > current else -1
            self._jog_single_id3(direction, self.single_preset_target)
            return
        if self.held_motor_direction:
            self._trace_jog(f'{self.held_motor_key} repeat')
            self._jog_single_id3(self.held_motor_direction)

    def _stop_motor_repeat(self):
        self.held_motor_direction = 0
        self.held_motor_key = None
        self.single_preset_target = None
        self.motor_repeat.stop()

    def _trace_jog(self, message):
        """Emit requested key traces to the launch terminal (and test stdout)."""
        get_logger = getattr(self.node, 'get_logger', None)
        if get_logger is not None:
            get_logger().info(message)
        else:
            print(message, flush=True)

    def _start_motor_jog(self, direction, key_name):
        """Perform one immediate jog and start our own stable repeat timer."""
        if not (self.dual_single_motor_test_mode or self.dual_manual_test_mode):
            self._jog_gripper(direction)
            return
        if (self.held_motor_direction == direction
                and self.motor_repeat.isActive()):
            return
        self._stop_motor_repeat()
        self.held_motor_direction = direction
        self.held_motor_key = key_name
        self._trace_jog(f'{key_name} pressed')
        self._jog_single_id3(direction)
        # A blocked immediate movement stops the timer through the safety path.
        if self.held_motor_direction == direction:
            self.motor_repeat.start(150)

    def _release_motor_jog(self, direction, key_name):
        """Stop only the direction associated with the physical release."""
        if self.held_motor_direction != direction:
            return
        self._trace_jog(f'{key_name} released')
        self._stop_motor_repeat()

    def _start_single_motor_preset(self, kind):
        target = getattr(self.node, 'single_motor_endpoints', {}).get(
            f'{kind}_tick')
        if target is None or not self._single_motor_test_ready():
            self._append_log(f'{kind} 최대 위치/안전 상태 미준비')
            return
        self._stop_motor_repeat()
        self.single_preset_target = int(target)
        self.motor_repeat.start()
        self._repeat_single_motor_jog()

    def _refresh_endpoint_label(self):
        endpoints = getattr(self.node, 'single_motor_endpoints', {})
        opened = endpoints.get('open_tick')
        closed = endpoints.get('close_tick')
        safe = ('미설정' if opened is None or closed is None else
                f'{min(opened, closed)} ~ {max(opened, closed)}')
        self.endpoint_label.setText(
            f'열림 최대: {opened} | 닫힘 최대: {closed}\n'
            f'저장된 안전범위: {safe}')

    def _save_single_endpoint(self, kind):
        sample = self._gripper_samples().get(3, {})
        current = sample.get('position')
        if current is None or not self._single_motor_test_ready():
            self._append_log('ID3 위치를 저장할 수 없음: 피드백/안전 상태 미준비')
            return
        try:
            endpoints = self.node.save_single_motor_endpoint(kind, int(current))
        except (OSError, ValueError) as exc:
            self._append_log(f'ID3 최대 위치 저장 실패: {exc}')
            return
        self.gripper_target_ticks.clear()
        self._refresh_endpoint_label()
        self._append_log(
            f'ID3 {kind} 최대 저장: {current}, endpoints={endpoints}')
        self._refresh_buttons()

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
        if self.dual_single_motor_test_mode:
            return '' if self._single_motor_test_ready() \
                else 'ID3 torque / ID4 free-wheel state is not ready'
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

    def _handle_motor_key(self, event):
        """Handle one safety-gated gripper shortcut; return True if consumed."""
        minus_key = Qt.Key_Q if self.dual_single_motor_test_mode else Qt.Key_Left
        plus_key = Qt.Key_W if self.dual_single_motor_test_mode else Qt.Key_Right
        if event.isAutoRepeat():
            # OS/Qt repeat presses and synthetic repeat releases must never
            # restart or interrupt the single application-owned timer.
            event.accept()
            return event.key() in (minus_key, plus_key, Qt.Key_Space)
        focus = self.focusWidget()
        editing = isinstance(
            focus, (QAbstractSpinBox, QLineEdit, QTextEdit, QComboBox))
        enabled = (self.node.control_scope == 'END_EFFECTOR_ONLY'
                   and self.control_mode == 'MANUAL')
        if enabled and event.key() == Qt.Key_Space:
            self._stop_motor_repeat()
            self.node.stop_gripper()
            event.accept()
            return True
        allow_jog = (self.dual_single_motor_test_mode
                     or self.dual_manual_test_mode or not editing)
        if self.dual_manual_test_mode:
            if enabled and allow_jog and event.key() == Qt.Key_Q:
                self._start_motor_jog(1, 'Q')
                event.accept()
                return True
            if enabled and allow_jog and event.key() == Qt.Key_W:
                self._start_motor_jog(-1, 'W')
                event.accept()
                return True
            return False
        if enabled and allow_jog and event.key() == minus_key:
            self._start_motor_jog(-1, 'Q' if self.dual_single_motor_test_mode else 'LEFT')
            event.accept()
            return True
        if enabled and allow_jog and event.key() == plus_key:
            self._start_motor_jog(1, 'W' if self.dual_single_motor_test_mode else 'RIGHT')
            event.accept()
            return True
        return False

    def eventFilter(self, watched, event):
        if event.type() == QEvent.KeyPress and self._handle_motor_key(event):
            return True
        if (event.type() == QEvent.KeyRelease
                and event.key() in self._motor_jog_keys()
                and (self.dual_single_motor_test_mode
                     or self.dual_manual_test_mode)):
            if event.isAutoRepeat():
                event.accept()
                return True
            direction = ((1 if event.key() == Qt.Key_Q else -1)
                         if self.dual_manual_test_mode else
                         (-1 if event.key() == Qt.Key_Q else 1))
            self._release_motor_jog(
                direction, 'Q' if direction < 0 else 'W')
            event.accept()
            return True
        return super().eventFilter(watched, event)

    def keyReleaseEvent(self, event):
        if (event.key() in self._motor_jog_keys()
                and not event.isAutoRepeat()
                and (self.dual_single_motor_test_mode
                     or self.dual_manual_test_mode)):
            direction = ((1 if event.key() == Qt.Key_Q else -1)
                         if self.dual_manual_test_mode else
                         (-1 if event.key() == Qt.Key_Q else 1))
            self._release_motor_jog(
                direction, 'Q' if direction < 0 else 'W')
            event.accept()
            return
        super().keyReleaseEvent(event)

    def _motor_jog_keys(self):
        if self.dual_single_motor_test_mode or self.dual_manual_test_mode:
            return (Qt.Key_Q, Qt.Key_W)
        return (Qt.Key_Left, Qt.Key_Right)

    def keyPressEvent(self, event):
        if self._handle_motor_key(event):
            return
        super().keyPressEvent(event)

    def closeEvent(self, event):
        self._stop_motor_repeat()
        if self.application is not None:
            self.application.removeEventFilter(self)
        super().closeEvent(event)

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
                self, '제어권 요청 거부',
                '수동 모드는 IDLE/STOWED 상태에서만 허용됩니다. '
                f'현재 상태: {self.fsm_state}')
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
                self, '도구 변경 거부',
                f'ToolManager 정책상 {self.fsm_state} 상태에서는 '
                '변경할 수 없습니다.')
            self.tool_combo.setCurrentText(current)
            return
        QMessageBox.information(
            self, '재시작 필요',
            '실행 중 하드웨어 재설정은 지원하지 않습니다. '
            '런치를 종료하고 도구를 안전하게 분리한 뒤 '
            f'tool_type:={requested}로 재시작하세요.')
        self.tool_combo.setCurrentText(current)

    def _estop(self):
        self.node.emergency_stop()
        self.estop_state.setText('긴급 정지: 요청됨')
        self.estop_state.setStyleSheet(FALSE_STYLE)

    def _detach(self):
        answer = QMessageBox.question(
            self, '도구 분리 확인',
            '현재 도구를 분리 상태로 표시하고 정지할까요?')
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
        process.finished.connect(lambda: self._append_log('진단 프로세스 종료'))
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
            self, '전원 캘리브레이션 확인',
            '캘리브레이션 중 그리퍼가 움직일 수 있습니다. '
            '먼저 브리지를 정지하고, 기구 주변을 비우고, '
            '비상 전원 차단을 준비하세요. '
            '보호된 캘리브레이션 터미널을 실행할까요?',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if answer != QMessageBox.Yes:
            return
        if time.monotonic() - self.last_status_time < 1.5:
            QMessageBox.critical(
                self, '직렬 버스 사용 중',
                '브리지가 아직 실행 중입니다. 먼저 브리지/스택을 '
                '종료하세요. 다른 프로세스가 직렬 버스를 사용하는 '
                '동안은 캘리브레이션을 실행하지 않습니다.')
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
