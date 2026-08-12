"""
Axis control panel with tabbed sub-panels for motor control.
Each tab provides a different aspect of axis control.
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QGroupBox, QGridLayout, QTabWidget, QScrollArea,
    QDoubleSpinBox, QComboBox, QSlider, QLineEdit
)
from PySide6.QtCore import Qt, QTimer

import odrive.enums


class AxisControlPanel(QWidget):
    """Complete control panel for one motor axis."""

    STATE_BUTTONS = [
        (1, "空闲 IDLE", "停止电机 PWM 输出"),
        (4, "电机校准", "测量相电阻和相电感"),
        (6, "编码器索引搜索", "搜索编码器 Z 相索引"),
        (7, "编码器偏移校准", "校准编码器电角度偏移"),
        (3, "完整校准序列", "运行全部校准流程"),
        (8, "闭环控制", "进入 FOC 闭环控制"),
        (9, "Lockin Spin", "旋转锁定启动"),
        (5, "无传感器控制", "运行无传感器 FOC"),
        (11, "Homing 回零", "执行回零序列"),
    ]

    CONTROL_MODES = [
        (0, "电压控制"),
        (1, "扭矩控制"),
        (2, "速度控制"),
        (3, "位置控制"),
    ]

    INPUT_MODES = [
        (1, "直通 Passthrough"),
        (2, "速度斜坡 VelRamp"),
        (3, "位置滤波 PosFilter"),
        (5, "梯形轨迹 TrapTraj"),
        (6, "扭矩斜坡 TorqueRamp"),
        (7, "镜像 Mirror"),
    ]

    def __init__(self, axis_num, worker, parent=None):
        super().__init__(parent)
        self._axis_num = axis_num
        self._worker = worker
        self._axis_name = f"axis{axis_num}"
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(4)

        # Sub-tabs
        self._tabs = QTabWidget()
        self._tabs.addTab(self._create_dashboard_tab(), "📊 仪表盘")
        self._tabs.addTab(self._create_control_tab(), "🎮 控制")
        self._tabs.addTab(self._create_commands_tab(), "⚡ 设定值")
        self._tabs.addTab(self._create_params_tab(), "🔧 参数")
        layout.addWidget(self._tabs)

    def _create_dashboard_tab(self):
        """Per-axis quick-view dashboard."""
        w = QWidget()
        layout = QGridLayout(w)
        layout.setSpacing(4)

        labels_def = [
            ("状态:", 'state', '—'),
            ("位置:", 'pos', '0.000 turn'),
            ("速度:", 'vel', '0.00 turn/s'),
            ("Iq 电流:", 'iq', '0.00 A'),
            ("Id 电流:", 'id', '0.00 A'),
            ("FET 温度:", 'fet_temp', '0.0 °C'),
            ("电机温度:", 'mot_temp', '0.0 °C'),
            ("轴错误:", 'axis_error', '0'),
            ("电机错误:", 'motor_error', '0'),
            ("编码器错误:", 'encoder_error', '0'),
            ("控制器错误:", 'controller_error', '0'),
            ("已校准:", 'calibrated', '否'),
            ("编码器就绪:", 'ready', '否'),
        ]
        self._dash_labels = {}
        for row, (label, key, default) in enumerate(labels_def):
            layout.addWidget(QLabel(label), row, 0)
            lbl = QLabel(default)
            lbl.setStyleSheet("font-family: Consolas; font-weight: bold;")
            layout.addWidget(lbl, row, 1)
            self._dash_labels[key] = lbl

        return w

    def _create_control_tab(self):
        """State machine and mode control."""
        w = QScrollArea()
        w.setWidgetResizable(True)
        inner = QWidget()
        layout = QVBoxLayout(inner)
        layout.setSpacing(8)

        # State transition buttons
        state_group = QGroupBox("状态切换")
        state_layout = QVBoxLayout(state_group)
        for state_val, state_name, tooltip in self.STATE_BUTTONS:
            btn = QPushButton(f"{state_name}")
            btn.setToolTip(tooltip)
            btn.clicked.connect(lambda checked, v=state_val, n=state_name: self._set_state(v, n))
            state_layout.addWidget(btn)
        layout.addWidget(state_group)

        # Control mode selection
        mode_group = QGroupBox("控制模式")
        mode_layout = QVBoxLayout(mode_group)

        self._control_mode_combo = QComboBox()
        for val, name in self.CONTROL_MODES:
            self._control_mode_combo.addItem(name, val)
        self._control_mode_combo.currentIndexChanged.connect(self._on_control_mode_changed)
        mode_layout.addWidget(QLabel("控制模式:"))
        mode_layout.addWidget(self._control_mode_combo)

        self._input_mode_combo = QComboBox()
        for val, name in self.INPUT_MODES:
            self._input_mode_combo.addItem(name, val)
        self._input_mode_combo.currentIndexChanged.connect(self._on_input_mode_changed)
        mode_layout.addWidget(QLabel("输入模式:"))
        mode_layout.addWidget(self._input_mode_combo)
        layout.addWidget(mode_group)

        # Clear errors
        clear_btn = QPushButton("🔄 清除错误")
        clear_btn.setStyleSheet("QPushButton { background-color: #f9e2af; color: #1e1e2e; border: none; border-radius: 4px; padding: 8px; font-weight: bold; } QPushButton:hover { background-color: #f5c890; }")
        clear_btn.clicked.connect(self._clear_errors)
        layout.addWidget(clear_btn)

        layout.addStretch()
        w.setWidget(inner)
        return w

    def _create_commands_tab(self):
        """Setpoint input and motion commands."""
        w = QWidget()
        layout = QVBoxLayout(w)
        layout.setSpacing(8)

        # Position setpoint
        pos_group = QGroupBox("位置控制")
        pos_layout = QGridLayout(pos_group)
        pos_layout.addWidget(QLabel("设定位置 (turn):"), 0, 0)
        self._pos_spin = QDoubleSpinBox()
        self._pos_spin.setRange(-10000, 10000)
        self._pos_spin.setDecimals(3)
        self._pos_spin.setValue(0)
        self._pos_spin.setSingleStep(0.1)
        pos_layout.addWidget(self._pos_spin, 0, 1)
        pos_btn = QPushButton("执行 move_to_pos")
        pos_btn.clicked.connect(self._move_to_pos)
        pos_layout.addWidget(pos_btn, 1, 0, 1, 2)
        layout.addWidget(pos_group)

        # Velocity setpoint
        vel_group = QGroupBox("速度控制")
        vel_layout = QGridLayout(vel_group)
        vel_layout.addWidget(QLabel("设定速度 (turn/s):"), 0, 0)
        self._vel_spin = QDoubleSpinBox()
        self._vel_spin.setRange(-1000, 1000)
        self._vel_spin.setDecimals(3)
        self._vel_spin.setValue(0)
        self._vel_spin.setSingleStep(0.5)
        vel_layout.addWidget(self._vel_spin, 0, 1)
        vel_btn = QPushButton("设置速度")
        vel_btn.clicked.connect(self._set_velocity)
        vel_layout.addWidget(vel_btn, 1, 0, 1, 2)
        layout.addWidget(vel_group)

        # Torque setpoint
        torque_group = QGroupBox("扭矩控制")
        torque_layout = QGridLayout(torque_group)
        torque_layout.addWidget(QLabel("设定扭矩 (Nm):"), 0, 0)
        self._torque_spin = QDoubleSpinBox()
        self._torque_spin.setRange(-100, 100)
        self._torque_spin.setDecimals(4)
        self._torque_spin.setValue(0)
        self._torque_spin.setSingleStep(0.01)
        torque_layout.addWidget(self._torque_spin, 0, 1)
        torque_btn = QPushButton("设置扭矩")
        torque_btn.clicked.connect(self._set_torque)
        torque_layout.addWidget(torque_btn, 1, 0, 1, 2)
        layout.addWidget(torque_group)

        layout.addStretch()
        return w

    def _create_params_tab(self):
        """Quick-access parameter editor for motor and controller gains."""
        w = QScrollArea()
        w.setWidgetResizable(True)
        inner = QWidget()
        layout = QVBoxLayout(inner)
        layout.setSpacing(8)

        # Motor parameters
        motor_group = QGroupBox("电机参数")
        motor_grid = QGridLayout(motor_group)
        motor_params = [
            ("current_lim", "电流限制", "A", 0, 200, 10.0),
            ("calibration_current", "校准电流", "A", 0, 100, 10.0),
            ("pole_pairs", "极对数", "", 1, 100, 7),
            ("torque_constant", "扭矩常数", "Nm/A", 0, 10, 0.04),
            ("requested_current_range", "电流量程", "A", 0, 200, 60.0),
            ("current_control_bandwidth", "电流环带宽", "rad/s", 0, 10000, 1000.0),
        ]
        self._motor_spins = {}
        for row, (key, name, unit, vmin, vmax, default) in enumerate(motor_params):
            motor_grid.addWidget(QLabel(name + ":"), row, 0)
            spin = QDoubleSpinBox()
            spin.setRange(vmin, vmax)
            spin.setDecimals(4)
            spin.setValue(default)
            spin.setSuffix(f" {unit}")
            motor_grid.addWidget(spin, row, 1)
            apply_btn = QPushButton("应用")
            apply_btn.clicked.connect(lambda checked, k=key, s=spin: self._set_motor_param(k, s.value()))
            motor_grid.addWidget(apply_btn, row, 2)
            self._motor_spins[key] = spin
        layout.addWidget(motor_group)

        # Controller parameters
        ctrl_group = QGroupBox("控制器增益")
        ctrl_grid = QGridLayout(ctrl_group)
        ctrl_params = [
            ("pos_gain", "位置 P 增益", "(turn/s)/turn", 0, 1000, 20.0),
            ("vel_gain", "速度 P 增益", "Nm/(turn/s)", 0, 100, 0.1667),
            ("vel_integrator_gain", "速度 I 增益", "Nm/(turn*s)", 0, 100, 0.3333),
            ("vel_limit", "速度限制", "turn/s", 0, 10000, 2.0),
        ]
        self._ctrl_spins = {}
        for row, (key, name, unit, vmin, vmax, default) in enumerate(ctrl_params):
            ctrl_grid.addWidget(QLabel(name + ":"), row, 0)
            spin = QDoubleSpinBox()
            spin.setRange(vmin, vmax)
            spin.setDecimals(4)
            spin.setValue(default)
            spin.setSuffix(f" {unit}")
            ctrl_grid.addWidget(spin, row, 1)
            apply_btn = QPushButton("应用")
            apply_btn.clicked.connect(lambda checked, k=key, s=spin: self._set_ctrl_param(k, s.value()))
            ctrl_grid.addWidget(apply_btn, row, 2)
            self._ctrl_spins[key] = spin
        layout.addWidget(ctrl_group)

        layout.addStretch()
        w.setWidget(inner)
        return w

    # --- Control Actions ---

    def _set_state(self, state_value, state_name):
        """Request an axis state change."""
        path = f"{self._axis_name}.requested_state"
        self._worker.write_property(path, state_value)

    def _on_control_mode_changed(self, index):
        mode = self._control_mode_combo.currentData()
        path = f"{self._axis_name}.controller.config.control_mode"
        self._worker.write_property(path, mode)

    def _on_input_mode_changed(self, index):
        mode = self._input_mode_combo.currentData()
        path = f"{self._axis_name}.controller.config.input_mode"
        self._worker.write_property(path, mode)

    def _move_to_pos(self, *_):
        pos = self._pos_spin.value()
        self._worker.write_property(
            f"{self._axis_name}.controller.input_pos", pos)

    def _set_velocity(self, *_):
        vel = self._vel_spin.value()
        self._worker.write_property(
            f"{self._axis_name}.controller.input_vel", vel)

    def _set_torque(self, *_):
        torque = self._torque_spin.value()
        self._worker.write_property(
            f"{self._axis_name}.controller.input_torque", torque)

    def _clear_errors(self, *_):
        """Clear all errors on this axis."""
        self._worker.call_function(f"{self._axis_name}.clear_errors")

    def _set_motor_param(self, key, value):
        path = f"{self._axis_name}.motor.config.{key}"
        self._worker.write_property(path, value)

    def _set_ctrl_param(self, key, value):
        path = f"{self._axis_name}.controller.config.{key}"
        self._worker.write_property(path, value)

    # --- Telemetry Update ---

    def update_telemetry(self, data):
        """Update per-axis dashboard labels."""
        prefix = f'm{self._axis_num}'
        from odrive_gui.odrive_worker import ODriveWorker

        state_val = data.get(f'{prefix}_current_state', 0)
        self._dash_labels['state'].setText(
            ODriveWorker.get_state_name(state_val))
        self._dash_labels['pos'].setText(
            f"{data.get(f'{prefix}_pos', 0):.3f} turn")
        self._dash_labels['vel'].setText(
            f"{data.get(f'{prefix}_vel', 0):.2f} turn/s")
        self._dash_labels['iq'].setText(
            f"{data.get(f'{prefix}_iq', 0):.2f} A")
        self._dash_labels['id'].setText(
            f"{data.get(f'{prefix}_id', 0):.3f} A")
        self._dash_labels['fet_temp'].setText(
            f"{data.get(f'{prefix}_fet_temp', 0):.1f} °C")
        self._dash_labels['mot_temp'].setText(
            f"{data.get(f'{prefix}_motor_temp', 0):.1f} °C")

        # Error flags
        ae = data.get(f'{prefix}_axis_error', 0)
        me = data.get(f'{prefix}_motor_error', 0)
        ee = data.get(f'{prefix}_encoder_error', 0)
        ce = data.get(f'{prefix}_controller_error', 0)
        self._dash_labels['axis_error'].setText(
            f"0x{ae:08X}" if ae else "✓ OK")
        self._dash_labels['axis_error'].setStyleSheet(
            f"color: {'#f38ba8' if ae else '#a6e3a1'};")
        self._dash_labels['motor_error'].setText(
            f"0x{me:08X}" if me else "✓ OK")
        self._dash_labels['motor_error'].setStyleSheet(
            f"color: {'#f38ba8' if me else '#a6e3a1'};")
        self._dash_labels['encoder_error'].setText(
            f"0x{ee:08X}" if ee else "✓ OK")
        self._dash_labels['encoder_error'].setStyleSheet(
            f"color: {'#f38ba8' if ee else '#a6e3a1'};")
        self._dash_labels['controller_error'].setText(
            f"0x{ce:08X}" if ce else "✓ OK")
        self._dash_labels['controller_error'].setStyleSheet(
            f"color: {'#f38ba8' if ce else '#a6e3a1'};")

        # Status
        is_cal = data.get(f'{prefix}_is_calibrated', False)
        is_ready = data.get(f'{prefix}_is_ready', False)
        self._dash_labels['calibrated'].setText("是 ✓" if is_cal else "否 ✗")
        self._dash_labels['calibrated'].setStyleSheet(
            f"color: {'#a6e3a1' if is_cal else '#f38ba8'};")
        self._dash_labels['ready'].setText("是 ✓" if is_ready else "否 ✗")
        self._dash_labels['ready'].setStyleSheet(
            f"color: {'#a6e3a1' if is_ready else '#f38ba8'};")
