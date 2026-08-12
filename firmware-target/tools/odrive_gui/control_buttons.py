"""
Control buttons widget for quick motor control operations.
Provides safety-critical control buttons with confirmation dialogs.
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QGroupBox, QMessageBox, QGridLayout
)
from PySide6.QtCore import Qt, Signal


class EmergencyStopButton(QPushButton):
    """Large emergency stop button."""

    triggered = Signal()

    def __init__(self, parent=None):
        super().__init__("🛑 急停 E-STOP", parent)
        self.setMinimumHeight(50)
        self.setStyleSheet("""
            QPushButton {
                background-color: #e64553;
                color: white;
                border: 2px solid #d20f39;
                border-radius: 8px;
                padding: 12px 24px;
                font-size: 16px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #d20f39;
                border-color: #e64553;
            }
            QPushButton:pressed {
                background-color: #b01030;
            }
        """)
        self.clicked.connect(self._on_clicked)

    def _on_clicked(self):
        reply = QMessageBox.warning(
            self.parent(), "确认急停",
            "⚠ 这将立即停止两个轴的电机！\n\n确定要急停吗？",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.triggered.emit()


class AxisControlButtons(QGroupBox):
    """Control button group for one axis."""

    def __init__(self, axis_num, worker, parent=None):
        super().__init__(f"M{axis_num} 控制", parent)
        self._axis_num = axis_num
        self._axis_name = f"axis{axis_num}"
        self._worker = worker
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(4)

        # Calibration buttons
        cal_group = QGroupBox("校准")
        cal_layout = QVBoxLayout(cal_group)

        motor_cal_btn = QPushButton("🔧 电机校准")
        motor_cal_btn.setToolTip("测量相电阻和相电感\n需要电机可自由旋转")
        motor_cal_btn.clicked.connect(lambda checked=False: self._request_state(4))
        cal_layout.addWidget(motor_cal_btn)

        encoder_cal_btn = QPushButton("📏 编码器偏移校准")
        encoder_cal_btn.setToolTip("校准编码器电角度偏移\n需要电机可自由旋转")
        encoder_cal_btn.clicked.connect(lambda checked=False: self._request_state(7))
        cal_layout.addWidget(encoder_cal_btn)

        idx_search_btn = QPushButton("🔍 编码器索引搜索")
        idx_search_btn.setToolTip("搜索编码器 Z 相索引脉冲")
        idx_search_btn.clicked.connect(lambda checked=False: self._request_state(6))
        cal_layout.addWidget(idx_search_btn)

        full_cal_btn = QPushButton("🔄 完整校准序列")
        full_cal_btn.setToolTip("依次执行: 电机校准 → 编码器偏移校准")
        full_cal_btn.clicked.connect(lambda checked=False: self._request_state(3))
        cal_layout.addWidget(full_cal_btn)

        layout.addWidget(cal_group)

        # Operation buttons
        op_group = QGroupBox("运行")
        op_layout = QVBoxLayout(op_group)

        closed_loop_btn = QPushButton("▶ 进入闭环控制")
        closed_loop_btn.setToolTip("启动 FOC 闭环控制\n前提: 电机已校准, 编码器已就绪")
        closed_loop_btn.setStyleSheet(
            "QPushButton { background-color: #a6e3a1; color: #1e1e2e; font-weight: bold; "
            "border: none; border-radius: 4px; padding: 8px; }"
            "QPushButton:hover { background-color: #94e2a0; }"
        )
        closed_loop_btn.clicked.connect(lambda checked=False: self._request_state(8))
        op_layout.addWidget(closed_loop_btn)

        sensorless_btn = QPushButton("🌀 无传感器控制")
        sensorless_btn.setToolTip("启动无传感器 FOC\n前提: 电机已校准")
        sensorless_btn.clicked.connect(lambda checked=False: self._request_state(5))
        op_layout.addWidget(sensorless_btn)

        idle_btn = QPushButton("⏸ 空闲 IDLE")
        idle_btn.setToolTip("停止电机 PWM 输出, 电机自由旋转")
        idle_btn.clicked.connect(lambda checked=False: self._request_state(1))
        op_layout.addWidget(idle_btn)

        layout.addWidget(op_group)

        # Utility buttons
        util_group = QGroupBox("工具")
        util_layout = QVBoxLayout(util_group)

        lockin_btn = QPushButton("🔄 Lockin Spin")
        lockin_btn.setToolTip("旋转锁定启动\n使用 general_lockin 参数")
        lockin_btn.clicked.connect(lambda checked=False: self._request_state(9))
        util_layout.addWidget(lockin_btn)

        homing_btn = QPushButton("🏠 Homing 回零")
        homing_btn.setToolTip("执行回零序列\n需要配置限位开关")
        homing_btn.clicked.connect(lambda checked=False: self._request_state(11))
        util_layout.addWidget(homing_btn)

        clear_errors_btn = QPushButton("🧹 清除错误")
        clear_errors_btn.setToolTip("清除此轴的所有错误标志")
        clear_errors_btn.setStyleSheet(
            "QPushButton { background-color: #f9e2af; color: #1e1e2e; "
            "border: none; border-radius: 4px; padding: 6px; }"
            "QPushButton:hover { background-color: #f5c890; }"
        )
        clear_errors_btn.clicked.connect(self._clear_errors)
        util_layout.addWidget(clear_errors_btn)

        layout.addWidget(util_group)
        layout.addStretch()

    def _request_state(self, state_value):
        """Request an axis state change with safety confirmation for dangerous states."""
        dangerous_states = {4: "电机校准", 7: "编码器偏移校准", 3: "完整校准序列",
                           9: "Lockin Spin", 11: "Homing 回零", 5: "无传感器控制"}
        if state_value in dangerous_states:
            reply = QMessageBox.question(
                self, "确认操作",
                f"即将执行: {dangerous_states[state_value]}\n\n"
                f"⚠ 电机会转动！确保安全后再继续。\n\n确定执行？",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No
            )
            if reply != QMessageBox.Yes:
                return

        path = f"{self._axis_name}.requested_state"
        self._worker.write_property(path, state_value)

    def _clear_errors(self, *_):
        """Clear all errors for this axis."""
        self._worker.call_function(f"{self._axis_name}.clear_errors")


class ControlButtonPanel(QWidget):
    """Combined control panel with E-Stop and per-axis buttons."""

    def __init__(self, worker, parent=None):
        super().__init__(parent)
        self._worker = worker
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(8)

        # Emergency stop (prominent, at top)
        self._estop = EmergencyStopButton()
        self._estop.triggered.connect(self._emergency_stop)
        layout.addWidget(self._estop)

        # Per-axis buttons in horizontal layout
        axes_layout = QHBoxLayout()
        self._m0_buttons = AxisControlButtons(0, self._worker)
        self._m1_buttons = AxisControlButtons(1, self._worker)
        axes_layout.addWidget(self._m0_buttons)
        axes_layout.addWidget(self._m1_buttons)
        layout.addLayout(axes_layout)

    def _emergency_stop(self):
        """Stop both axes immediately."""
        for i in [0, 1]:
            self._worker.write_property(f"axis{i}.requested_state", 1)  # IDLE
