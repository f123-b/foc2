"""
Dashboard panel showing real-time system telemetry.
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QGroupBox, QGridLayout, QProgressBar
)
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont


class DashboardPanel(QGroupBox):
    """Real-time telemetry dashboard for both axes."""

    STATE_COLORS = {
        'IDLE': '#6c7086',
        'CLOSED_LOOP_CONTROL': '#a6e3a1',
        'MOTOR_CALIBRATION': '#f9e2af',
        'ENCODER_OFFSET_CALIBRATION': '#f9e2af',
        'SENSORLESS_CONTROL': '#89b4fa',
        'FULL_CALIBRATION_SEQUENCE': '#f9e2af',
    }

    def __init__(self, parent=None):
        super().__init__("系统仪表盘", parent)
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(6)

        # System-level metrics row
        sys_row = QHBoxLayout()

        # VBus
        vbus_group = QGroupBox("直流母线")
        vbus_layout = QVBoxLayout(vbus_group)
        self._vbus_label = QLabel("— V")
        self._vbus_label.setFont(QFont("Consolas", 18, QFont.Bold))
        self._vbus_label.setAlignment(Qt.AlignCenter)
        self._vbus_label.setStyleSheet("color: #89b4fa;")
        vbus_layout.addWidget(self._vbus_label)
        self._vbus_bar = QProgressBar()
        self._vbus_bar.setRange(0, 60)
        self._vbus_bar.setValue(0)
        self._vbus_bar.setTextVisible(False)
        vbus_layout.addWidget(self._vbus_bar)
        sys_row.addWidget(vbus_group)

        # IBus
        ibus_group = QGroupBox("母线电流")
        ibus_layout = QVBoxLayout(ibus_group)
        self._ibus_label = QLabel("— A")
        self._ibus_label.setFont(QFont("Consolas", 18, QFont.Bold))
        self._ibus_label.setAlignment(Qt.AlignCenter)
        self._ibus_label.setStyleSheet("color: #f9e2af;")
        ibus_layout.addWidget(self._ibus_label)
        sys_row.addWidget(ibus_group)

        # Brake status
        brake_group = QGroupBox("制动电阻")
        brake_layout = QVBoxLayout(brake_group)
        self._brake_armed = QLabel("—")
        self._brake_armed.setFont(QFont("", 12))
        self._brake_armed.setAlignment(Qt.AlignCenter)
        brake_layout.addWidget(self._brake_armed)
        self._brake_sat = QLabel("—")
        self._brake_sat.setFont(QFont("", 10))
        self._brake_sat.setAlignment(Qt.AlignCenter)
        brake_layout.addWidget(self._brake_sat)
        sys_row.addWidget(brake_group)

        layout.addLayout(sys_row)

        # Per-axis telemetry
        axes_row = QHBoxLayout()
        self._axis_groups = []
        for i, name in enumerate(["M0", "M1"]):
            group = QGroupBox(f"{name} 轴状态")
            grid = QGridLayout(group)
            grid.setSpacing(2)

            # Row 0: State
            grid.addWidget(QLabel("状态:"), 0, 0)
            state_lbl = QLabel("—")
            state_lbl.setFont(QFont("", 11, QFont.Bold))
            grid.addWidget(state_lbl, 0, 1)

            # Row 1: Position
            grid.addWidget(QLabel("位置:"), 1, 0)
            pos_lbl = QLabel("—")
            pos_lbl.setStyleSheet("font-family: Consolas; color: #89b4fa;")
            grid.addWidget(pos_lbl, 1, 1)

            # Row 2: Velocity
            grid.addWidget(QLabel("速度:"), 2, 0)
            vel_lbl = QLabel("—")
            vel_lbl.setStyleSheet("font-family: Consolas;")
            grid.addWidget(vel_lbl, 2, 1)

            # Row 3: Iq current
            grid.addWidget(QLabel("Iq 电流:"), 3, 0)
            iq_lbl = QLabel("—")
            iq_lbl.setStyleSheet("font-family: Consolas; color: #f9e2af;")
            grid.addWidget(iq_lbl, 3, 1)

            # Row 4: FET temp
            grid.addWidget(QLabel("FET 温度:"), 4, 0)
            fet_temp_lbl = QLabel("—")
            grid.addWidget(fet_temp_lbl, 4, 1)

            # Row 5: Motor temp
            grid.addWidget(QLabel("电机温度:"), 5, 0)
            mot_temp_lbl = QLabel("—")
            grid.addWidget(mot_temp_lbl, 5, 1)

            # Row 6: Calibrated + Ready
            grid.addWidget(QLabel("状态:"), 6, 0)
            cal_lbl = QLabel("—")
            cal_lbl.setStyleSheet("font-family: Consolas;")
            grid.addWidget(cal_lbl, 6, 1)

            axes_row.addWidget(group)

            self._axis_groups.append({
                'state': state_lbl,
                'pos': pos_lbl,
                'vel': vel_lbl,
                'iq': iq_lbl,
                'fet_temp': fet_temp_lbl,
                'mot_temp': mot_temp_lbl,
                'cal': cal_lbl,
            })

        layout.addLayout(axes_row)

    def clear(self):
        """Reset all labels to default."""
        self._vbus_label.setText("— V")
        self._ibus_label.setText("— A")
        self._vbus_bar.setValue(0)
        self._brake_armed.setText("—")
        self._brake_sat.setText("—")
        for g in self._axis_groups:
            g['state'].setText("—")
            g['pos'].setText("—")
            g['vel'].setText("—")
            g['iq'].setText("—")
            g['fet_temp'].setText("—")
            g['mot_temp'].setText("—")
            g['cal'].setText("—")

    def update_telemetry(self, data):
        """Update all metrics from telemetry data dict."""
        # Board-level
        vbus = data.get('vbus_voltage', 0)
        ibus = data.get('ibus', 0)
        self._vbus_label.setText(f"{vbus:.1f} V")
        self._ibus_label.setText(f"{ibus:.2f} A")
        self._vbus_bar.setValue(int(min(vbus, 60)))

        brake_armed = data.get('brake_resistor_armed', False)
        brake_sat = data.get('brake_resistor_saturated', False)
        self._brake_armed.setText("已启用" if brake_armed else "未启用")
        self._brake_armed.setStyleSheet(
            f"color: {'#a6e3a1' if brake_armed else '#f38ba8'};")
        self._brake_sat.setText("饱和!" if brake_sat else "")
        self._brake_sat.setStyleSheet("color: #f38ba8;")

        # Per-axis
        for i in [0, 1]:
            prefix = f'm{i}'
            g = self._axis_groups[i]

            state_val = data.get(f'{prefix}_current_state', 0)
            from odrive_gui.odrive_worker import ODriveWorker
            state_name = ODriveWorker.get_state_name(state_val)
            g['state'].setText(state_name)
            color = self.STATE_COLORS.get(state_name, '#cdd6f4')
            g['state'].setStyleSheet(f"color: {color}; font-weight: bold;")

            g['pos'].setText(f"{data.get(f'{prefix}_pos', 0):.3f} turn")
            g['vel'].setText(f"{data.get(f'{prefix}_vel', 0):.2f} turn/s")
            g['iq'].setText(f"{data.get(f'{prefix}_iq', 0):.2f} A")

            fet_temp = data.get(f'{prefix}_fet_temp', 0)
            fet_temp_color = '#a6e3a1' if fet_temp < 80 else '#f9e2af' if fet_temp < 100 else '#f38ba8'
            g['fet_temp'].setText(f"{fet_temp:.1f} °C")
            g['fet_temp'].setStyleSheet(f"color: {fet_temp_color};")

            mot_temp = data.get(f'{prefix}_motor_temp', 0)
            g['mot_temp'].setText(f"{mot_temp:.1f} °C")

            is_cal = data.get(f'{prefix}_is_calibrated', False)
            is_ready = data.get(f'{prefix}_is_ready', False)
            cal_parts = []
            cal_parts.append("已校准" if is_cal else "未校准")
            cal_parts.append("就绪" if is_ready else "未就绪")
            g['cal'].setText(" | ".join(cal_parts))
            g['cal'].setStyleSheet(
                f"color: {'#a6e3a1' if is_cal and is_ready else '#f38ba8'};")
