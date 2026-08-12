"""
Error panel widget for displaying error states of one motor axis.

Displays error bitfields as categorized groups with color-coded
indicators (green = OK, red = ERROR). Hardcoded error bit definitions
match ODrive firmware v0.5.1.
"""

from PySide6.QtWidgets import (
    QGroupBox, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QGridLayout, QScrollArea, QWidget, QFrame
)
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont


# ---------------------------------------------------------------------------
# Hardcoded ODrive v0.5.1 error bit definitions
# ---------------------------------------------------------------------------

AXIS_ERRORS = {
    0x00000001: 'INVALID_STATE',
    0x00000002: 'DC_BUS_UNDER_VOLTAGE',
    0x00000004: 'DC_BUS_OVER_VOLTAGE',
    0x00000008: 'CURRENT_MEASUREMENT_TIMEOUT',
    0x00000010: 'BRAKE_RESISTOR_DISARMED',
    0x00000020: 'MOTOR_DISARMED',
    0x00000040: 'MOTOR_FAILED',
    0x00000080: 'SENSORLESS_ESTIMATOR_FAILED',
    0x00000100: 'ENCODER_FAILED',
    0x00000200: 'CONTROLLER_FAILED',
    0x00000400: 'POS_CTRL_DURING_SENSORLESS',
    0x00000800: 'WATCHDOG_TIMER_EXPIRED',
    0x00001000: 'MIN_ENDSTOP_PRESSED',
    0x00002000: 'MAX_ENDSTOP_PRESSED',
    0x00004000: 'ESTOP_REQUESTED',
    0x00020000: 'HOMING_WITHOUT_ENDSTOP',
    0x00040000: 'OVER_TEMP',
}

MOTOR_ERRORS = {
    0x00000001: 'PHASE_RESISTANCE_OUT_OF_RANGE',
    0x00000002: 'PHASE_INDUCTANCE_OUT_OF_RANGE',
    0x00000004: 'ADC_FAILED',
    0x00000008: 'DRV_FAULT',
    0x00000010: 'CONTROL_DEADLINE_MISSED',
    0x00000020: 'NOT_IMPLEMENTED_MOTOR_TYPE',
    0x00000040: 'BRAKE_CURRENT_OUT_OF_RANGE',
    0x00000080: 'MODULATION_MAGNITUDE',
    0x00000100: 'BRAKE_DEADTIME_VIOLATION',
    0x00000200: 'UNEXPECTED_TIMER_CALLBACK',
    0x00000400: 'CURRENT_SENSE_SATURATION',
    0x00001000: 'CURRENT_LIMIT_VIOLATION',
    0x00002000: 'BRAKE_DUTY_CYCLE_NAN',
    0x00004000: 'DC_BUS_OVER_REGEN_CURRENT',
    0x00008000: 'DC_BUS_OVER_CURRENT',
}

ENCODER_ERRORS = {
    0x00000001: 'UNSTABLE_GAIN',
    0x00000002: 'CPR_POLEPAIRS_MISMATCH',
    0x00000004: 'NO_RESPONSE',
    0x00000008: 'UNSUPPORTED_ENCODER_MODE',
    0x00000010: 'ILLEGAL_HALL_STATE',
    0x00000020: 'INDEX_NOT_FOUND_YET',
    0x00000040: 'ABS_SPI_TIMEOUT',
    0x00000080: 'ABS_SPI_COM_FAIL',
    0x00000100: 'ABS_SPI_NOT_READY',
}

CONTROLLER_ERRORS = {
    0x00000001: 'OVERSPEED',
    0x00000002: 'INVALID_INPUT_MODE',
    0x00000004: 'UNSTABLE_GAIN',
    0x00000008: 'INVALID_MIRROR_AXIS',
    0x00000010: 'INVALID_LOAD_ENCODER',
    0x00000020: 'INVALID_ESTIMATE',
}

THERMISTOR_ERRORS = {
    0x00000001: 'OVER_TEMP',
    0x00000002: 'UNDER_TEMP',
}

# Mapping from data key suffix to error definitions and display name
ERROR_CATEGORIES = [
    ('axis_error',    AXIS_ERRORS,       '轴错误 (Axis)'),
    ('motor_error',   MOTOR_ERRORS,      '电机错误 (Motor)'),
    ('encoder_error', ENCODER_ERRORS,    '编码器错误 (Encoder)'),
    ('controller_error', CONTROLLER_ERRORS, '控制器错误 (Controller)'),
    ('thermistor_error', THERMISTOR_ERRORS, '热敏电阻错误 (Thermistor)'),
]

# Colors
COLOR_OK = '#a6e3a1'          # green
COLOR_ERROR = '#f38ba8'        # red
COLOR_OK_BG = '#1a3a1a'       # dark green background
COLOR_ERROR_BG = '#3a1a1a'    # dark red background
COLOR_NO_ERROR = '#a6e3a1'    # green text for "no errors"
COLOR_HAS_ERROR = '#f38ba8'   # red text for error summary
COLOR_GROUP_TITLE = '#89b4fa' # blue for group titles


class ErrorPanel(QGroupBox):
    """
    Displays error states for a single motor axis.

    Errors are shown in categorized groups (Axis, Motor, Encoder,
    Controller, Thermistor) with each error bit rendered as a colored
    indicator label.
    """

    def __init__(self, axis_num, worker, parent=None):
        """
        Parameters
        ----------
        axis_num : int
            Axis number (0 or 1).
        worker : ODriveWorker
            The background worker thread for ODrive communication.
        parent : QWidget, optional
        """
        super().__init__(parent)
        self._axis_num = axis_num
        self._worker = worker
        self._prefix = f'm{axis_num}'
        self._error_labels = {}   # category_name -> {bit_mask: (indicator_label, name_label)}
        self._summary_labels = {} # category_name -> QLabel for the summary row

        self.setTitle(f"⚙ M{axis_num} 错误状态")
        self._setup_ui()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _setup_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setSpacing(6)

        # Overall status line at the top
        self._all_clear_label = QLabel("✓ 无错误")
        self._all_clear_label.setAlignment(Qt.AlignCenter)
        self._all_clear_label.setFont(QFont("", 13, QFont.Bold))
        self._all_clear_label.setStyleSheet(
            f"color: {COLOR_OK}; padding: 4px; background-color: #252536; "
            f"border-radius: 4px;"
        )
        main_layout.addWidget(self._all_clear_label)

        # Scrollable area for all error groups
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setStyleSheet("QScrollArea { border: none; background: transparent; }")

        groups_widget = QWidget()
        groups_layout = QVBoxLayout(groups_widget)
        groups_layout.setSpacing(6)
        groups_layout.setContentsMargins(0, 0, 0, 0)

        for key, error_dict, title in ERROR_CATEGORIES:
            group = self._create_error_group(key, error_dict, title)
            groups_layout.addWidget(group)

        groups_layout.addStretch()
        scroll.setWidget(groups_widget)
        main_layout.addWidget(scroll, 1)

        # Clear errors button
        btn_row = QHBoxLayout()
        self._clear_btn = QPushButton("🔄 清除该轴所有错误")
        self._clear_btn.setToolTip(f"调用 axis{self._axis_num}.clear_errors()")
        self._clear_btn.clicked.connect(self._clear_errors)
        self._clear_btn.setStyleSheet(
            "QPushButton {"
            "  background-color: #f9e2af;"
            "  color: #1e1e2e;"
            "  border: none;"
            "  border-radius: 4px;"
            "  padding: 8px 16px;"
            "  font-weight: bold;"
            "}"
            "QPushButton:hover { background-color: #f5c890; }"
        )
        btn_row.addStretch()
        btn_row.addWidget(self._clear_btn)
        btn_row.addStretch()
        main_layout.addLayout(btn_row)

    def _create_error_group(self, key, error_dict, title):
        """Create a collapsible-looking group for one error category."""
        group = QGroupBox(title)
        group.setStyleSheet(
            f"QGroupBox {{ font-weight: bold; color: {COLOR_GROUP_TITLE}; "
            f"border: 1px solid #45475a; border-radius: 4px; margin-top: 8px; "
            f"padding-top: 14px; }}"
            f"QGroupBox::title {{ subcontrol-origin: margin; left: 8px; padding: 0 4px; }}"
        )

        layout = QGridLayout(group)
        layout.setSpacing(2)
        layout.setContentsMargins(6, 6, 6, 6)

        labels = {}
        # Sort bits so higher-value (more significant) bits appear first
        sorted_bits = sorted(error_dict.keys(), reverse=True)

        for row, bit in enumerate(sorted_bits):
            name = error_dict[bit]

            # Indicator dot
            indicator = QLabel("●")
            indicator.setFixedWidth(20)
            indicator.setAlignment(Qt.AlignCenter)
            indicator.setFont(QFont("", 10))
            indicator.setStyleSheet(
                f"color: {COLOR_OK}; font-weight: bold;"
            )

            # Error name
            name_label = QLabel(name)
            name_label.setStyleSheet("font-family: Consolas; font-size: 11px; color: #bac2de;")

            layout.addWidget(indicator, row, 0)
            layout.addWidget(name_label, row, 1)

            labels[bit] = (indicator, name_label)

        # Summary line at bottom of group
        summary_label = QLabel("OK")
        summary_label.setFont(QFont("", 10, QFont.Bold))
        summary_label.setStyleSheet(
            f"color: {COLOR_OK}; padding-top: 2px;"
        )
        layout.addWidget(summary_label, len(sorted_bits), 0, 1, 2)

        self._error_labels[key] = labels
        self._summary_labels[key] = summary_label

        return group

    # ------------------------------------------------------------------
    # Telemetry update
    # ------------------------------------------------------------------

    def update_telemetry(self, data):
        """
        Update all error indicators from a telemetry data dictionary.

        Expected keys in *data* (matching the worker poll keys):

        - ``m{axis_num}_axis_error``
        - ``m{axis_num}_motor_error``
        - ``m{axis_num}_encoder_error``
        - ``m{axis_num}_controller_error``
        - ``m{axis_num}_thermistor_error``  (optional; defaults to 0)
        """
        total_active = 0

        for key, error_dict, _title in ERROR_CATEGORIES:
            data_key = f'{self._prefix}_{key}'
            value = int(data.get(data_key, 0) or 0)

            labels = self._error_labels.get(key, {})
            category_errors = 0

            for bit, (indicator, name_label) in labels.items():
                if value & bit:
                    # Error is active
                    indicator.setText("●")
                    indicator.setStyleSheet(
                        f"color: {COLOR_ERROR}; font-weight: bold;"
                    )
                    name_label.setStyleSheet(
                        f"font-family: Consolas; font-size: 11px; color: {COLOR_ERROR}; "
                        f"font-weight: bold;"
                    )
                    category_errors += 1
                else:
                    # No error
                    indicator.setText("○")
                    indicator.setStyleSheet(
                        f"color: {COLOR_OK}; font-weight: normal;"
                    )
                    name_label.setStyleSheet(
                        "font-family: Consolas; font-size: 11px; color: #6c7086;"
                    )

            # Update category summary
            summary = self._summary_labels.get(key)
            if summary is not None:
                if category_errors == 0:
                    summary.setText("✓ OK")
                    summary.setStyleSheet(f"color: {COLOR_OK}; font-weight: bold;")
                else:
                    summary.setText(f"✗ {category_errors} 个错误 (0x{value:08X})")
                    summary.setStyleSheet(f"color: {COLOR_ERROR}; font-weight: bold;")

            total_active += category_errors

        # Update overall status
        if total_active == 0:
            self._all_clear_label.setText("✓ 无错误 — 轴状态正常")
            self._all_clear_label.setStyleSheet(
                f"color: {COLOR_OK}; padding: 4px; background-color: #1a3a1a; "
                f"border-radius: 4px; font-weight: bold;"
            )
        else:
            self._all_clear_label.setText(f"✗ 检测到 {total_active} 个活跃错误")
            self._all_clear_label.setStyleSheet(
                f"color: {COLOR_ERROR}; padding: 4px; background-color: #3a1a1a; "
                f"border-radius: 4px; font-weight: bold;"
            )

    # ------------------------------------------------------------------
    # Actions
    # ------------------------------------------------------------------

    def _clear_errors(self, *_):
        """Send a clear-errors command for this axis."""
        self._worker.call_function(f'axis{self._axis_num}.clear_errors')

    def clear(self):
        """Reset all indicators to their default (OK) state."""
        for key, labels in self._error_labels.items():
            for bit, (indicator, name_label) in labels.items():
                indicator.setText("○")
                indicator.setStyleSheet(f"color: {COLOR_OK}; font-weight: normal;")
                name_label.setStyleSheet(
                    "font-family: Consolas; font-size: 11px; color: #6c7086;"
                )

            summary = self._summary_labels.get(key)
            if summary is not None:
                summary.setText("✓ OK")
                summary.setStyleSheet(f"color: {COLOR_OK}; font-weight: bold;")

        self._all_clear_label.setText("✓ 无错误 — 轴状态正常")
        self._all_clear_label.setStyleSheet(
            f"color: {COLOR_OK}; padding: 4px; background-color: #1a3a1a; "
            f"border-radius: 4px; font-weight: bold;"
        )
