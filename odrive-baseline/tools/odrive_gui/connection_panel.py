"""
Connection panel for discovering and connecting to ODrive devices.
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QComboBox,
    QLabel, QGroupBox, QLineEdit, QGridLayout, QProgressBar
)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont


class ConnectionPanel(QGroupBox):
    """Panel for device discovery, connection, and basic info display."""

    def __init__(self, worker, parent=None):
        super().__init__("设备连接", parent)
        self._worker = worker
        self._connected = False
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(8)

        # Path selection row
        path_layout = QHBoxLayout()
        path_layout.addWidget(QLabel("接口:"))
        self._path_combo = QComboBox()
        self._path_combo.addItems(["usb", "serial", "tcp", "udp",
                                     "usb,serial", "serial:COM3", "serial:COM4"])
        self._path_combo.setEditable(True)
        self._path_combo.setCurrentText("usb")
        path_layout.addWidget(self._path_combo, 1)
        layout.addLayout(path_layout)

        # Buttons row
        btn_layout = QHBoxLayout()
        self._scan_btn = QPushButton("🔍 扫描")
        self._scan_btn.setToolTip("搜索可用 ODrive 设备")
        self._scan_btn.clicked.connect(self.scan_devices)
        btn_layout.addWidget(self._scan_btn)

        self._connect_btn = QPushButton("🔌 连接")
        self._connect_btn.setToolTip("连接到选定的 ODrive")
        self._connect_btn.clicked.connect(self._connect)
        self._connect_btn.setStyleSheet("QPushButton { background-color: #45475a; color: #cdd6f4; border: 1px solid #585b70; border-radius: 4px; padding: 6px 14px; } QPushButton:hover { background-color: #585b70; }")
        btn_layout.addWidget(self._connect_btn)

        self._disconnect_btn = QPushButton("⏻ 断开")
        self._disconnect_btn.setEnabled(False)
        self._disconnect_btn.clicked.connect(self._disconnect)
        self._disconnect_btn.setStyleSheet("QPushButton { background-color: #e64553; color: white; border: none; border-radius: 4px; padding: 6px 14px; } QPushButton:hover { background-color: #d20f39; }")
        btn_layout.addWidget(self._disconnect_btn)
        layout.addLayout(btn_layout)

        # Status indicator
        self._status_label = QLabel("🔴 未连接")
        self._status_label.setAlignment(Qt.AlignCenter)
        self._status_label.setFont(QFont("", 14, QFont.Bold))
        layout.addWidget(self._status_label)

        # Progress
        self._progress = QProgressBar()
        self._progress.setVisible(False)
        self._progress.setRange(0, 0)  # indeterminate
        layout.addWidget(self._progress)

        # Device info grid
        info_group = QGroupBox("设备信息")
        info_grid = QGridLayout(info_group)
        info_grid.setSpacing(4)

        labels = [
            ("序列号:", "serial"),
            ("固件版本:", "fw_ver"),
            ("硬件版本:", "hw_ver"),
            ("VBUS 电压:", "vbus"),
            ("电流:", "ibus"),
        ]
        self._info_labels = {}
        for row, (label_text, key) in enumerate(labels):
            info_grid.addWidget(QLabel(label_text), row, 0)
            value_label = QLabel("—")
            value_label.setStyleSheet("font-weight: bold; color: #89b4fa;")
            info_grid.addWidget(value_label, row, 1)
            self._info_labels[key] = value_label

        layout.addWidget(info_group)
        layout.addStretch()

    def auto_connect(self, path="usb", serial_number=None):
        """Automatically connect on startup."""
        self._path_combo.setCurrentText(path)
        self._connect()

    def scan_devices(self, *_):
        """Manually trigger device scan."""
        self._connect()

    def _connect(self, *_):
        """Initiate connection."""
        path = self._path_combo.currentText()
        self._status_label.setText("🟡 搜索中...")
        self._status_label.setStyleSheet("color: #f9e2af;")
        self._scan_btn.setEnabled(False)
        self._connect_btn.setEnabled(False)
        self._progress.setVisible(True)
        self._worker.connect_device(path=path)

    def _disconnect(self, *_):
        """Manually disconnect."""
        self._worker.disconnect_device()

    def on_connected(self, odrv):
        """Called when device is connected."""
        self._connected = True
        self._status_label.setText("🟢 已连接")
        self._status_label.setStyleSheet("color: #a6e3a1;")
        self._scan_btn.setEnabled(False)
        self._connect_btn.setEnabled(False)
        self._disconnect_btn.setEnabled(True)
        self._progress.setVisible(False)

        # Populate device info
        try:
            sn = f"{odrv.serial_number:012X}" if hasattr(odrv, 'serial_number') else "N/A"
        except:
            sn = "N/A"
        self._info_labels['serial'].setText(sn)

        try:
            fw = f"v{odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}"
        except:
            fw = "N/A"
        self._info_labels['fw_ver'].setText(fw)

        try:
            hw_major = odrv.hw_version_major if hasattr(odrv, 'hw_version_major') else 0
            hw_minor = odrv.hw_version_minor if hasattr(odrv, 'hw_version_minor') else 0
            hw_variant = odrv.hw_version_variant if hasattr(odrv, 'hw_version_variant') else 0
            hw = f"v{hw_major}.{hw_minor}-{hw_variant}V"
        except:
            hw = "N/A"
        self._info_labels['hw_ver'].setText(hw)

    def on_disconnected(self):
        """Called when device is disconnected."""
        self._connected = False
        self._status_label.setText("🔴 未连接")
        self._status_label.setStyleSheet("color: #f38ba8;")
        self._scan_btn.setEnabled(True)
        self._connect_btn.setEnabled(True)
        self._disconnect_btn.setEnabled(False)
        self._progress.setVisible(False)
        for label in self._info_labels.values():
            label.setText("—")

    def update_telemetry(self, data):
        """Update live telemetry in info panel."""
        if not self._connected:
            return
        vbus = data.get('vbus_voltage', 0)
        ibus = data.get('ibus', 0)
        self._info_labels['vbus'].setText(f"{vbus:.1f} V")
        self._info_labels['ibus'].setText(f"{ibus:.2f} A")
