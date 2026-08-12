"""
Main window for ODrive GUI.
Orchestrates all panels and manages the overall application layout.
"""

import sys
import os
from PySide6.QtWidgets import (
    QMainWindow, QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QSplitter, QTabWidget, QStatusBar, QLabel, QMenuBar, QMenu,
    QMessageBox, QFileDialog, QGroupBox
)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QAction, QFont

from odrive_gui.odrive_worker import ODriveWorker
from odrive_gui.connection_panel import ConnectionPanel
from odrive_gui.dashboard_panel import DashboardPanel
from odrive_gui.axis_control_panel import AxisControlPanel
from odrive_gui.live_plotter import LivePlotterWidget
from odrive_gui.terminal_widget import TerminalWidget


class ODriveMainWindow(QMainWindow):
    """Main application window for ODrive GUI."""

    def __init__(self):
        super().__init__()
        self._worker = ODriveWorker()
        self._setup_worker_signals()
        self._setup_ui()
        self._apply_style()

    def _setup_worker_signals(self):
        """Connect worker signals to UI slots."""
        self._worker.device_connected.connect(self._on_device_connected)
        self._worker.device_disconnected.connect(self._on_device_disconnected)
        self._worker.connection_error.connect(self._on_connection_error)
        self._worker.connection_status.connect(self._on_connection_status)
        self._worker.telemetry_updated.connect(self._on_telemetry_updated)

    def _setup_ui(self):
        """Build the main window UI."""
        self.setWindowTitle("ODrive Control GUI v1.0")
        self.setMinimumSize(1200, 800)
        self.resize(1400, 900)

        # --- Menu Bar ---
        menubar = self.menuBar()

        file_menu = menubar.addMenu("文件(&F)")
        save_cfg_action = QAction("保存配置到 Flash", self)
        save_cfg_action.triggered.connect(self._save_config)
        file_menu.addAction(save_cfg_action)

        export_action = QAction("导出配置到 JSON...", self)
        export_action.triggered.connect(self._export_config)
        file_menu.addAction(export_action)

        import_action = QAction("从 JSON 导入配置...", self)
        import_action.triggered.connect(self._import_config)
        file_menu.addAction(import_action)

        file_menu.addSeparator()
        quit_action = QAction("退出(&Q)", self)
        quit_action.setShortcut("Ctrl+Q")
        quit_action.triggered.connect(self.close)
        file_menu.addAction(quit_action)

        device_menu = menubar.addMenu("设备(&D)")
        scan_action = QAction("扫描设备", self)
        scan_action.triggered.connect(self._scan_devices)
        device_menu.addAction(scan_action)

        reboot_action = QAction("重启 ODrive", self)
        reboot_action.triggered.connect(self._reboot_device)
        device_menu.addAction(reboot_action)

        dfu_action = QAction("进入 DFU 模式", self)
        dfu_action.triggered.connect(self._enter_dfu)
        device_menu.addAction(dfu_action)

        view_menu = menubar.addMenu("视图(&V)")
        self._dark_action = QAction("深色主题", self, checkable=True, checked=True)
        view_menu.addAction(self._dark_action)

        help_menu = menubar.addMenu("帮助(&H)")
        about_action = QAction("关于", self)
        about_action.triggered.connect(self._show_about)
        help_menu.addAction(about_action)

        # --- Central Widget ---
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(6, 6, 6, 6)
        main_layout.setSpacing(6)

        # Top row: Connection + Dashboard
        top_row = QHBoxLayout()

        # Connection panel (left side)
        self.connection_panel = ConnectionPanel(self._worker)
        top_row.addWidget(self.connection_panel, 1)

        # Dashboard panel
        self.dashboard_panel = DashboardPanel()
        top_row.addWidget(self.dashboard_panel, 2)

        main_layout.addLayout(top_row, 1)

        # Middle: Axis control + Live plotter
        middle_splitter = QSplitter(Qt.Horizontal)

        # Axis control tabs (M0 and M1)
        self.axis_tabs = QTabWidget()
        self.axis0_panel = AxisControlPanel(0, self._worker)
        self.axis1_panel = AxisControlPanel(1, self._worker)
        self.axis_tabs.addTab(self.axis0_panel, "⚙ M0 轴控制")
        self.axis_tabs.addTab(self.axis1_panel, "⚙ M1 轴控制")
        middle_splitter.addWidget(self.axis_tabs)

        # Live plotter
        self.live_plotter = LivePlotterWidget()
        middle_splitter.addWidget(self.live_plotter)

        middle_splitter.setSizes([600, 600])
        main_layout.addWidget(middle_splitter, 3)

        # Bottom: Terminal
        self.terminal = TerminalWidget(self._worker)
        self.terminal.setMaximumHeight(180)
        main_layout.addWidget(self.terminal, 1)

        # --- Status Bar ---
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self._status_connection = QLabel("🔴 未连接")
        self._status_poll_rate = QLabel("轮询: 20Hz")
        self._status_errors = QLabel("错误: 0")
        self.status_bar.addWidget(self._status_connection)
        self.status_bar.addPermanentWidget(self._status_poll_rate)
        self.status_bar.addPermanentWidget(self._status_errors)

        # HACK: Pre-populate telemetry labels so they don't show 0 before connection
        # This is done inside each panel's clear/reset method

    def _apply_style(self):
        """Load and apply the QSS stylesheet."""
        style_path = os.path.join(os.path.dirname(__file__), "resources", "style.qss")
        if os.path.exists(style_path):
            with open(style_path, 'r', encoding='utf-8') as f:
                self.setStyleSheet(f.read())
        else:
            # Apply minimal dark style inline
            self.setStyleSheet("""
                QMainWindow { background-color: #1e1e2e; }
                QWidget { color: #cdd6f4; font-family: "Segoe UI", "Microsoft YaHei", sans-serif; font-size: 13px; }
                QGroupBox { border: 1px solid #45475a; border-radius: 6px; margin-top: 8px; padding-top: 14px; font-weight: bold; }
                QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
            """)

    # --- Signal Handlers ---

    def _on_device_connected(self, odrv):
        self._status_connection.setText("🟢 已连接")
        self._status_connection.setStyleSheet("color: #a6e3a1;")
        self.connection_panel.on_connected(odrv)
        self.dashboard_panel.clear()

    def _on_device_disconnected(self):
        self._status_connection.setText("🔴 未连接")
        self._status_connection.setStyleSheet("color: #f38ba8;")
        self.connection_panel.on_disconnected()
        self.live_plotter.stop()

    def _on_connection_error(self, msg):
        self._status_connection.setText("⚠ 连接错误")
        self._status_connection.setStyleSheet("color: #fab387;")
        QMessageBox.warning(self, "连接错误", msg)

    def _on_connection_status(self, msg):
        self.status_bar.showMessage(msg, 3000)

    def _on_telemetry_updated(self, data):
        self.dashboard_panel.update_telemetry(data)
        self.axis0_panel.update_telemetry(data)
        self.axis1_panel.update_telemetry(data)
        self.live_plotter.add_data(data)
        self.connection_panel.update_telemetry(data)

        # Update error count
        total_errors = 0
        for i in [0, 1]:
            if data.get(f'm{i}_axis_error', 0):
                total_errors += 1
            if data.get(f'm{i}_motor_error', 0):
                total_errors += 1
        self._status_errors.setText(f"错误: {total_errors}")
        if total_errors > 0:
            self._status_errors.setStyleSheet("color: #f38ba8;")
        else:
            self._status_errors.setStyleSheet("color: #a6e3a1;")

    # --- Menu Actions ---

    def _scan_devices(self):
        self.connection_panel.scan_devices()

    def _save_config(self):
        if self._worker.is_connected:
            try:
                self._worker.odrv.save_configuration()
                self.status_bar.showMessage("配置已保存到 Flash ✔", 3000)
            except Exception as e:
                QMessageBox.critical(self, "保存失败", str(e))
        else:
            QMessageBox.warning(self, "未连接", "请先连接 ODrive 设备。")

    def _export_config(self):
        if not self._worker.is_connected:
            QMessageBox.warning(self, "未连接", "请先连接 ODrive 设备。")
            return
        path, _ = QFileDialog.getSaveFileName(self, "导出配置", "odrive_config.json", "JSON (*.json)")
        if path:
            try:
                from odrive.configuration import save_config_to_file
                save_config_to_file(self._worker.odrv, path)
                self.status_bar.showMessage(f"配置已导出到 {path}", 3000)
            except Exception as e:
                QMessageBox.critical(self, "导出失败", str(e))

    def _import_config(self):
        if not self._worker.is_connected:
            QMessageBox.warning(self, "未连接", "请先连接 ODrive 设备。")
            return
        path, _ = QFileDialog.getOpenFileName(self, "导入配置", "", "JSON (*.json)")
        if path:
            reply = QMessageBox.question(self, "确认导入",
                "导入配置将覆盖当前所有参数。\n导入后需要保存到 Flash 并可能重新校准。\n确定继续？",
                QMessageBox.Yes | QMessageBox.No)
            if reply == QMessageBox.Yes:
                try:
                    from odrive.configuration import restore_config_from_file
                    restore_config_from_file(self._worker.odrv, path)
                    self.status_bar.showMessage(f"配置已从 {path} 导入", 3000)
                except Exception as e:
                    QMessageBox.critical(self, "导入失败", str(e))

    def _reboot_device(self):
        if self._worker.is_connected:
            reply = QMessageBox.question(self, "确认重启",
                "确定要重启 ODrive 吗？\n重启后需要重新连接。",
                QMessageBox.Yes | QMessageBox.No)
            if reply == QMessageBox.Yes:
                try:
                    self._worker.odrv.reboot()
                    self._worker.disconnect_device()
                except Exception:
                    pass

    def _enter_dfu(self):
        if self._worker.is_connected:
            reply = QMessageBox.warning(self, "进入 DFU 模式",
                "这会将 ODrive 置于固件更新模式。\n仅适用于 v3.5+ 板卡。\n确定继续？",
                QMessageBox.Yes | QMessageBox.No)
            if reply == QMessageBox.Yes:
                try:
                    self._worker.odrv.enter_dfu_mode()
                    self._worker.disconnect_device()
                except Exception as e:
                    QMessageBox.critical(self, "DFU 错误", str(e))

    def _show_about(self):
        QMessageBox.about(self, "关于 ODrive GUI",
            "<h3>ODrive Control GUI v1.0</h3>"
            "<p>ODrive 电机控制器图形化控制界面</p>"
            "<p>基于 PySide6 (Qt for Python)</p>"
            "<p>适用于 ODrive 固件 v0.5.1</p>"
            "<hr>"
            "<p>不修改固件代码，仅通过 Fibre 协议通信</p>")

    def closeEvent(self, event):
        """Clean shutdown."""
        self.live_plotter.stop()
        self._worker.stop()
        event.accept()


def main():
    """Application entry point."""
    app = QApplication(sys.argv)
    app.setApplicationName("ODrive GUI")
    app.setOrganizationName("ODriveCommunity")

    # Parse command line args
    path = "usb"
    serial_number = None
    for i, arg in enumerate(sys.argv):
        if arg == '--path' and i + 1 < len(sys.argv):
            path = sys.argv[i + 1]
        if arg == '--serial-number' and i + 1 < len(sys.argv):
            serial_number = sys.argv[i + 1]

    window = ODriveMainWindow()
    window.show()

    # Auto-connect on startup
    window.connection_panel.auto_connect(path, serial_number)

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
