"""
Parameter editor widget for browsing and editing all ODrive configuration.
Provides a tree view and a form-based interface.
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QGroupBox, QTreeWidget, QTreeWidgetItem, QScrollArea,
    QDoubleSpinBox, QSpinBox, QComboBox, QCheckBox, QMessageBox,
    QSplitter, QFormLayout, QLineEdit
)
from PySide6.QtCore import Qt


class ParameterEditor(QWidget):
    """Tree-based parameter browser and editor for all ODrive config."""

    def __init__(self, worker, parent=None):
        super().__init__(parent)
        self._worker = worker
        self._current_obj_path = None  # e.g. "axis0.motor.config"
        self._pending_changes = {}     # path -> value
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(6)

        # Toolbar
        toolbar = QHBoxLayout()

        self._refresh_btn = QPushButton("🔄 刷新参数树")
        self._refresh_btn.clicked.connect(self.refresh_tree)
        toolbar.addWidget(self._refresh_btn)

        self._apply_btn = QPushButton("✅ 应用更改")
        self._apply_btn.clicked.connect(self._apply_changes)
        self._apply_btn.setStyleSheet("QPushButton { background-color: #a6e3a1; color: #1e1e2e; font-weight: bold; border: none; border-radius: 4px; padding: 6px 16px; } QPushButton:hover { background-color: #94e2a0; }")
        toolbar.addWidget(self._apply_btn)

        self._save_flash_btn = QPushButton("💾 保存到 Flash")
        self._save_flash_btn.clicked.connect(self._save_to_flash)
        self._save_flash_btn.setStyleSheet("QPushButton { background-color: #f9e2af; color: #1e1e2e; font-weight: bold; border: none; border-radius: 4px; padding: 6px 16px; } QPushButton:hover { background-color: #f5c890; }")
        toolbar.addWidget(self._save_flash_btn)

        toolbar.addStretch()
        layout.addLayout(toolbar)

        # Splitter: tree on left, form editor on right
        splitter = QSplitter(Qt.Horizontal)

        # Tree widget
        tree_group = QGroupBox("参数树")
        tree_layout = QVBoxLayout(tree_group)
        self._tree = QTreeWidget()
        self._tree.setHeaderLabels(["参数", "值", "单位"])
        self._tree.setColumnWidth(0, 220)
        self._tree.setColumnWidth(1, 120)
        self._tree.setColumnWidth(2, 80)
        self._tree.setAlternatingRowColors(True)
        self._tree.itemDoubleClicked.connect(self._on_item_double_clicked)
        tree_layout.addWidget(self._tree)
        splitter.addWidget(tree_group)

        # Form editor (shown when a config group is selected)
        form_group = QGroupBox("参数编辑")
        form_layout = QVBoxLayout(form_group)
        self._form_scroll = QScrollArea()
        self._form_scroll.setWidgetResizable(True)
        self._form_widget = QWidget()
        self._form_layout = QFormLayout(self._form_widget)
        self._form_layout.setSpacing(4)
        self._form_scroll.setWidget(self._form_widget)
        form_layout.addWidget(self._form_scroll)
        splitter.addWidget(form_group)

        splitter.setSizes([400, 500])
        layout.addWidget(splitter)

    def refresh_tree(self, *_):
        """Rebuild the parameter tree from the connected ODrive."""
        self._tree.clear()
        if not self._worker.is_connected or self._worker.odrv is None:
            return

        odrv = self._worker.odrv

        # Board config
        board_item = QTreeWidgetItem(["Board Config", "", ""])
        self._tree.addTopLevelItem(board_item)
        self._populate_config_group(board_item, odrv, "config")

        # CAN config
        can_item = QTreeWidgetItem(["CAN Config", "", ""])
        self._tree.addTopLevelItem(can_item)
        self._populate_config_group(can_item, odrv.can, "config")

        # Per-axis
        for axis_num in [0, 1]:
            axis_name = f"axis{axis_num}"
            axis = getattr(odrv, axis_name)
            axis_item = QTreeWidgetItem([f"M{axis_num} Axis Config", "", ""])
            self._tree.addTopLevelItem(axis_item)

            sub_groups = [
                (f"M{axis_num} Axis", axis, "config"),
                (f"M{axis_num} Motor", axis.motor, "config"),
                (f"M{axis_num} Controller", axis.controller, "config"),
                (f"M{axis_num} Encoder", axis.encoder, "config"),
                (f"M{axis_num} Trap Traj", axis.trap_traj, "config"),
                (f"M{axis_num} FET Thermistor", axis.fet_thermistor, "config"),
                (f"M{axis_num} Motor Thermistor", axis.motor_thermistor, "config"),
                (f"M{axis_num} Min Endstop", axis.min_endstop, "config"),
                (f"M{axis_num} Max Endstop", axis.max_endstop, "config"),
            ]
            for name, obj, cfg_name in sub_groups:
                sub_item = QTreeWidgetItem([name, "", ""])
                axis_item.addChild(sub_item)
                try:
                    cfg_obj = getattr(obj, cfg_name)
                    self._populate_config_group(sub_item, obj, cfg_name)
                except Exception:
                    pass

        self._tree.expandAll()

    def _populate_config_group(self, parent_item, obj, config_attr_name):
        """Read all config properties and add them as tree items."""
        try:
            cfg = getattr(obj, config_attr_name)
        except Exception:
            return

        if not hasattr(cfg, '_remote_attributes'):
            return

        for key, prop in sorted(cfg._remote_attributes.items()):
            try:
                from fibre.remote_object import RemoteProperty
                if isinstance(prop, RemoteProperty):
                    try:
                        value = prop.get_value()
                    except Exception:
                        value = "?"
                    type_name = prop._codec.__class__.__name__ if hasattr(prop, '_codec') else ""
                    path = f"{config_attr_name}.{key}"
                    item = QTreeWidgetItem([key, str(value), ""])
                    item.setData(0, Qt.UserRole, (obj, config_attr_name, key, prop))
                    parent_item.addChild(item)
            except Exception:
                pass

    def _on_item_double_clicked(self, item, column):
        """When a leaf parameter is double-clicked, show editor in form."""
        data = item.data(0, Qt.UserRole)
        if not data:
            return
        obj, config_attr_name, key, prop = data

        # Clear previous form
        while self._form_layout.rowCount() > 0:
            self._form_layout.removeRow(0)

        try:
            current_value = prop.get_value()
        except Exception:
            current_value = 0

        self._form_layout.addRow(QLabel(f"<b>{key}</b>"), QLabel(""))

        # Find the full property path for writing
        path = self._find_property_path(obj, config_attr_name, key)

        if isinstance(current_value, bool):
            cb = QCheckBox()
            cb.setChecked(current_value)
            cb.toggled.connect(lambda checked, p=path, v=cb: self._queue_change(p, v.isChecked()))
            self._form_layout.addRow("值:", cb)
        elif isinstance(current_value, int):
            # Check if it's an enum
            spin = QSpinBox()
            spin.setRange(-2147483648, 2147483647)
            spin.setValue(current_value)
            spin.valueChanged.connect(lambda val, p=path: self._queue_change(p, val))
            self._form_layout.addRow("值:", spin)
        elif isinstance(current_value, float):
            spin = QDoubleSpinBox()
            spin.setRange(-1e9, 1e9)
            spin.setDecimals(6)
            spin.setValue(current_value)
            spin.valueChanged.connect(lambda val, p=path: self._queue_change(p, val))
            self._form_layout.addRow("值:", spin)
        else:
            label = QLabel(str(current_value))
            self._form_layout.addRow("值:", label)

        self._form_layout.addRow(QLabel(f"<i>路径: {path}</i>"), QLabel(""))

    def _find_property_path(self, obj, config_attr_name, key):
        """Heuristic to find the full property path."""
        return f"{config_attr_name}.{key}"

    def _queue_change(self, path, value):
        """Queue a property change."""
        self._pending_changes[path] = value

    def _apply_changes(self, *_):
        """Write all pending changes to the device."""
        if not self._pending_changes:
            return

        for path, value in self._pending_changes.items():
            self._worker.write_property(path, value)

        count = len(self._pending_changes)
        self._pending_changes.clear()
        # Refresh tree after a short delay
        from PySide6.QtCore import QTimer
        QTimer.singleShot(500, self.refresh_tree)

    def _save_to_flash(self, *_):
        """Save configuration to non-volatile memory."""
        if not self._worker.is_connected:
            QMessageBox.warning(self, "未连接", "请先连接 ODrive 设备。")
            return
        try:
            self._worker.odrv.save_configuration()
            QMessageBox.information(self, "成功", "配置已保存到 Flash。")
        except Exception as e:
            QMessageBox.critical(self, "保存失败", str(e))

    def get_tree(self):
        """Get the tree widget for external access."""
        return self._tree
