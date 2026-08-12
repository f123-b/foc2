"""
Background worker thread for ODrive communication.
Handles device discovery, telemetry polling, and property read/write
in a separate QThread to keep the UI responsive.
"""

import time
import struct
import traceback
from PySide6.QtCore import QThread, Signal, QMutex, QMutexLocker, QTimer

import odrive
import odrive.enums
from odrive.enums import *


class ODriveWorker(QThread):
    """
    Background thread for all ODrive communication.
    Uses Qt signals to update the UI thread-safely.
    """

    # Connection signals
    device_connected = Signal(object)        # emits the odrive object
    device_disconnected = Signal()
    connection_error = Signal(str)
    connection_status = Signal(str)          # status messages

    # Telemetry update signal - emits a dict of all live data
    telemetry_updated = Signal(dict)

    # Error update
    error_updated = Signal(int, str, dict)   # axis_num, axis_name, error_info dict

    def __init__(self, parent=None):
        super().__init__(parent)
        self._odrv = None
        self._running = False
        self._connected = False
        self._mutex = QMutex()
        self._poll_interval = 0.05  # 50ms = 20Hz polling
        self._pending_writes = []
        self._telemetry_keys = [
            'vbus_voltage', 'ibus',
            'm0_pos', 'm0_vel', 'm0_iq', 'm0_id',
            'm0_fet_temp', 'm0_motor_temp',
            'm0_current_state', 'm0_axis_error', 'm0_motor_error',
            'm0_encoder_error', 'm0_controller_error',
            'm0_is_calibrated', 'm0_is_ready',
            'm1_pos', 'm1_vel', 'm1_iq', 'm1_id',
            'm1_fet_temp', 'm1_motor_temp',
            'm1_current_state', 'm1_axis_error', 'm1_motor_error',
            'm1_encoder_error', 'm1_controller_error',
            'm1_is_calibrated', 'm1_is_ready',
            'brake_resistor_armed', 'brake_resistor_saturated',
        ]

    @property
    def odrv(self):
        return self._odrv

    @property
    def is_connected(self):
        return self._connected

    def connect_device(self, path="usb", serial_number=None, timeout=10):
        """Initiate device connection in background thread."""
        self._path = path
        self._serial_number = serial_number
        self._timeout = timeout
        if not self.isRunning():
            self._running = True
            self.start()
        else:
            self.connection_status.emit("Worker already running, attempting reconnect...")

    def disconnect_device(self):
        """Disconnect from device."""
        self._running = False
        self._connected = False
        self._odrv = None
        self.device_disconnected.emit()

    def stop(self):
        """Stop the worker thread."""
        self._running = False
        self._connected = False
        self.wait(2000)

    def write_property(self, property_path, value):
        """Queue a property write operation. Thread-safe."""
        with QMutexLocker(self._mutex):
            self._pending_writes.append((property_path, value))

    def call_function(self, func_name, *args):
        """Call a function on the ODrive object. Thread-safe."""
        with QMutexLocker(self._mutex):
            self._pending_writes.append(('__func__', (func_name, args)))

    def set_poll_interval(self, seconds):
        """Change the telemetry polling interval."""
        self._poll_interval = max(0.01, min(1.0, seconds))

    def run(self):
        """Main thread loop."""
        # --- Phase 1: Connect to device ---
        self.connection_status.emit("正在搜索 ODrive 设备...")
        try:
            self._odrv = odrive.find_any(
                path=self._path,
                serial_number=self._serial_number,
                timeout=self._timeout
            )
            if self._odrv is None:
                self.connection_error.emit("未找到 ODrive 设备。请检查 USB 连接和电源。")
                self._running = False
                return

            self._connected = True
            self.device_connected.emit(self._odrv)
            self.connection_status.emit("已连接到 ODrive")

        except Exception as e:
            self.connection_error.emit(f"连接失败: {str(e)}")
            self._running = False
            return

        # --- Phase 2: Telemetry polling loop ---
        poll_timer = time.monotonic()
        while self._running and self._connected:
            loop_start = time.monotonic()

            try:
                # Process pending writes
                self._process_writes()

                # Poll telemetry at configured interval
                if loop_start - poll_timer >= self._poll_interval:
                    telemetry = self._poll_telemetry()
                    if telemetry:
                        self.telemetry_updated.emit(telemetry)
                    poll_timer = loop_start

            except Exception as e:
                if self._running:
                    self.connection_error.emit(f"通信错误: {str(e)}")
                    self._connected = False
                    self.device_disconnected.emit()
                break

            # Sleep to maintain poll rate
            elapsed = time.monotonic() - loop_start
            sleep_time = max(0.001, self._poll_interval - elapsed)
            if sleep_time > 0.001:
                time.sleep(sleep_time)

    def _process_writes(self):
        """Process queued write operations safely."""
        writes = []
        with QMutexLocker(self._mutex):
            writes = self._pending_writes[:]
            self._pending_writes = []

        for item in writes:
            try:
                if item[0] == '__func__':
                    func_name, args = item[1]
                    func = self._odrv
                    for part in func_name.split('.'):
                        func = getattr(func, part)
                    func(*args)
                else:
                    prop_path, value = item
                    obj = self._odrv
                    parts = prop_path.split('.')
                    for part in parts[:-1]:
                        obj = getattr(obj, part)
                    setattr(obj, parts[-1], value)
            except Exception as e:
                self.connection_status.emit(f"写入失败 {item}: {str(e)}")

    def _safe_get(self, obj, attr_path, default=0.0):
        """Safely get a nested attribute. Returns default on error."""
        try:
            result = obj
            for part in attr_path.split('.'):
                result = getattr(result, part)
            return result if result is not None else default
        except Exception:
            return default

    def _poll_telemetry(self):
        """Read all telemetry data from the ODrive."""
        if not self._odrv or not self._connected:
            return None

        odrv = self._odrv
        data = {}

        # Board-level
        data['vbus_voltage'] = self._safe_get(odrv, 'vbus_voltage')
        data['ibus'] = self._safe_get(odrv, 'ibus')
        data['brake_resistor_armed'] = self._safe_get(odrv, 'brake_resistor_armed', False)
        data['brake_resistor_saturated'] = self._safe_get(odrv, 'brake_resistor_saturated', False)

        # Per-axis telemetry
        for i in [0, 1]:
            prefix = f'm{i}'
            axis = getattr(odrv, f'axis{i}', None)
            if axis is None:
                continue

            data[f'{prefix}_pos'] = self._safe_get(axis, 'encoder.pos_estimate')
            data[f'{prefix}_vel'] = self._safe_get(axis, 'encoder.vel_estimate')
            data[f'{prefix}_iq'] = self._safe_get(axis, 'motor.current_control.Iq_measured')
            data[f'{prefix}_id'] = self._safe_get(axis, 'motor.current_control.Id_measured')
            data[f'{prefix}_fet_temp'] = self._safe_get(axis, 'fet_thermistor.temperature')
            data[f'{prefix}_motor_temp'] = self._safe_get(axis, 'motor_thermistor.temperature')
            data[f'{prefix}_current_state'] = self._safe_get(axis, 'current_state', 0)
            data[f'{prefix}_axis_error'] = self._safe_get(axis, 'error', 0)
            data[f'{prefix}_motor_error'] = self._safe_get(axis, 'motor.error', 0)
            data[f'{prefix}_encoder_error'] = self._safe_get(axis, 'encoder.error', 0)
            data[f'{prefix}_controller_error'] = self._safe_get(axis, 'controller.error', 0)
            data[f'{prefix}_is_calibrated'] = self._safe_get(axis, 'motor.is_calibrated', False)
            data[f'{prefix}_is_ready'] = self._safe_get(axis, 'encoder.is_ready', False)

        return data

    @staticmethod
    def get_state_name(state_value):
        """Convert axis state number to name."""
        state_map = {
            0: 'UNDEFINED',
            1: 'IDLE',
            2: 'STARTUP_SEQUENCE',
            3: 'FULL_CALIBRATION_SEQUENCE',
            4: 'MOTOR_CALIBRATION',
            5: 'SENSORLESS_CONTROL',
            6: 'ENCODER_INDEX_SEARCH',
            7: 'ENCODER_OFFSET_CALIBRATION',
            8: 'CLOSED_LOOP_CONTROL',
            9: 'LOCKIN_SPIN',
            10: 'ENCODER_DIR_FIND',
            11: 'HOMING',
        }
        return state_map.get(state_value, f'UNKNOWN({state_value})')
