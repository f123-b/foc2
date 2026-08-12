"""
Real-time data plotter widget for ODrive motor controller GUI.
Provides live telemetry visualization using matplotlib embedded in Qt.
"""

import csv
import time
from collections import deque
from datetime import datetime

import numpy as np
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
    QComboBox, QCheckBox, QLabel, QFileDialog, QMessageBox,
    QFrame, QSizePolicy,
)
from PySide6.QtCore import Qt, QTimer, QMutex, QMutexLocker

import matplotlib
matplotlib.use("QtAgg")

from matplotlib.figure import Figure
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas


# ---------------------------------------------------------------------------
# Matplotlib global dark-theme settings (Catppuccin Mocha palette)
# ---------------------------------------------------------------------------
DARK_BG = "#1e1e2e"
DARK_GRID = "#313244"
DARK_TEXT = "#cdd6f4"
DARK_AXES = "#45475a"

matplotlib.rcParams.update({
    "figure.facecolor": DARK_BG,
    "axes.facecolor": DARK_BG,
    "axes.edgecolor": DARK_AXES,
    "axes.labelcolor": DARK_TEXT,
    "axes.titlecolor": DARK_TEXT,
    "text.color": DARK_TEXT,
    "xtick.color": DARK_TEXT,
    "ytick.color": DARK_TEXT,
    "grid.color": DARK_GRID,
    "grid.alpha": 0.5,
    "legend.facecolor": "#313244",
    "legend.edgecolor": DARK_AXES,
    "legend.labelcolor": DARK_TEXT,
    "legend.fontsize": 8,
})


# ---------------------------------------------------------------------------
# Channel definitions
# ---------------------------------------------------------------------------
_CHANNELS = [
    # (key,          label,              unit,      subplot_index, color)
    ("m0_pos",       "M0 Position",      "turn",    0, "#89b4fa"),
    ("m1_pos",       "M1 Position",      "turn",    0, "#f38ba8"),
    ("m0_vel",       "M0 Velocity",      "turn/s",  1, "#89b4fa"),
    ("m1_vel",       "M1 Velocity",      "turn/s",  1, "#f38ba8"),
    ("m0_iq",        "M0 Iq Current",    "A",       2, "#89b4fa"),
    ("m1_iq",        "M1 Iq Current",    "A",       2, "#f38ba8"),
    ("vbus_voltage", "VBus Voltage",     "V",       3, "#a6e3a1"),
]

# Subplot titles
_SUBPLOT_TITLES = [
    "Position",
    "Velocity",
    "Iq Current",
    "VBus Voltage",
]

# Subplot y-axis labels
_SUBPLOT_YLABELS = [
    "Position (turn)",
    "Velocity (turn/s)",
    "Iq Current (A)",
    "VBus Voltage (V)",
]


class LivePlotterWidget(QWidget):
    """
    Real-time telemetry plotter with multiple channels stacked vertically.

    Displays four subplots sharing a common time axis:
      0. Position (M0, M1)
      1. Velocity (M0, M1)
      2. Iq Current (M0, M1)
      3. VBus Voltage

    Features:
      - Rolling time window (5s / 10s / 30s / 60s)
      - Start / Stop / Pause / Clear controls
      - Per-channel visibility checkboxes
      - CSV export of buffered data
      - Thread-safe data ingestion from the ODrive worker thread
    """

    TIME_WINDOWS = {
        "5s": 5,
        "10s": 10,
        "30s": 30,
        "60s": 60,
    }
    DEFAULT_WINDOW = 10

    # Buffer sizing: support the largest window at ~30 Hz
    _MAX_SAMPLE_RATE = 30
    _MAX_WINDOW_SEC = 60
    _MAXLEN = _MAX_WINDOW_SEC * _MAX_SAMPLE_RATE  # 1800 points per channel

    # ------------------------------------------------------------------
    def __init__(self, parent=None):
        super().__init__(parent)

        # ---- state ----
        self._mutex = QMutex()
        self._running = False
        self._paused = False
        self._window_seconds = self.DEFAULT_WINDOW

        # ---- ring buffers ----
        self._timestamps = deque(maxlen=self._MAXLEN)
        self._data_buffers = {
            ch[0]: deque(maxlen=self._MAXLEN) for ch in _CHANNELS
        }

        # ---- pending data from worker thread (mutex-protected) ----
        self._pending_data = None

        # ---- per-channel visibility ----
        self._channel_visible = {ch[0]: True for ch in _CHANNELS}
        self._channel_checkboxes: dict[str, QCheckBox] = {}

        # ---- time base ----
        self._t0: float | None = None

        # ---- Plot (must come before UI because UI needs self._figure) ----
        self._setup_plot()
        # ---- UI ----
        self._setup_ui()

        # ---- update timer (~20 Hz) ----
        self._update_timer = QTimer(self)
        self._update_timer.timeout.connect(self._on_timer_tick)
        self._update_timer.setInterval(50)  # 20 Hz

        # Start running immediately
        self._running = True
        self._t0 = time.monotonic()
        self._update_timer.start()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _setup_ui(self):
        """Build the Qt widget layout: toolbar + checkboxes + canvas."""
        root = QVBoxLayout(self)
        root.setContentsMargins(4, 4, 4, 4)
        root.setSpacing(4)

        # -- Toolbar row -------------------------------------------------
        toolbar = QHBoxLayout()
        toolbar.setSpacing(6)

        # Start button
        self._btn_start = QPushButton("Start")
        self._btn_start.setFixedWidth(70)
        self._btn_start.clicked.connect(self._on_start)
        toolbar.addWidget(self._btn_start)

        # Stop button
        self._btn_stop = QPushButton("Stop")
        self._btn_stop.setFixedWidth(70)
        self._btn_stop.clicked.connect(self._on_stop)
        toolbar.addWidget(self._btn_stop)

        # Pause button (checkable)
        self._btn_pause = QPushButton("Pause")
        self._btn_pause.setFixedWidth(70)
        self._btn_pause.setCheckable(True)
        self._btn_pause.clicked.connect(self._on_pause)
        toolbar.addWidget(self._btn_pause)

        # Clear button
        self._btn_clear = QPushButton("Clear")
        self._btn_clear.setFixedWidth(70)
        self._btn_clear.clicked.connect(self._on_clear)
        toolbar.addWidget(self._btn_clear)

        # Separator
        sep = QFrame()
        sep.setFrameShape(QFrame.VLine)
        sep.setStyleSheet("color: #45475a;")
        sep.setFixedWidth(2)
        toolbar.addWidget(sep)

        # Time-window label + dropdown
        tw_label = QLabel("Window:")
        tw_label.setStyleSheet("color: #6c7086;")
        toolbar.addWidget(tw_label)

        self._combo_window = QComboBox()
        self._combo_window.addItems(list(self.TIME_WINDOWS.keys()))
        self._combo_window.setCurrentText("10s")
        self._combo_window.setFixedWidth(70)
        self._combo_window.currentTextChanged.connect(self._on_window_changed)
        toolbar.addWidget(self._combo_window)

        toolbar.addStretch()

        # Export CSV button
        self._btn_export = QPushButton("Export CSV")
        self._btn_export.setFixedWidth(100)
        self._btn_export.clicked.connect(self._on_export_csv)
        toolbar.addWidget(self._btn_export)

        root.addLayout(toolbar)

        # -- Channel visibility row --------------------------------------
        vis_layout = QHBoxLayout()
        vis_layout.setSpacing(10)

        for idx, title in enumerate(_SUBPLOT_TITLES):
            # Small group label
            grp_label = QLabel(title + ":")
            grp_label.setStyleSheet("color: #6c7086; font-size: 11px;")
            vis_layout.addWidget(grp_label)

            # Checkboxes for each channel in this subplot
            for ch_key, ch_label, _unit, sp_idx, _color in _CHANNELS:
                if sp_idx != idx:
                    continue
                cb = QCheckBox(ch_label.split(" ", 1)[0])  # "M0" / "M1" / "VBus"
                cb.setChecked(True)
                cb.setStyleSheet("font-size: 11px; spacing: 3px;")
                cb.toggled.connect(lambda checked, k=ch_key: self._on_visibility_toggled(k, checked))
                self._channel_checkboxes[ch_key] = cb
                vis_layout.addWidget(cb)

            # Small spacer between groups
            if idx < len(_SUBPLOT_TITLES) - 1:
                sep2 = QFrame()
                sep2.setFrameShape(QFrame.VLine)
                sep2.setStyleSheet("color: #45475a;")
                sep2.setFixedWidth(1)
                vis_layout.addWidget(sep2)

        vis_layout.addStretch()
        root.addLayout(vis_layout)

        # -- Matplotlib canvas -------------------------------------------
        self._canvas = FigureCanvas(self._figure)
        self._canvas.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._canvas.setMinimumHeight(300)
        root.addWidget(self._canvas, 1)

        # Initial button state
        self._update_button_states()

    def _setup_plot(self):
        """Create the matplotlib figure, subplots, and line objects."""
        self._figure = Figure(figsize=(8, 6), dpi=100)
        self._figure.subplots_adjust(
            left=0.10, right=0.97,
            bottom=0.08, top=0.96,
            hspace=0.30,
        )

        n_subplots = len(_SUBPLOT_TITLES)
        self._axes = []
        self._lines: dict[str, object] = {}  # key -> matplotlib Line2D

        for i in range(n_subplots):
            if i == 0:
                ax = self._figure.add_subplot(n_subplots, 1, i + 1)
            else:
                ax = self._figure.add_subplot(n_subplots, 1, i + 1,
                                              sharex=self._axes[0])

            ax.set_ylabel(_SUBPLOT_YLABELS[i], fontsize=8)
            ax.set_title(_SUBPLOT_TITLES[i], fontsize=9, fontweight="bold",
                         loc="left", pad=2)
            ax.grid(True, linestyle=":", linewidth=0.5)
            ax.tick_params(labelsize=7)
            ax.set_xlim(-self._window_seconds, 0)
            ax.spines["top"].set_visible(False)
            ax.spines["right"].set_visible(False)

            self._axes.append(ax)

        # Shared x-axis label on the bottom subplot only
        self._axes[-1].set_xlabel("Time (s)", fontsize=8)

        # Create a line for each channel
        for ch_key, ch_label, _unit, sp_idx, color in _CHANNELS:
            ax = self._axes[sp_idx]
            (line,) = ax.plot([], [], color=color, linewidth=1.2,
                              label=ch_label, alpha=0.9)
            self._lines[ch_key] = line

        # Add legends to subplots that have multiple channels
        for i, ax in enumerate(self._axes):
            lines_in_ax = []
            for ch_key, _label, _unit, sp_idx, _color in _CHANNELS:
                if sp_idx == i:
                    lines_in_ax.append(self._lines[ch_key])
            if lines_in_ax:
                ax.legend(loc="upper right", fontsize=7,
                          ncol=len(lines_in_ax), framealpha=0.7)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def add_data(self, data: dict):
        """
        Thread-safe: accept a telemetry dict from any thread.

        The data dict is expected to contain some or all of these keys:
          m0_pos, m1_pos, m0_vel, m1_vel, m0_iq, m1_iq, vbus_voltage
        """
        with QMutexLocker(self._mutex):
            self._pending_data = data

    def stop(self):
        """Stop the update timer and release resources. Safe to call multiple times."""
        self._running = False
        self._paused = False
        self._btn_pause.setChecked(False)
        if self._update_timer.isActive():
            self._update_timer.stop()
        self._update_button_states()

    # ------------------------------------------------------------------
    # Internal: timer tick
    # ------------------------------------------------------------------

    def _on_timer_tick(self):
        """Called at ~20 Hz. Ingests pending data and refreshes the plot."""
        if not self._running or self._paused:
            return

        # -- Ingest pending data (thread-safe swap) ----------------------
        data = None
        with QMutexLocker(self._mutex):
            if self._pending_data is not None:
                data = self._pending_data
                self._pending_data = None

        if data is None:
            # No new data; keep the plot alive but skip buffer append
            self._redraw_plot()
            return

        # -- Compute relative timestamp --------------------------------
        now = time.monotonic()
        if self._t0 is None:
            self._t0 = now
        t_rel = now - self._t0

        # -- Append to ring buffers ------------------------------------
        self._timestamps.append(t_rel)
        for ch_key in self._data_buffers:
            val = data.get(ch_key, None)
            self._data_buffers[ch_key].append(val)

        # -- Redraw ----------------------------------------------------
        self._redraw_plot()

    # ------------------------------------------------------------------
    # Plot redraw
    # ------------------------------------------------------------------

    def _redraw_plot(self):
        """Update all line artists from the ring buffers and refresh the canvas."""
        if not self._timestamps:
            return

        # Convert timestamps to a numpy array once
        t_array = np.array(self._timestamps, dtype=float)

        # Centre the x-axis on "now" (last timestamp)
        t_now = t_array[-1]
        x_min = t_now - self._window_seconds
        x_max = t_now

        for i, ax in enumerate(self._axes):
            ax.set_xlim(x_min, x_max)
            # Auto-scale y within visible data only if data exists
            y_min_global = float("inf")
            y_max_global = float("-inf")

            for ch_key, _label, _unit, sp_idx, _color in _CHANNELS:
                if sp_idx != i:
                    continue

                line = self._lines[ch_key]
                visible = self._channel_visible.get(ch_key, True)
                line.set_visible(visible)

                if not visible:
                    continue

                buf = self._data_buffers[ch_key]
                # Build filtered arrays excluding None entries
                mask = [v is not None for v in buf]
                if not any(mask):
                    line.set_data([], [])
                    continue

                t_filtered = t_array[mask]
                y_filtered = np.array([v for v, m in zip(buf, mask) if m], dtype=float)

                # Trim to visible window
                in_window = (t_filtered >= x_min) & (t_filtered <= x_max)
                if not np.any(in_window):
                    line.set_data([], [])
                    continue

                line.set_data(t_filtered[in_window], y_filtered[in_window])

                # Track Y range
                yw = y_filtered[in_window]
                y_min_global = min(y_min_global, float(np.min(yw)))
                y_max_global = max(y_max_global, float(np.max(yw)))

            # Set Y limits with 5% padding, clamping to reasonable defaults
            if y_min_global < float("inf") and y_max_global > float("-inf"):
                y_pad = max((y_max_global - y_min_global) * 0.05, 0.01)
                ax.set_ylim(y_min_global - y_pad, y_max_global + y_pad)

        self._canvas.draw_idle()

    # ------------------------------------------------------------------
    # Slot handlers
    # ------------------------------------------------------------------

    def _on_start(self, *_):
        """Resume data collection and plot updates."""
        self._running = True
        self._paused = False
        self._btn_pause.setChecked(False)
        if self._t0 is None:
            self._t0 = time.monotonic()
        if not self._update_timer.isActive():
            self._update_timer.start()
        self._update_button_states()

    def _on_stop(self, *_):
        """Stop data collection and plot updates (keeps buffered data)."""
        self._running = False
        self._paused = False
        self._btn_pause.setChecked(False)
        if self._update_timer.isActive():
            self._update_timer.stop()
        self._update_button_states()

    def _on_pause(self, checked):
        """Pause / resume the plot display."""
        self._paused = checked
        self._update_button_states()

    def _on_clear(self, *_):
        """Clear all accumulated data buffers and reset time base."""
        self._timestamps.clear()
        for buf in self._data_buffers.values():
            buf.clear()
        self._t0 = time.monotonic()
        for line in self._lines.values():
            line.set_data([], [])
        for ax in self._axes:
            ax.set_xlim(-self._window_seconds, 0)
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)
        self._canvas.draw_idle()

    def _on_window_changed(self, text):
        """Handle time-window combo-box selection."""
        self._window_seconds = self.TIME_WINDOWS.get(text, self.DEFAULT_WINDOW)

    def _on_visibility_toggled(self, ch_key, checked):
        """Handle a channel visibility checkbox toggle."""
        self._channel_visible[ch_key] = checked

    def _on_export_csv(self, *_):
        """Export all buffered data to a CSV file."""
        if not self._timestamps:
            QMessageBox.information(self, "Export CSV", "No data to export.")
            return
        default_name = f"odrive_telemetry_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        path, _filter = QFileDialog.getSaveFileName(
            self, "Export Telemetry to CSV", default_name,
            "CSV Files (*.csv);;All Files (*)",
        )
        if not path:
            return

        try:
            self._write_csv(path)
            QMessageBox.information(
                self, "Export CSV",
                f"Data exported successfully to:\n{path}",
            )
        except Exception as exc:
            QMessageBox.critical(self, "Export Failed", str(exc))

    def _write_csv(self, path: str):
        """Write buffered data rows to a CSV file."""
        ch_keys = [ch[0] for ch in _CHANNELS]
        headers = ["timestamp_s"] + ch_keys

        with open(path, "w", newline="", encoding="utf-8") as fh:
            writer = csv.writer(fh)
            writer.writerow(headers)

            n = len(self._timestamps)
            for i in range(n):
                row = [f"{self._timestamps[i]:.4f}"]
                for key in ch_keys:
                    buf = self._data_buffers[key]
                    if i < len(buf) and buf[i] is not None:
                        row.append(f"{buf[i]:.6g}")
                    else:
                        row.append("")
                writer.writerow(row)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _update_button_states(self):
        """Update button enabled/checked state to reflect current mode."""
        self._btn_start.setEnabled(not self._running)
        self._btn_stop.setEnabled(self._running or self._paused)
        self._btn_pause.setEnabled(self._running)
