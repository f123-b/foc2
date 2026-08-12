"""
Terminal / console widget for entering ODrive commands.

Provides a read-only output area and a single-line input with
command history and Python-expression evaluation against the
connected ODrive object.
"""

import sys
import traceback
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QTextEdit,
    QLineEdit, QLabel, QPushButton, QScrollBar
)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont, QTextCursor, QColor, QKeyEvent

# ---------------------------------------------------------------------------
# Colour palette (dark theme)
# ---------------------------------------------------------------------------
COLOR_BG = '#181825'                # main background
COLOR_PROMPT = '#89b4fa'            # prompt ">>>"
COLOR_OK = '#a6e3a1'               # successful output
COLOR_ERROR = '#f38ba8'            # error output
COLOR_ECHO = '#cdd6f4'             # echoed command text
COLOR_INFO = '#6c7086'             # informational / help text
COLOR_WARN = '#f9e2af'             # warnings

FONT_FAMILY = 'Consolas, "Courier New", monospace'
FONT_SIZE = 12

MAX_HISTORY = 50


class TerminalWidget(QWidget):
    """
    A simple interactive terminal for sending commands to a connected
    ODrive device.

    Parameters
    ----------
    worker : ODriveWorker
        The background worker thread. Its ``odrv`` property provides
        the connected device object for expression evaluation.
    parent : QWidget, optional
    """

    # Emitted when a command produces output (for status-bar integration)
    command_executed = Signal(str)

    def __init__(self, worker, parent=None):
        super().__init__(parent)
        self._worker = worker
        self._history = []           # list of past commands, newest last
        self._history_index = -1     # -1 means "at the live prompt"
        self._current_line = ''      # text being typed before history navigation

        self._setup_ui()
        self._print_welcome()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)

        # --- Header label ---
        header = QLabel("  ODrive 终端")
        header.setFont(QFont(FONT_FAMILY, 10))
        header.setStyleSheet(
            f"color: {COLOR_INFO}; background-color: #11111b; "
            f"padding: 2px 6px; border: none;"
        )
        header.setMaximumHeight(22)
        layout.addWidget(header)

        # --- Read-only output area ---
        self._output = QTextEdit()
        self._output.setReadOnly(True)
        self._output.setFont(QFont(FONT_FAMILY, FONT_SIZE))
        self._output.setStyleSheet(
            f"QTextEdit {{"
            f"  background-color: {COLOR_BG};"
            f"  color: {COLOR_ECHO};"
            f"  border: 1px solid #313244;"
            f"  border-radius: 0px;"
            f"  padding: 4px;"
            f"  selection-background-color: #45475a;"
            f"}}"
        )
        # Allow the output area to shrink/grow with the widget
        self._output.setMinimumHeight(60)
        layout.addWidget(self._output, 1)

        # --- Input row: prompt + line edit + send button ---
        input_row = QHBoxLayout()
        input_row.setSpacing(4)

        prompt = QLabel(">>>")
        prompt.setFont(QFont(FONT_FAMILY, FONT_SIZE, QFont.Bold))
        prompt.setStyleSheet(f"color: {COLOR_PROMPT}; background: transparent;")
        prompt.setFixedWidth(36)
        input_row.addWidget(prompt)

        self._input = QLineEdit()
        self._input.setFont(QFont(FONT_FAMILY, FONT_SIZE))
        self._input.setStyleSheet(
            f"QLineEdit {{"
            f"  background-color: #1e1e2e;"
            f"  color: {COLOR_ECHO};"
            f"  border: 1px solid #45475a;"
            f"  border-radius: 3px;"
            f"  padding: 3px 6px;"
            f"  selection-background-color: #585b70;"
            f"}}"
            f"QLineEdit:focus {{ border: 1px solid {COLOR_PROMPT}; }}"
        )
        self._input.setPlaceholderText("输入命令，如 odrv0.vbus_voltage 或 help() ...")
        self._input.returnPressed.connect(self._on_execute)
        self._input.installEventFilter(self)
        input_row.addWidget(self._input, 1)

        send_btn = QPushButton("发送")
        send_btn.setFont(QFont("", 10))
        send_btn.setFixedWidth(48)
        send_btn.clicked.connect(self._on_execute)
        send_btn.setStyleSheet(
            f"QPushButton {{"
            f"  background-color: #45475a;"
            f"  color: {COLOR_ECHO};"
            f"  border: 1px solid #585b70;"
            f"  border-radius: 3px;"
            f"  padding: 3px 8px;"
            f"}}"
            f"QPushButton:hover {{ background-color: #585b70; }}"
            f"QPushButton:pressed {{ background-color: #313244; }}"
        )
        input_row.addWidget(send_btn)

        layout.addLayout(input_row)

        # Set initial focus on the input line
        self._input.setFocus()

    def eventFilter(self, obj, event):
        """
        Intercept key events on the QLineEdit for history navigation.
        """
        if obj is self._input and event.type() == QKeyEvent.KeyPress:
            key = event.key()
            if key == Qt.Key_Up:
                self._history_prev()
                return True
            if key == Qt.Key_Down:
                self._history_next()
                return True
        return super().eventFilter(obj, event)

    # ------------------------------------------------------------------
    # History navigation
    # ------------------------------------------------------------------

    def _history_prev(self):
        """Move backward through command history."""
        if not self._history:
            return

        # First press: save the current in-progress text
        if self._history_index == -1:
            self._current_line = self._input.text()

        if self._history_index < len(self._history) - 1:
            self._history_index += 1
            idx = len(self._history) - 1 - self._history_index
            self._input.setText(self._history[idx])
            self._input.setCursorPosition(len(self._history[idx]))

    def _history_next(self):
        """Move forward through command history."""
        if self._history_index <= 0:
            self._history_index = -1
            self._input.setText(self._current_line)
            self._input.setCursorPosition(len(self._current_line))
            return

        self._history_index -= 1
        idx = len(self._history) - 1 - self._history_index
        self._input.setText(self._history[idx])
        self._input.setCursorPosition(len(self._history[idx]))

    # ------------------------------------------------------------------
    # Command execution
    # ------------------------------------------------------------------

    def _on_execute(self, *_):
        """Handle the Enter key or Send button."""
        text = self._input.text().strip()
        if not text:
            return

        # Add to history (avoid consecutive duplicates)
        if not self._history or self._history[-1] != text:
            self._history.append(text)
            if len(self._history) > MAX_HISTORY:
                self._history.pop(0)

        # Reset history navigation state
        self._history_index = -1
        self._current_line = ''

        # Echo the command
        self._append_html(f'<span style="color:{COLOR_PROMPT};">&gt;&gt;&gt; </span>'
                          f'<span style="color:{COLOR_ECHO};">{self._htext(text)}</span>')

        # Execute
        self._input.clear()

        # Run in the main thread (we are already in the main thread, but
        # we guard against long operations that would block the UI).
        try:
            result = self._execute_command(text)
            if result is not None:
                self._append_html(
                    f'<span style="color:{COLOR_OK};">{self._htext(str(result))}</span>')
        except Exception:
            tb = traceback.format_exc()
            # Only show the last line of the traceback for readability
            lines = tb.strip().split('\n')
            self._append_html(
                f'<span style="color:{COLOR_ERROR}; white-space: pre-wrap;">'
                f'{self._htext(tb)}</span>')

        self.command_executed.emit(text)

        # Scroll to bottom
        vsb = self._output.verticalScrollBar()
        vsb.setValue(vsb.maximum())

    def _execute_command(self, text):
        """
        Evaluate or execute *text* in a controlled namespace.

        Returns the result of ``eval``, or ``None`` for an exec /
        assignment statement.

        The namespace contains:

        - ``odrv0``  -- the connected device (``worker.odrv``) or ``None``
        - ``worker`` -- the ODriveWorker instance
        - ``odrive`` -- the top-level ``odrive`` package
        - All ODrive constants from ``odrive.enums``
        """
        # Special built-in commands
        lower = text.lower()
        if lower in ('help', 'help()'):
            self._print_help()
            return None

        if lower == 'clear' or lower == 'cls':
            self._output.clear()
            return None

        if lower in ('history', 'hist', 'history()'):
            self._print_history()
            return None

        if lower.startswith('help(') or lower == '?':
            return self._handle_help_topic(text)

        # Build execution namespace
        import odrive
        import odrive.enums

        odrv = self._worker.odrv

        namespace = {
            'odrv0': odrv,
            'worker': self._worker,
            'odrive': odrive,
        }
        # Load all public names from odrive.enums
        for name in dir(odrive.enums):
            if name[0].isupper() or name.startswith('_'):
                continue
            val = getattr(odrive.enums, name, None)
            if val is not None and not callable(val):
                namespace[name] = val

        # Decide eval-vs-exec: if text looks like an assignment or is a
        # multi-line / compound statement, use exec.
        is_assignment = (
            '=' in text and
            not text.strip().startswith('==') and
            not text.strip().startswith('!=') and
            not text.strip().startswith('<=') and
            not text.strip().startswith('>=') and
            not text.strip().startswith(('if ', 'for ', 'while ', 'def ', 'class ', 'import ', 'from '))
        )

        if is_assignment:
            # Use exec for assignments (does not return a value)
            exec(text, namespace)
            return None
        else:
            # Try eval first; fall back to exec if it's a statement
            try:
                code = compile(text, '<terminal>', 'eval')
            except SyntaxError:
                # Not an expression -- treat as exec
                exec(text, namespace)
                return None

            return eval(code, namespace)

    # ------------------------------------------------------------------
    # Output helpers
    # ------------------------------------------------------------------

    def _append_html(self, html):
        """Append a line of HTML to the output area."""
        # Ensure a fresh block
        if self._output.document().blockCount() > 1 or self._output.toPlainText():
            prefix = '<br>'
        else:
            prefix = ''
        self._output.moveCursor(QTextCursor.End)
        self._output.textCursor().insertHtml(prefix + html)
        self._output.moveCursor(QTextCursor.End)

    def _append_text(self, text, color=COLOR_ECHO):
        """Append plain text (auto-HTML-escaped) to the output area."""
        self._append_html(f'<span style="color:{color};">{self._htext(text)}</span>')

    @staticmethod
    def _htext(s):
        """Minimal HTML-escape for terminal output."""
        return (str(s)
                .replace('&', '&amp;')
                .replace('<', '&lt;')
                .replace('>', '&gt;')
                .replace('\n', '<br>'))

    # ------------------------------------------------------------------
    # Built-in commands
    # ------------------------------------------------------------------

    def _print_welcome(self):
        """Print welcome banner."""
        self._append_html(
            f'<span style="color:{COLOR_INFO};">'
            f'ODrive 交互终端 — 固件 v0.5.1<br>'
            f'输入 <b>help()</b> 查看可用命令<br>'
            f'</span>'
        )

    def _print_help(self):
        """Print the full help text."""
        lines = [
            '<span style="color:#f5c2e7; font-weight:bold;">'
            '═══════════ ODrive 终端帮助 ═══════════</span>',

            f'<span style="color:{COLOR_WARN};">'
            '── 读取属性 ──</span>'
            f'<span style="color:{COLOR_OK};">'
            '  odrv0.vbus_voltage          - 母线电压<br>'
            '  odrv0.axis0.motor.current_control.Iq_measured - Iq 电流<br>'
            '  odrv0.axis0.encoder.pos_estimate - 位置估算值<br>'
            '  odrv0.axis0.current_state   - 当前状态<br>'
            '  odrv0.axis0.error           - 轴错误码<br>'
            '  odrv0.axis0.motor.error     - 电机错误码<br>'
            '</span>',

            f'<span style="color:{COLOR_WARN};">'
            '── 写入属性 ──</span>'
            f'<span style="color:{COLOR_ECHO};">'
            '  odrv0.axis0.requested_state = 8     - 进入闭环控制<br>'
            '  odrv0.axis0.controller.input_pos = 2.0  - 设置位置<br>'
            '  odrv0.axis0.controller.input_vel = 1.0  - 设置速度<br>'
            '  odrv0.axis0.controller.input_torque = 0.1 - 设置扭矩<br>'
            '</span>',

            f'<span style="color:{COLOR_WARN};">'
            '── 函数调用 ──</span>'
            f'<span style="color:{COLOR_OK};">'
            '  odrv0.axis0.clear_errors()  - 清除轴错误<br>'
            '  odrv0.save_configuration()  - 保存配置到 Flash<br>'
            '  odrv0.reboot()              - 重启 ODrive<br>'
            '</span>',

            f'<span style="color:{COLOR_WARN};">'
            '── 内置命令 ──</span>'
            f'<span style="color:{COLOR_INFO};">'
            '  help()     - 显示此帮助<br>'
            '  history    - 显示命令历史<br>'
            '  clear      - 清空终端<br>'
            '</span>',

            f'<span style="color:{COLOR_INFO};">'
            '── 枚举常量 (已自动导入) ──<br>'
            '  AXIS_STATE_IDLE, AXIS_STATE_CLOSED_LOOP_CONTROL, ...<br>'
            '  CONTROL_MODE_VELOCITY_CONTROL, ...<br>'
            '</span>',

            f'<span style="color:#f5c2e7;">'
            '══════════════════════════════════════'
            '</span>',
        ]
        for line in lines:
            self._append_html(line)

    def _print_history(self):
        """Print the command history."""
        if not self._history:
            self._append_html(
                f'<span style="color:{COLOR_INFO};">（命令历史为空）</span>')
            return

        for i, cmd in enumerate(self._history):
            self._append_html(
                f'<span style="color:{COLOR_INFO};">{i:4d}</span>  '
                f'<span style="color:{COLOR_ECHO};">{self._htext(cmd)}</span>')

    def _handle_help_topic(self, text):
        """Show help for a specific topic string."""
        topic = text[5:-1].strip().strip("'").strip('"')
        if not topic:
            self._print_help()
            return None

        # Try to look up the attribute
        odrv = self._worker.odrv
        try:
            if topic == 'odrv0' or topic == 'odrv':
                obj = odrv
            elif topic.startswith('odrv0.'):
                obj = odrv
                for part in topic[6:].split('.'):
                    obj = getattr(obj, part)
            else:
                obj = getattr(odrv, topic, None)

            if obj is not None:
                doc = getattr(obj, '__doc__', None) or ''
                val = repr(obj) if not callable(obj) else '<function/method>'
                self._append_html(
                    f'<span style="color:{COLOR_OK}; font-weight:bold;">{self._htext(topic)}</span>'
                    f'<span style="color:{COLOR_ECHO};"> = {self._htext(val)}</span>')
                if doc:
                    self._append_html(
                        f'<span style="color:{COLOR_INFO};">{self._htext(doc[:500])}</span>')
                return None
        except Exception:
            pass

        self._append_html(
            f'<span style="color:{COLOR_ERROR};">未知主题: {self._htext(topic)}</span>')
        return None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def clear(self):
        """Clear the output area."""
        self._output.clear()
        self._print_welcome()

    def write(self, text, color=None):
        """
        Programmatically write a line to the terminal.

        Parameters
        ----------
        text : str
            The text to append.
        color : str, optional
            CSS color string. Defaults to COLOR_ECHO.
        """
        self._append_text(text, color or COLOR_ECHO)
        vsb = self._output.verticalScrollBar()
        vsb.setValue(vsb.maximum())

    def write_error(self, text):
        """Write an error line in red."""
        self.write(text, COLOR_ERROR)

    def setFocus(self):
        """Focus the input line."""
        super().setFocus()
        self._input.setFocus()
