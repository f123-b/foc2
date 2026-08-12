#!/usr/bin/env python
"""
ODrive GUI - Graphical control interface for ODrive motor controllers.
Standalone launch script.
"""
import sys
import os

_this_dir = os.path.dirname(os.path.abspath(__file__))
_tools_dir = os.path.dirname(_this_dir)
_fibre_dir = os.path.join(os.path.dirname(os.path.dirname(_tools_dir)),
                          "Firmware", "fibre", "python")

sys.path.insert(0, _tools_dir)
sys.path.insert(0, _fibre_dir)

from odrive_gui.main_window import main

if __name__ == "__main__":
    main()
