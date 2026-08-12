"""
Entry point for python -m odrive_gui
"""
import sys
import os

# Ensure the tools/odrive and fibre/python paths are available
_this_dir = os.path.dirname(os.path.abspath(__file__))
_tools_dir = os.path.dirname(_this_dir)
_fibre_dir = os.path.join(os.path.dirname(os.path.dirname(_tools_dir)),
                          "Firmware", "fibre", "python")

sys.path.insert(0, _tools_dir)
sys.path.insert(0, _fibre_dir)

from odrive_gui.main_window import main

if __name__ == "__main__":
    main()
