# Copyright (c) 2026 Robomation
# 
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
# 
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
# 
# You should have received a copy of the GNU Lesser General
# Public License along with this library; if not, write to the
# Free Software Foundation, Inc., 59 Temple Place, Suite 330,
# Boston, MA  02111-1307  USA

from typing import Literal, Union, get_args
from robomation.core.error import _err


# ── Module-level Literal aliases (for IDE auto-complete) ─────────────────────
_PID13Axis    = Literal['x', 'y']
_PID13Button  = Literal['a', 'b']

class PID13:
    """Joystick & button module on CheeseStick PID slot."""
    ID = "kr.robomation.physical.module.cheesestick.pid13"
    _modules = {}

    _VALID_AXIS    = get_args(_PID13Axis)
    _VALID_BUTTONS = get_args(_PID13Button)

    def __init__(self, parent):
        if parent is None or not hasattr(parent, '_index'):
            return _err(PID13, '__init__', 'parent', parent, 'CheeseStick instance — use cheesestick.PID13()')
        parent_index = parent._index
        if parent_index in PID13._modules:
            mod = PID13._modules[parent_index]
            if mod: mod.dispose()
        self._parent = parent
        self._parent_index = parent_index
        PID13._modules[parent_index] = self

    def dispose(self):
        PID13._modules.pop(self._parent_index, None)

    def start(self):
        self._parent.write(self._parent.PID, 13)

    def joystick(self, unit: _PID13Axis) -> Union[int, float]:
        if unit not in PID13._VALID_AXIS:
            return _err(PID13, 'joystick', 'unit', unit, PID13._VALID_AXIS)
        dev = self._parent.PID13_X if unit == 'x' else self._parent.PID13_Y
        return self._parent.read(dev)

    def button_input(self, unit: _PID13Button) -> int:
        if unit not in PID13._VALID_BUTTONS:
            return _err(PID13, 'button_input', 'unit', unit, PID13._VALID_BUTTONS)
        dev = self._parent.PID13_BUTTON_A if unit == 'a' else self._parent.PID13_BUTTON_B
        return self._parent.read(dev)

    def button_click(self, unit: _PID13Button) -> bool:
        if unit not in PID13._VALID_BUTTONS:
            return _err(PID13, 'button_click', 'unit', unit, PID13._VALID_BUTTONS)
        dev = self._parent.PID13_BUTTON_A_STATE if unit == 'a' else self._parent.PID13_BUTTON_B_STATE
        return self._parent.e(dev)
