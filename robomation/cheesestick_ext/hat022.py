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

from typing import Literal, get_args
from robomation.core.error import _err


# ── Module-level Literal aliases (for IDE auto-complete) ─────────────────────
_HAT022Key    = Literal['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B', 'left', 'right', 'fn']
# _HAT022Button = Literal['left', 'right', 'Fn']

class HAT022:
    """Touch piano module on CheeseStick HAT slot."""
    ID = "kr.robomation.physical.module.cheesestick.hat022"
    _modules = {}

    _VALID_KEYS    = get_args(_HAT022Key)
    # _VALID_BUTTONS = get_args(_HAT022Button)

    def __init__(self, parent):
        if parent is None or not hasattr(parent, '_index'):
            return _err(HAT022, '__init__', 'parent', parent, 'CheeseStick instance — use cheesestick.HAT022()')
        parent_index = parent._index
        if parent_index in HAT022._modules:
            mod = HAT022._modules[parent_index]
            if mod: mod.dispose()
        self._parent = parent
        self._parent_index = parent_index
        HAT022._modules[parent_index] = self

    def dispose(self):
        HAT022._modules.pop(self._parent_index, None)

    def start(self):
        self._parent.write(self._parent.HAT, 22)

    def _key_device(self, key):
        device = {
            'C': self._parent.HAT022_C,         'C#': self._parent.HAT022_C_SHARP,
            'D': self._parent.HAT022_D,         'D#': self._parent.HAT022_D_SHARP,
            'E': self._parent.HAT022_E,
            'F': self._parent.HAT022_F,         'F#': self._parent.HAT022_F_SHARP,
            'G': self._parent.HAT022_G,         'G#': self._parent.HAT022_G_SHARP,
            'A': self._parent.HAT022_A,         'A#': self._parent.HAT022_A_SHARP,
            'B': self._parent.HAT022_B,         
            'left': self._parent.HAT022_LEFT,   'right': self._parent.HAT022_RIGHT,
            'fn': self._parent.HAT022_FN,
        }
        return device[key]

    def key_input(self, unit: _HAT022Key) -> int:
        if unit not in HAT022._VALID_KEYS:
            return _err(HAT022, 'key_input', 'unit', unit, HAT022._VALID_KEYS)
        return self._parent.read(self._key_device(unit))

    def key_pressed(self, unit: _HAT022Key) -> bool:
        if unit not in HAT022._VALID_KEYS:
            return _err(HAT022, 'key_pressed', 'unit', unit, HAT022._VALID_KEYS)
        return self._parent.read(self._key_device(unit)) == 1

    '''
    def _button_device(self, button):
        device = {
            'left': self._parent.HAT022_LEFT, 'right': self._parent.HAT022_RIGHT, 'Fn': self._parent.HAT022_FN
        }
        return device[button]
        
    def button_input(self, unit: _HAT022Button) -> int:
        if unit not in HAT022._VALID_BUTTONS:
            return _err(HAT022, 'button_input', 'unit', unit, HAT022._VALID_BUTTONS)
        return self._parent.read(self._button_device(unit))

    def button_pressed(self, unit: _HAT022Button) -> bool:
        if unit not in HAT022._VALID_BUTTONS:
            return _err(HAT022, 'button_pressed', 'unit', unit, HAT022._VALID_BUTTONS)
        return self._parent.read(self._button_device(unit)) == 1
    '''
