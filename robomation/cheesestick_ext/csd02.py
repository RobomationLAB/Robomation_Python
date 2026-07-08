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

from typing import Literal, Union
from robomation.core.error import _err


# ── Module-level Literal aliases (for IDE auto-complete) ─────────────────────
_CSD02Color = Literal['black', 'red', 'orange', 'yellow', 'green', 'cyan', 'blue', 'purple', 'magenta', 'white']

class CSD02:
    """RGB LED module (PWM output on CheeseStick L-port La/Lb/Lc)."""
    ID = "kr.robomation.physical.module.cheesestick.csd02"
    _modules = {}

    _MAX = 30

    _VALID_COLORS = {
        'black':   [  0,   0,   0],
        'red':     [255,   0,   0],
        'orange':  [255, 128,   0],
        'yellow':  [255, 255,   0],
        'green':   [  0, 255,   0],
        'cyan':    [  0, 255, 255],
        'blue':    [  0,   0, 255],
        'violet':  [128,   0, 128],
        'magenta': [255,   0, 255],
        'white':   [255, 255, 255],
    }

    def __init__(self, parent):
        if parent is None or not hasattr(parent, '_index'):
            return _err(CSD02, '__init__', 'parent', parent, 'CheeseStick instance — use cheesestick.CSD02()')
        parent_index = parent._index
        if parent_index in CSD02._modules:
            mod = CSD02._modules[parent_index]
            if mod: mod.dispose()
        self._parent = parent
        self._parent_index = parent_index
        CSD02._modules[parent_index] = self

    def dispose(self):
        CSD02._modules.pop(self._parent_index, None)

    def start(self):
        for port in ('La', 'Lb', 'Lc'):
            self._parent.write(self._parent._MODE_DEVICE_IDS[port], self._parent._VALID_OUTPUT_MODES['pwm'])   # PWM 출력 모드 (포트 mode wire 설정)
            self._parent._set_io_range(port, 0, None, 100, 0, None, CSD02._MAX)

    def _set_rgb(self, r, g, b):
        self._parent.write(self._parent.LA_OUTPUT_PWM, self._parent._analog_to_pwm(r))
        self._parent.write(self._parent.LB_OUTPUT_PWM, self._parent._analog_to_pwm(g))
        self._parent.write(self._parent.LC_OUTPUT_PWM, self._parent._analog_to_pwm(b))

    def set_color(self, r: Union[_CSD02Color, int, float], g: Union[int, None] = None, b: Union[int, None] = None):
        if isinstance(r, str):
            if r not in CSD02._VALID_COLORS:
                return _err(CSD02, 'set_color', 'color', r, tuple(CSD02._VALID_COLORS))
            rgb = CSD02._VALID_COLORS[r]
        else:
            for name, val in (('r', r), ('g', g), ('b', b)):
                if not isinstance(val, (int, float)):
                    return _err(CSD02, 'set_color', name, val, 'int | float')
            rgb = [r, g, b]
        self._set_rgb(*rgb)

    def change_color(self, r: int, g: int, b: int):
        for name, val in (('r', r), ('g', g), ('b', b)):
            if not isinstance(val, (int, float)):
                return _err(CSD02, 'change_color', name, val, 'int | float')
        port_ids = (
            (self._parent.LA_OUTPUT_PWM, r),
            (self._parent.LB_OUTPUT_PWM, g),
            (self._parent.LC_OUTPUT_PWM, b),
        )
        for dev_id, delta in port_ids:
            cur = self._parent._pwm_to_analog(self._parent.read(dev_id))
            self._parent.write(dev_id, self._parent._analog_to_pwm(cur + delta))

    def turn_off(self):
        for port in ('La', 'Lb', 'Lc'):
            self._parent.write(self._parent._PWM_OUT_DEVICE_IDS[port], 0)
