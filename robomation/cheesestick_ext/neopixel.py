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

import time
from typing import Literal, Union, get_args
from robomation.core.error import _err
from robomation.core.runner import Runner


# ── Module-level Literal aliases (for IDE auto-complete) ─────────────────────
_NeoPixelColor      = Literal['black', 'red', 'yellow', 'green', 'cyan', 'blue', 'magenta', 'white']
_NeoPixelMode       = Literal['GRB', 'GRBW']
_NeoPixelCommand    = Literal['fill', 'change', 'brightness', 'pattern', 'shift', 'clear']
# _NeoPixelChange     = Literal['add', 'sub']
_NeoPixelShiftMode  = Literal['shift', 'rotate']
_NeoPixelShiftDir   = Literal['left', 'right']
_NeoPixelPattern    = Literal['3_colors', '6_colors', '12_colors',
                                'red_green', 'red_blue', 'red_white',
                                'green_red', 'green_blue', 'green_white',
                                'blue_red', 'blue_green', 'blue_white',
                                'white_red', 'white_green', 'white_blue',
                                'black_red', 'black_green', 'black_blue', 'black_white',
                                'red_black', 'green_black', 'blue_black', 'white_black']

class NeoPixel:
    """Addressable LED strip module on CheeseStick."""
    ID = "kr.robomation.physical.module.cheesestick.NeoPixel"
    _modules = {}

    # ── Valid values (derived from module-level Literals) ───────────────────
    _VALID_MODES                = {m: i for i, m in enumerate(get_args(_NeoPixelMode))}
    _VALID_COMMANDS             = {m: i for i, m in enumerate(get_args(_NeoPixelCommand))}
    # _VALID_CHANGE               = {m: i for i, m in enumerate(get_args(_NeoPixelChange))}
    _VALID_SHIFT_MODES          = {m: i for i, m in enumerate(get_args(_NeoPixelShiftMode))}
    _VALID_SHIFT_DIRS           = {m: i for i, m in enumerate(get_args(_NeoPixelShiftDir))}
    _VALID_COLORS = {
        'black':   [  0,   0,   0],
        'red':     [255,   0,   0],
        'yellow':  [255, 255,   0],
        'green':   [  0, 255,   0],
        'cyan':    [  0, 255, 255],
        'blue':    [  0,   0, 255],
        'magenta': [255,   0, 255],
        'white':   [255, 255, 255],
    }
    _VALID_PATTERNS = {
        '3_colors': 192,    '6_colors': 193,    '12_colors': 194,
        'red_green': 128,   'red_blue': 129,    'red_white': 130,
        'green_red': 132,   'green_blue': 133,  'green_white': 134,
        'blue_red': 136,    'blue_green': 137,  'blue_white': 138,
        'white_red': 140,   'white_green': 141, 'white_blue': 142,
        'black_red': 144,   'black_green': 145, 'black_blue': 146,  'black_white': 147,
        'red_black': 131,   'green_black': 135, 'blue_black': 139,  'white_black': 143,
    }

    def __init__(self, parent):
        if parent is None or not hasattr(parent, '_index'):
            return _err(NeoPixel, '__init__', 'parent', parent, 'CheeseStick instance — use cheesestick.NeoPixel()')
        parent_index = parent._index
        if parent_index in NeoPixel._modules:
            mod = NeoPixel._modules[parent_index]
            if mod: mod.dispose()
        self._parent = parent
        self._parent_index = parent_index
        NeoPixel._modules[parent_index] = self

    def dispose(self):
        NeoPixel._modules.pop(self._parent_index, None)

    def _set_fill_range(self, from_idx, to_idx):
        self._parent.write(self._parent.NEO_COMMAND, NeoPixel._VALID_COMMANDS['fill'])
        self._parent.write(self._parent.NEO_FROM, from_idx)
        self._parent.write(self._parent.NEO_TO, to_idx)

    def _set_rgb(self, r, g, b):
        self._parent.write(self._parent.NEO_RED, r)
        self._parent.write(self._parent.NEO_GREEN, g)
        self._parent.write(self._parent.NEO_BLUE, b)
        time.sleep(0.1)  

    # 색 인자가 이름 문자열이면 RGB로 변환, 숫자면 그대로 검증해 반환.
    def _resolve_color(self, method, r, g, b):
        if isinstance(r, str):
            if r not in NeoPixel._VALID_COLORS:
                _err(NeoPixel, method, 'color', r, tuple(NeoPixel._VALID_COLORS))
            return NeoPixel._VALID_COLORS[r]
        for name, val in (('r', r), ('g', g), ('b', b)):
            if not isinstance(val, (int, float)):
                _err(NeoPixel, method, name, val, 'int | float')
        return [r, g, b]
    
    def start(self):
        self._parent.write(self._parent.S_NEO, 1)
        self._parent.write(self._parent.SA_MODE, self._parent._VALID_OUTPUT_MODES['analog_servo'])

    def mode(self, unit: _NeoPixelMode = 'GRBW'):
        if unit not in NeoPixel._VALID_MODES:
            return _err(NeoPixel, 'mode', 'unit', unit, NeoPixel._VALID_MODES)
        self._parent.write(self._parent.NEO_MODE, NeoPixel._VALID_MODES[unit])

    # ── Single pixel ─────────────────────────────────────────────────────────
    def _fill_color_impl(self, from_idx, to_idx, rgb):
        self._set_fill_range(from_idx, to_idx)
        self._set_rgb(*rgb)

    def _change_color_impl(self, from_idx, to_idx, r, g, b):
        self._set_fill_range(from_idx, to_idx)
        self._set_rgb(
            self._parent.read(self._parent.NEO_RED) + r,
            self._parent.read(self._parent.NEO_GREEN) + g,
            self._parent.read(self._parent.NEO_BLUE) + b
        )

    def _fill_increment_color_impl(self, from_idx, to_idx, increment, rgb):
        self._set_fill_range(from_idx, to_idx)
        self._parent.write(self._parent.NEO_INCREMENT, increment)
        self._set_rgb(*rgb)

    def _change_increment_color_impl(self, from_idx, to_idx, increment, r, g, b):
        self._set_fill_range(from_idx, to_idx)
        self._parent.write(self._parent.NEO_INCREMENT, increment)
        self._set_rgb(
            self._parent.read(self._parent.NEO_RED) + r,
            self._parent.read(self._parent.NEO_GREEN) + g,
            self._parent.read(self._parent.NEO_BLUE) + b
        )

    def set_one_color(self, idx: int, r: Union[_NeoPixelColor, int, float], g: int | None = None, b: int | None = None):
        rgb = self._resolve_color('set_one_color', r, g, b)
        Runner.dispatch(lambda: self._fill_color_impl(idx, idx, rgb), True)

    def change_one_color(self, idx: int, r: int, g: int, b: int):
        Runner.dispatch(lambda: self._change_color_impl(idx, idx, r, g, b), True)

    def turn_off_one(self, idx: int):
        Runner.dispatch(lambda: self._fill_color_impl(idx, idx, [0, 0, 0]), True)

    # ── Range ────────────────────────────────────────────────────────────────
    def _set_range_pattern_impl(self, from_idx, to_idx, pattern):
        self._parent.write(self._parent.NEO_COMMAND, NeoPixel._VALID_COMMANDS['pattern'])
        self._parent.write(self._parent.NEO_FROM, from_idx)
        self._parent.write(self._parent.NEO_TO, to_idx)
        self._parent.write(self._parent.NEO_PATTERN_MODE, NeoPixel._VALID_PATTERNS[pattern])
        time.sleep(0.1)  

    def set_range_pattern(self, from_idx: int, to_idx: int, pattern: _NeoPixelPattern):
        if pattern not in NeoPixel._VALID_PATTERNS:
            return _err(NeoPixel, 'set_range_pattern', 'pattern', pattern, NeoPixel._VALID_PATTERNS)
        Runner.dispatch(lambda: self._set_range_pattern_impl(from_idx, to_idx, pattern), True)

    def set_range_color(self, from_idx: int, to_idx: int, r: Union[_NeoPixelColor, int, float], g: int | None = None, b: int | None = None):
        rgb = self._resolve_color('set_range_color', r, g, b)
        Runner.dispatch(lambda: self._fill_color_impl(from_idx, to_idx, rgb), True)

    def change_range_color(self, from_idx: int, to_idx: int, r: int, g: int, b: int):
        Runner.dispatch(lambda: self._change_color_impl(from_idx, to_idx, r, g, b), True)

    def turn_off_range(self, from_idx: int, to_idx: int):
        Runner.dispatch(lambda: self._fill_color_impl(from_idx, to_idx, [0, 0, 0]), True)

    # ── Range increment ──────────────────────────────────────────────────────
    def set_range_increment_color(self, from_idx: int, to_idx: int, increment: int, r: Union[_NeoPixelColor, int, float], g: int | None = None, b: int | None = None):
        rgb = self._resolve_color('set_range_increment_color', r, g, b)
        Runner.dispatch(lambda: self._fill_increment_color_impl(from_idx, to_idx, increment, rgb), True)

    def change_range_increment_color(self, from_idx: int, to_idx: int, increment: int, r: int, g: int, b: int):
        Runner.dispatch(lambda: self._change_increment_color_impl(from_idx, to_idx, increment, r, g, b), True)

    def turn_off_range_increment(self, from_idx: int, to_idx: int, increment: int):
        Runner.dispatch(lambda: self._fill_increment_color_impl(from_idx, to_idx, increment, [0, 0, 0]), True)

    # ── Shift / Rotate ───────────────────────────────────────────────────────
    def _do_shift(self, mode, direction, pixel):
        self._parent.write(self._parent.NEO_COMMAND, NeoPixel._VALID_COMMANDS['shift'])
        self._parent.write(self._parent.NEO_SHIFT_MODE, NeoPixel._VALID_SHIFT_MODES[mode])
        self._parent.write(self._parent.NEO_SHIFT_DIRECTION, NeoPixel._VALID_SHIFT_DIRS[direction])
        self._parent.write(self._parent.NEO_SHIFT_PIXEL, pixel)
        time.sleep(0.1)  

    def shift(self, direction: _NeoPixelShiftDir, pixel: int):
        if direction not in NeoPixel._VALID_SHIFT_DIRS:
            return _err(NeoPixel, 'shift', 'direction', direction, NeoPixel._VALID_SHIFT_DIRS)
        if not isinstance(pixel, (int, float)):
            return _err(NeoPixel, 'shift', 'pixel', pixel, 'int | float')
        Runner.dispatch(lambda: self._do_shift('shift', direction, pixel), True)

    def rotate(self, direction: _NeoPixelShiftDir, pixel: int):
        if direction not in NeoPixel._VALID_SHIFT_DIRS:
            return _err(NeoPixel, 'rotate', 'direction', direction, NeoPixel._VALID_SHIFT_DIRS)
        if not isinstance(pixel, (int, float)):
            return _err(NeoPixel, 'rotate', 'pixel', pixel, 'int | float')
        Runner.dispatch(lambda: self._do_shift('rotate', direction, pixel), True)

    # ── Brightness ───────────────────────────────────────────────────────────
    
    def set_brightness(self, value: Union[int, float]):
        if not isinstance(value, (int, float)):
            return _err(NeoPixel, 'set_brightness', 'value', value, 'int | float')
        self._parent.write(self._parent.NEO_BRIGHTNESS, value)

    def change_brightness(self, value: Union[int, float]):
        if not isinstance(value, (int, float)):
            return _err(NeoPixel, 'change_brightness', 'value', value, 'int | float')
        value = self._parent.read(self._parent.NEO_BRIGHTNESS) + value
        self._parent.write(self._parent.NEO_BRIGHTNESS, value)