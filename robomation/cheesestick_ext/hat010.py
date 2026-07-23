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
_HAT010Button = Literal['a', 'b']
_HAT010Color  = Literal['blue', 'green', 'cyan', 'red', 'magenta', 'yellow', 'white', 'orange', 'violet']
_HAT010Shape  = Literal['rectangle', 'triangle', 'diamond', 'circle', 'x', 'like', 'dislike', 'angry',
                        'mouth_open', 'mouth_close', 'walking', 'heart', 'star', 'airplane', 'dog', 'butterfly',
                        'quarter_note', 'eighth_note', 'up_arrow', 'left_arrow', 'right_arrow', 'down_arrow']
_HAT010Number = Literal[0, 1, 2, 3, 4, 5, 6, 7, 8, 9]

class HAT010:
    """5x5 LED matrix module on CheeseStick HAT slot."""
    ID = "kr.robomation.physical.module.cheesestick.hat010"
    _modules = {}

    # ── Valid values for enum parameters ─────────────────────────────────────
    _VALID_BUTTONS = get_args(_HAT010Button)
    _VALID_COLORS  = {c: i+1 for i, c in enumerate(get_args(_HAT010Color))}
    _VALID_SHAPE   = {
        'rectangle': [1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1],
        'triangle': [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0],
        'diamond': [0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0],
        'circle': [0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 1, 1, 1, 0],
        'x': [1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1],
        'like': [0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1, 0],
        'dislike': [0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 0, 0, 0, 1],
        'angry': [1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1],
        'mouth_open': [0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1, 0],
        'mouth_close': [0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0],
        'walking': [0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0],
        'heart': [0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0],
        'star': [0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0, 0, 1],
        'airplane': [0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 0, 0],
        'dog': [0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 0, 1, 0],
        'butterfly': [1, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 1],
        'quarter_note': [0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0],
        'eighth_note': [0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0],
        'up_arrow': [0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0],
        'left_arrow': [0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0],
        'right_arrow': [0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0],
        'down_arrow': [0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0],
    }
    _VALID_NUMBER   = {
        0: [0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0],
        1: [0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0],
        2: [0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0],
        3: [0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0],
        4: [0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0],
        5: [1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 0, 0],
        6: [0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0],
        7: [1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0],
        8: [0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0],
        9: [0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0],
    }

    def __init__(self, parent):
        if parent is None or not hasattr(parent, '_index'):
            return _err(HAT010, '__init__', 'parent', parent, 'CheeseStick instance — use cheesestick.HAT010()')
        parent_index = parent._index
        if parent_index in HAT010._modules:
            mod = HAT010._modules[parent_index]
            if mod: mod.dispose()
        self._parent = parent
        self._parent_index = parent_index
        HAT010._modules[parent_index] = self

    def dispose(self):
        HAT010._modules.pop(self._parent_index, None)

    def start(self):
        self._parent.write(self._parent.HAT, 10)

    def _evaluate_led(self):
        return self._parent.e(self._parent.HAT010_LED_STATE)

    def _evaluate_draw(self):
        return self._parent.e(self._parent.HAT010_DRAW_STATE)

    def _set_one_impl(self, x, y, color):
        self._parent.write(self._parent.HAT010_X, x)
        self._parent.write(self._parent.HAT010_Y, y)
        self._parent.write(self._parent.HAT010_LED, HAT010._VALID_COLORS[color])
        Runner.wait_until(self._evaluate_led)

    def set_one(self, x: int, y: int, color: _HAT010Color):
        if color not in HAT010._VALID_COLORS:
            return _err(HAT010, 'set_one', 'color', color, tuple(HAT010._VALID_COLORS))
        Runner.dispatch(lambda: self._set_one_impl(x, y, color), True)

    def _draw_impl(self, x, y, color, shape_data):
        self._parent.write(self._parent.HAT010_X, x)
        self._parent.write(self._parent.HAT010_Y, y)
        color_val = HAT010._VALID_COLORS[color]
        draw_data = [(1 if cell > 0 else 0) * color_val for cell in shape_data]
        self._parent.write(self._parent.HAT010_DRAW, draw_data)
        Runner.wait_until(self._evaluate_draw)

    def draw_shape(self, x: int, y: int, color: _HAT010Color, shape: Union[_HAT010Shape, list]):
        if color not in HAT010._VALID_COLORS:
            return _err(HAT010, 'draw_shape', 'color', color, tuple(HAT010._VALID_COLORS))
        if isinstance(shape, str):
            if shape not in HAT010._VALID_SHAPE:
                return _err(HAT010, 'draw_shape', 'shape', shape, tuple(HAT010._VALID_SHAPE))
            shape_data = HAT010._VALID_SHAPE[shape]
        elif isinstance(shape, list):
            shape_data = shape
        else:
            return _err(HAT010, 'draw_shape', 'shape', shape, 'str (shape name) | list (25 cells)')
        Runner.dispatch(lambda: self._draw_impl(x, y, color, shape_data), True)

    def draw_number(self, x: int, y: int, color: _HAT010Color, number: Union[_HAT010Number, list]):
        if color not in HAT010._VALID_COLORS:
            return _err(HAT010, 'draw_number', 'color', color, tuple(HAT010._VALID_COLORS))
        if isinstance(number, int):
            if number not in HAT010._VALID_NUMBER:
                return _err(HAT010, 'draw_number', 'number', number, tuple(HAT010._VALID_NUMBER))
            shape_data = HAT010._VALID_NUMBER[number]
        elif isinstance(number, list):
            shape_data = number
        else:
            return _err(HAT010, 'draw_number', 'number', number, 'int (0-9) | list (25 cells)')
        Runner.dispatch(lambda: self._draw_impl(x, y, color, shape_data), True)

    def draw_pattern(self, x: int, y: int, color: _HAT010Color, matrix: list):
        if color not in HAT010._VALID_COLORS:
            return _err(HAT010, 'draw_pattern', 'color', color, tuple(HAT010._VALID_COLORS))
        if not isinstance(matrix, list):
            return _err(HAT010, 'draw_pattern', 'matrix', matrix, 'list (25 cells)')
        Runner.dispatch(lambda: self._draw_impl(x, y, color, matrix), True)

    def shift(self, x: int, y: int):
        self._parent.write(self._parent.HAT010_ORIGIN_X, self._parent.read(self._parent.HAT010_ORIGIN_X) + x)
        self._parent.write(self._parent.HAT010_ORIGIN_Y, self._parent.read(self._parent.HAT010_ORIGIN_Y) + y)

    def _turn_off_one_impl(self, x, y):
        self._parent.write(self._parent.HAT010_X, x)
        self._parent.write(self._parent.HAT010_Y, y)
        self._parent.write(self._parent.HAT010_LED, 0)
        Runner.wait_until(self._evaluate_led)

    def turn_off_one(self, x: int, y: int):
        Runner.dispatch(lambda: self._turn_off_one_impl(x, y), True)

    def _turn_off_all_impl(self):
        # self._parent.write(self._parent.HAT010_CLEAR, 1)
        # time.sleep(0.1)
        self._parent.write(self._parent.HAT010_X, 0)
        self._parent.write(self._parent.HAT010_Y, 0)
        self._parent.write(self._parent.HAT010_LED, 0)
        self._parent.write(self._parent.HAT010_DRAW, [0] * 25)
        Runner.wait_until(self._evaluate_draw)

    def turn_off_all(self):
        Runner.dispatch(lambda: self._turn_off_all_impl(), True)

    def set_brightness(self, value: Union[int, float]):
        if not isinstance(value, (int, float)):
            return _err(HAT010, 'set_brightness', 'value', value, 'int | float')
        self._parent.write(self._parent.HAT010_BRIGHTNESS, value)

    def change_brightness(self, value: Union[int, float]):
        if not isinstance(value, (int, float)):
            return _err(HAT010, 'change_brightness', 'value', value, 'int | float')
        dev = self._parent.HAT010_BRIGHTNESS
        self._parent.write(dev, self._parent.read(dev) + value)

    def button_input(self, unit: _HAT010Button) -> int:
        if not isinstance(unit, str):
            _err(HAT010, 'button_input', 'unit', unit, 'str')
        unit = unit.lower()
        if unit not in HAT010._VALID_BUTTONS:
            return _err(HAT010, 'button_input', 'unit', unit, HAT010._VALID_BUTTONS)
        dev = self._parent.HAT010_BUTTON_A if unit == 'a' else self._parent.HAT010_BUTTON_B
        return self._parent.read(dev)

    def button_click(self, unit: _HAT010Button) -> bool:
        if not isinstance(unit, str):
            _err(HAT010, 'button_click', 'unit', unit, 'str')
        unit = unit.lower()
        if unit not in HAT010._VALID_BUTTONS:
            return _err(HAT010, 'button_click', 'unit', unit, HAT010._VALID_BUTTONS)
        dev = self._parent.HAT010_BUTTON_A_STATE if unit == 'a' else self._parent.HAT010_BUTTON_B_STATE
        return self._parent.e(dev)
