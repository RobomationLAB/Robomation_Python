# Based on the ROBOID project - http://hamster.school
# Copyright (c) 2016 Kwang-Hyun Park (akaii@kw.ac.kr)
#
# Modified by Robomation in 2026.
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

import sys
import random
from typing import Literal

from robomation.core.error import _err
from robomation.core.runner import Runner
from robomation.core.keyboard import Keyboard
from robomation.core._internal import _log, _scope, _tts, _sound 

keyboard = None

_Lang = Literal['en-US', 'ko-KR']
_Color = Literal['black', 'red', 'yellow', 'green', 'cyan', 'blue', 'magenta', 'white']

class Utils:
    _VALID_COLORS = {
        'black':   [0, 0, 0],
        'red':     [255, 0, 0],
        'yellow':  [255, 255, 0],
        'green':   [0, 255, 0],
        'cyan':    [0, 255, 255],
        'blue':    [0, 0, 255],
        'magenta': [255, 0, 255],
        'white':   [255, 255, 255],
    }

    @staticmethod
    def round(value):
        if isinstance(value, (int, float)):
            if value < 0:
                return -int(0.5 - value)
            else:
                return int(0.5 + value)
        else:
            return 0
        
    @staticmethod
    def wait(sec):
        Runner.wait(sec)

    @staticmethod
    def wait_forever():
        Runner.wait(-1)

    @staticmethod
    def color(name: _Color):
        if name not in Utils._VALID_COLORS:
            return _err(Utils, 'color', 'name', name, Utils._VALID_COLORS)
        return list(Utils._VALID_COLORS.get(name, [0, 0, 0]))
    
    @staticmethod
    def color_slider(r: int, g: int, b: int):
        if not isinstance(r, int):
            return _err(Utils, 'color_slider', 'r', r, 'int')
        if not isinstance(g, int):
            return _err(Utils, 'color_slider', 'g', g, 'int')
        if not isinstance(b, int):
            return _err(Utils, 'color_slider', 'b', b, 'int')
        return [r, g, b]

    @staticmethod
    def color_rgb(r: int, g: int, b: int):
        if not isinstance(r, int):
            return _err(Utils, 'color_rgb', 'r', r, 'int')
        if not isinstance(g, int):
            return _err(Utils, 'color_rgb', 'g', g, 'int')
        if not isinstance(b, int):
            return _err(Utils, 'color_rgb', 'b', b, 'int')
        return [r, g, b]
    
    @staticmethod
    def random_color():
        return [random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)]

    @staticmethod
    def log(data, tag=None, unit=None):
        # globals()['__log'](data, tag, unit)
        _log.log(data, tag, unit)
        
    @staticmethod
    def scope(signal, name, min_val, max_val, color):
        # globals()['__scope'](name, min_val, max_val, color, signal)
        _scope.scope(signal, name, min_val, max_val, color)

    @staticmethod
    def keypressed(key):
        if key not in Keyboard._KEY_MAP:
            return False
        global keyboard
        if keyboard is None:
            keyboard = Keyboard()
        return keyboard._pressed_keys.get(key, False)

    @staticmethod
    def set_tts(lang: _Lang, name=None):
        # globals()['__setTTSOption'](lang, name)
        _tts.set_tts(lang, name)

    @staticmethod
    def speak(text):
        # globals()['__speak'](text)
        _tts.speak(text)

    @staticmethod
    def play_sound(file, volume=100, repeat=False):
        # globals()['__playSound'](file, volume, repeat)
        _sound.play_sound(file, volume, repeat)

    @staticmethod
    def exit():
        sys.exit(0)

    @staticmethod
    def parallel(*functions):
        Runner.parallel(functions)