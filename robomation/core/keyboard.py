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


# JS BlockComposer 키 이름 컨벤션 — 사용자가 Utils.keypressed(key) 에 넘기는 이름들.
_VALID_KEYS = frozenset([
    'space',
    *'abcdefghijklmnopqrstuvwxyz',
    *'0123456789',
    'up', 'down', 'left', 'right',
    'shift', 'control', 'alt',
    'enter', 'tab', 'esc', 'backspace',
])


class Keyboard(object):
    # 사용자 호출 호환용 (Utils.keypressed 가 _KEY_MAP 검증에 사용)
    _KEY_MAP = _VALID_KEYS

    def __init__(self):
        try:
            from pynput import keyboard
        except ImportError as e:
            raise ImportError(
                "Utils.keypressed() requires the 'pynput' package.\n"
                "Install with:  pip install pynput"
            ) from e

        self._pressed_keys = {}

        # pynput Key enum → 우리 이름 매핑 (모든 platform 공통).
        # 좌/우 modifier 변형은 같은 이름으로 통합 (shift_r → 'shift' 등)
        Key = keyboard.Key
        self._special_names = {
            Key.space:     'space',
            Key.up:        'up',
            Key.down:      'down',
            Key.left:      'left',
            Key.right:     'right',
            Key.enter:     'enter',
            Key.tab:       'tab',
            Key.esc:       'esc',
            Key.backspace: 'backspace',
            Key.shift:     'shift',
            Key.shift_r:   'shift',
            Key.ctrl:      'control',
            Key.ctrl_r:    'control',
            Key.alt:       'alt',
            Key.alt_r:     'alt',
        }
        # alt_gr 은 Windows/Linux 일부 키보드 — 있으면 추가
        if hasattr(Key, 'alt_gr'):
            self._special_names[Key.alt_gr] = 'alt'

        listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)
        listener.daemon = True
        listener.start()
        self._listener = listener

    def _key_name(self, key):
        """pynput 이벤트의 key → 우리 키 이름 (string) 또는 None (미지원 키)."""
        # 특수 키 (Key enum)
        name = self._special_names.get(key)
        if name is not None:
            return name
        # 문자/숫자 키 (KeyCode 객체, .char 속성)
        char = getattr(key, 'char', None)
        if char:
            c = char.lower()
            if c in _VALID_KEYS:
                return c
        return None

    def _on_press(self, key):
        name = self._key_name(key)
        if name is not None:
            self._pressed_keys[name] = True

    def _on_release(self, key):
        name = self._key_name(key)
        if name is not None:
            self._pressed_keys[name] = False
