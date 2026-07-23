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

from typing import Literal, Union, get_args

from robomation.core.error import _err
from robomation.core.runner import Runner
from robomation.core.model import Robot


# ── Module-level Literal aliases (for IDE auto-complete) ─────────────────────
_WheelUnit   = Literal['left', 'right', 'both']
_DistUnit    = Literal['cm', 'mm', 'inch']
_TurnDir     = Literal['left', 'right']
_GridMove    = Literal['forward', 'backward', 'left', 'right']
_GridTurn    = Literal['left', 'right']
_EyeUnit     = Literal['left', 'right', 'both']
_EyeColor    = Literal['black', 'red', 'yellow', 'green', 'cyan', 'blue', 'magenta', 'white']
_EyePattern  = Literal['reset', 'blink', 'dimming', 'rainbow']
_Note        = Literal['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B']
_Clip        = Literal['mute', 'beep', 'beep2', 'beep3', 'beep_repeat', 'beep_random', 'beep_random_repeat',
                          'noise', 'noise_repeat', 'siren', 'siren_repeat', 'engine', 'engine_repeat',
                          'fart_a', 'fart_b', 'noise_random', 'noise_random_repeat', 'whistle', 'chop', 'chop_repeat',
                          'random_melody', 'robot', 'connect']
_Melody      = Literal['mute', 'happy', 'angry', 'sad', 'sleep', 'march', 'birthday', 'dibidibidip', 'good_job',
                          'robot_poo', 'robot_bath', 'robot_sad', 'robot_happy', 'robot_angry', 'robot_sleep']
_KeypadButton = Literal['none', 'play', 'forward', 'backward', 'left', 'right', 'action', 'repeat', 'clear']
_SensorSide  = Literal['left', 'right']
# _Code        = Literal['none', 'forward', 'backward', 'left', 'right', 'action', 'repeat']


class Pio(Robot):
    ID = "kr.robomation.physical.robot.pio"
    _robots = {}

    # ── Device IDs (Pio 펌웨어 디바이스 식별자) ──────────────
    # Effectors (0x020000xx)
    TURBO               = 0x02000000
    LEFT_WHEEL          = 0x02000001
    RIGHT_WHEEL         = 0x02000002
    NECK_SPEED          = 0x02000003
    LEFT_RGB            = 0x02000004
    LEFT_COLOR          = 0x02000005
    LEFT_BRIGHTNESS     = 0x02000006
    LEFT_PATTERN_COLOR  = 0x02000007
    RIGHT_RGB           = 0x02000008
    RIGHT_COLOR         = 0x02000009
    RIGHT_BRIGHTNESS    = 0x0200000a
    RIGHT_PATTERN_COLOR = 0x0200000b
    SOUND_BUZZ          = 0x0200000c

    # Commands (0x020001xx)
    WHEEL_MOVE          = 0x02000100
    NECK_ANGLE          = 0x02000101
    EYE_PATTERN         = 0x02000102
    SOUND_NOTE          = 0x02000103
    SOUND_CLIP          = 0x02000104
    SOUND_MELODY        = 0x02000105
    # BEHAVIOR            = 0x02000106

    # Sensors (0x020002xx)
    WHEEL_COUNT         = 0x02000200
    WHEEL_MOVING        = 0x02000201
    NECK_COUNT          = 0x02000202
    NECK_MOVING         = 0x02000203
    SOUND_PLAYING       = 0x02000204
    KEYPAD              = 0x02000205
    BATTERY             = 0x02000206
    SIGNAL_STRENGTH     = 0x02000207
    # CODES               = 0x02000208

    # Events (0x020003xx)
    WHEEL_STATE         = 0x02000300
    NECK_STATE          = 0x02000301
    EYE_PATTERN_STATE       = 0x02000302
    SOUND_STATE         = 0x02000303
    KEYPAD_STATE        = 0x02000304
    # BEHAVIOR_STATE      = 0x02000305

    # ── Robot-specific constants ──────────────────────────────────────────────
    _SPEED = 50
    _VAL_TO_SPEED = 1
    _CM_TO_PULSE = 968
    _DEG_TO_PULSE = 41.955

    # ── Valid values (derived from module-level Literals) ────────────────────
    _VALID_WHEEL_UNITS    = get_args(_WheelUnit)
    _VALID_DIST_UNITS     = get_args(_DistUnit)
    _VALID_TURN_DIRS      = get_args(_TurnDir)
    _VALID_GRID_MOVES     = get_args(_GridMove)
    _VALID_GRID_TURNS     = get_args(_GridTurn)
    _VALID_EYE_UNITS      = get_args(_EyeUnit)
    _VALID_EYE_COLOR      = get_args(_EyeColor)
    _VALID_COLORS         = {
        'black':   [  0,   0,   0],
        'red':     [255,   0,   0],
        'yellow':  [255, 255,   0],
        'green':   [  0, 255,   0],
        'cyan':    [  0, 255, 255],
        'blue':    [  0,   0, 255],
        'magenta': [255,   0, 255],
        'white':   [255, 255, 255],
    }
    _VALID_EYE_PATTERN    = {p: i for i, p in enumerate(get_args(_EyePattern))}   # 0~3
    _VALID_NOTES = {n: i for i, n in enumerate(get_args(_Note))}
    _VALID_CLIPS  = {c: i for i, c in enumerate(get_args(_Clip))}
    _VALID_MELODYS = {m: i for i, m in enumerate(get_args(_Melody))}
    _VALID_KEYPAD = {
        'none': 0, 'play': 1, 'forward': 2, 'backward': 4, 'left': 8, 'right': 16, 
        'action': 32, 'repeat': 64, 'clear': 128,
    }
    _VALID_SENSOR_SIDES = get_args(_SensorSide)
    # _VALID_CODES   = {c: i for i, c in enumerate(get_args(_Code))}

    # ── Robot lifecycle ───────────────────────────────────────────────────────
    def __init__(self, index=0, port_name=None):
        if isinstance(index, str):
            port_name = index
            index = 0
        if index in Pio._robots:
            robot = Pio._robots[index]
            if robot: robot.dispose()
        Pio._robots[index] = self
        super(Pio, self).__init__(Pio.ID, "Pio", index)

        # Wheel-speed snapshot 상태 
        self._saved_wheel = None
        self._wrote_wheel = None

        self._init(port_name)

    def dispose(self):
        Pio._robots[self.get_index()] = None
        self._roboid._dispose()
        Runner.unregister_robot(self)

    def reset(self):
        self._roboid._reset()

    def _init(self, port_name):
        from robomation.roboids.pio_roboid import PioRoboid
        self._roboid = PioRoboid(self.get_index())
        self._add_roboid(self._roboid)
        Runner.register_robot(self)
        Runner.start()
        self._roboid._init(port_name)

    def find_device_by_id(self, device_id):
        return self._roboid.find_device_by_id(device_id)

    def _request_motoring_data(self):
        self._roboid._request_motoring_data()
    def _update_sensory_device_state(self):
        self._roboid._update_sensory_device_state()
    def _update_motoring_device_state(self):
        self._roboid._update_motoring_device_state()
    def _notify_sensory_device_data_changed(self):
        self._roboid._notify_sensory_device_data_changed()
    def _notify_motoring_device_data_changed(self):
        self._roboid._notify_motoring_device_data_changed()

    # ── 완료 감지 evaluator ───────────────────────────────────────────────────
    def _evaluate_wheel_state(self):
        return self.e(Pio.WHEEL_STATE)
    def _evaluate_neck_state(self):
        return self.e(Pio.NECK_STATE)
    def _evaluate_sound(self):
        return self.e(Pio.SOUND_STATE)

    # ── Wheel-speed snapshot helpers ──────
    # 사용자가 속도 미설정 상태로 이동을 시작 → 기본 속도로 동작 + 동작 후 미설정 상태로 복원.
    # 사용자가 속도 설정 후 이동 → 그 속도 유지. '정지' 후 다시 이동 시 직전 패턴 유지.

    def _begin_speed(self):
        if self._saved_wheel is not None:
            cur_l = self.read(Pio.LEFT_WHEEL)
            cur_r = self.read(Pio.RIGHT_WHEEL)
            if (self._wrote_wheel is not None
                    and cur_l == self._wrote_wheel[0]
                    and cur_r == self._wrote_wheel[1]):
                saved_l, saved_r = self._saved_wheel
                self.write(Pio.LEFT_WHEEL, saved_l)
                self.write(Pio.RIGHT_WHEEL, saved_r)
            self._saved_wheel = None
            self._wrote_wheel = None
        self._saved_wheel = (
            self.read(Pio.LEFT_WHEEL),
            self.read(Pio.RIGHT_WHEEL),
        )

    def _mark_speed(self):
        self._wrote_wheel = (
            self.read(Pio.LEFT_WHEEL),
            self.read(Pio.RIGHT_WHEEL),
        )

    def _restore_speed(self):
        if self._saved_wheel is not None:
            saved_l, saved_r = self._saved_wheel
            self.write(Pio.LEFT_WHEEL, saved_l)
            self.write(Pio.RIGHT_WHEEL, saved_r)
            self._saved_wheel = None
            self._wrote_wheel = None

    # ── Internal helpers ──────────────────────────────────────────────────────

    @staticmethod
    def _get_speed(value=None):
        if value is None:
            return Pio._SPEED
        return value * Pio._VAL_TO_SPEED

    @staticmethod
    def _get_distance(value, unit='cm'):
        d = value * Pio._CM_TO_PULSE
        if unit == 'mm': d /= 10
        elif unit == 'inch': d *= 2.54
        return d

    def _stop_move(self):
        bounded = self.read(Pio.WHEEL_MOVE) != 0
        self._begin_speed()
        self.write(Pio.WHEEL_MOVE, 0)
        self.write(Pio.LEFT_WHEEL, -128 if bounded else 0)
        self.write(Pio.RIGHT_WHEEL, -128 if bounded else 0)
        self._mark_speed()

    def _stop_sound(self):
        for dev in (Pio.SOUND_BUZZ, Pio.SOUND_NOTE, Pio.SOUND_CLIP, Pio.SOUND_MELODY):
            if self.read(dev) > 0:
                self.write(dev, 0)

    # ── Turn impl ─────────────────────────────────────────────────────────────
    def _turn_degree_left(self, degree):
        if self.read(Pio.WHEEL_MOVE) != 0:
            self.write(Pio.WHEEL_MOVE, 0)
        self._begin_speed()
        speed = Pio._SPEED
        self.write(Pio.LEFT_WHEEL, -speed)
        self.write(Pio.RIGHT_WHEEL, speed)
        self._mark_speed()
        self.write(Pio.WHEEL_MOVE, degree * Pio._DEG_TO_PULSE)
        Runner.wait_until(self._evaluate_wheel_state)
        self._restore_speed()

    def _turn_degree_right(self, degree):
        if self.read(Pio.WHEEL_MOVE) != 0:
            self.write(Pio.WHEEL_MOVE, 0)
        self._begin_speed()
        speed = Pio._SPEED
        self.write(Pio.LEFT_WHEEL, speed)
        self.write(Pio.RIGHT_WHEEL, -speed)
        self._mark_speed()
        self.write(Pio.WHEEL_MOVE, degree * Pio._DEG_TO_PULSE)
        Runner.wait_until(self._evaluate_wheel_state)
        self._restore_speed()

    # ── Grid (Pio: pulse-based compound moves) ────────────────────────────────
    def _move_pulses(self, left_speed, right_speed, distance_cm):
        if self.read(Pio.WHEEL_MOVE) != 0:
            self.write(Pio.WHEEL_MOVE, 0)
        self._begin_speed()
        self.write(Pio.LEFT_WHEEL, left_speed)
        self.write(Pio.RIGHT_WHEEL, right_speed)
        self._mark_speed()
        self.write(Pio.WHEEL_MOVE, self._get_distance(distance_cm, 'cm'))
        Runner.wait_until(self._evaluate_wheel_state)

    def _grid_move_forward(self):
        self._move_pulses(100, 100, 10)

    def _grid_move_backward(self):
        self._move_pulses(-100, -100, 10)

    def _grid_move_left(self):
        self._move_pulses(-100, -100, 1.45)
        Runner.wait(0.2)
        self._move_pulses(-100, 100, 3.8)
        Runner.wait(0.2)
        self._move_pulses(100, 100, 11.45)

    def _grid_move_right(self):
        self._move_pulses(-100, -100, 1.45)
        Runner.wait(0.2)
        self._move_pulses(100, -100, 3.8)
        Runner.wait(0.2)
        self._move_pulses(100, 100, 11.45)

    def _grid_turn_left(self):
        self._move_pulses(-100, -100, 1.45)
        Runner.wait(0.2)
        self._move_pulses(-100, 100, 3.8)
        Runner.wait(0.2)
        self._move_pulses(100, 100, 1.45)

    def _grid_turn_right(self):
        self._move_pulses(-100, -100, 1.45)
        Runner.wait(0.2)
        self._move_pulses(100, -100, 3.8)
        Runner.wait(0.2)
        self._move_pulses(100, 100, 1.45)

    def _grid_move_impl(self, direction):
        getattr(self, f'_grid_move_{direction}')()

    def _grid_turn_impl(self, direction):
        getattr(self, f'_grid_turn_{direction}')()

    # ── Move ──────────────────────────────────────────────────────────────────
    def set_wheel_speed(self, unit: _WheelUnit, speed: Union[int, float]):
        if unit not in Pio._VALID_WHEEL_UNITS:
            return _err(Pio, 'set_wheel_speed', 'unit', unit, Pio._VALID_WHEEL_UNITS)
        if not isinstance(speed, (int, float)):
            return _err(Pio, 'set_wheel_speed', 'speed', speed, 'int | float')
        if unit in ('both', 'left'):
            self.write(Pio.LEFT_WHEEL, self._get_speed(speed))
        if unit in ('both', 'right'):
            self.write(Pio.RIGHT_WHEEL, self._get_speed(speed))

    def _move_distance_impl(self, data, unit):
        self._begin_speed()
        cur_l = self.read(Pio.LEFT_WHEEL)
        cur_r = self.read(Pio.RIGHT_WHEEL)
        base_l = Pio._SPEED if cur_l == -128 else cur_l
        base_r = Pio._SPEED if cur_r == -128 else cur_r
        self.write(Pio.LEFT_WHEEL, base_l if data > 0 else -base_l)
        self.write(Pio.RIGHT_WHEEL, base_r if data > 0 else -base_r)
        self._mark_speed()
        self.write(Pio.WHEEL_MOVE, self._get_distance(abs(data), unit))
        Runner.wait_until(self._evaluate_wheel_state)
        self._restore_speed()

    def move_distance(self, data: Union[int, float], unit: _DistUnit = 'cm', wait: bool = True):
        if not isinstance(data, (int, float)):
            return _err(Pio, 'move_distance', 'data', data, 'int | float')
        if unit not in Pio._VALID_DIST_UNITS:
            return _err(Pio, 'move_distance', 'unit', unit, Pio._VALID_DIST_UNITS)
        Runner.dispatch(lambda: self._move_distance_impl(data, unit), wait)

    def _move_time_impl(self, seconds):
        self._begin_speed()
        if self.read(Pio.LEFT_WHEEL) == -128:
            self.write(Pio.LEFT_WHEEL, Pio._SPEED)
        if self.read(Pio.RIGHT_WHEEL) == -128:
            self.write(Pio.RIGHT_WHEEL, Pio._SPEED)
        self._mark_speed()
        Runner.wait(seconds)
        self._stop_move()

    def move_time(self, data: Union[int, float], wait: bool = True):
        if not isinstance(data, (int, float)):
            return _err(Pio, 'move_time', 'data', data, 'int | float')
        Runner.dispatch(lambda: self._move_time_impl(data), wait)

    def turn_degree(self, direction: _TurnDir, data: Union[int, float], wait: bool = True):
        if direction not in Pio._VALID_TURN_DIRS:
            return _err(Pio, 'turn_degree', 'direction', direction, Pio._VALID_TURN_DIRS)
        if not isinstance(data, (int, float)):
            return _err(Pio, 'turn_degree', 'data', data, 'int | float')
        impl = self._turn_degree_left if direction == 'left' else self._turn_degree_right
        Runner.dispatch(lambda: impl(data), wait)

    def change_wheel_speed(self, unit: _WheelUnit, speed: Union[int, float]):
        if unit not in Pio._VALID_WHEEL_UNITS:
            return _err(Pio, 'change_wheel_speed', 'unit', unit, Pio._VALID_WHEEL_UNITS)
        if not isinstance(speed, (int, float)):
            return _err(Pio, 'change_wheel_speed', 'speed', speed, 'int | float')
        if self.read(Pio.WHEEL_MOVE) != 0:
            self.write(Pio.WHEEL_MOVE, 0)
        if unit in ('both', 'left'):
            self.write(Pio.LEFT_WHEEL, self.read(Pio.LEFT_WHEEL) + self._get_speed(speed))
        if unit in ('both', 'right'):
            self.write(Pio.RIGHT_WHEEL, self.read(Pio.RIGHT_WHEEL) + self._get_speed(speed))

    def turbo(self, on: bool = True):
        if not isinstance(on, bool):
            return _err(Pio, 'turbo', 'on', on, 'bool')
        self.write(Pio.TURBO, 1 if on else 0)

    def stop(self):
        self._stop_move()

    def wheel_moving(self) -> bool:
        return self.read(Pio.WHEEL_MOVING)

    # ── Grid ──────────────────────────────────────────────────────────────────
    def grid_move(self, direction: _GridMove, wait: bool = True):
        if direction not in Pio._VALID_GRID_MOVES:
            return _err(Pio, 'grid_move', 'direction', direction, Pio._VALID_GRID_MOVES)
        Runner.dispatch(lambda: self._grid_move_impl(direction), wait)

    def grid_turn(self, direction: _GridTurn, wait: bool = True):
        if direction not in Pio._VALID_GRID_TURNS:
            return _err(Pio, 'grid_turn', 'direction', direction, Pio._VALID_GRID_TURNS)
        Runner.dispatch(lambda: self._grid_turn_impl(direction), wait)

    # ── Neck ──────────────────────────────────────────────────────────────────
    def set_neck_speed(self, data: int = 4):
        if not isinstance(data, (int, float)):
            return _err(Pio, 'set_neck_speed', 'data', data, 'int | float')
        self.write(Pio.NECK_SPEED, data)

    def _set_neck_angle_impl(self, data):
        self.write(Pio.NECK_ANGLE, data)
        Runner.wait_until(self._evaluate_neck_state)

    def set_neck_angle(self, data: Union[int, float], wait: bool = True):
        if not isinstance(data, (int, float)):
            return _err(Pio, 'set_neck_angle', 'data', data, 'int | float')
        Runner.dispatch(lambda: self._set_neck_angle_impl(data), wait)

    def neck_moving(self) -> bool:
        return self.read(Pio.NECK_MOVING)

    # ── Eye LED ───────────────────────────────────────────────────────────────
    def set_eye_color(self, unit: _EyeUnit, r: Union[_EyeColor, int, float], g: Union[int, None] = None, b: Union[int, None] = None):
        if unit not in Pio._VALID_EYE_UNITS:
            return _err(Pio, 'set_eye_color', 'unit', unit, Pio._VALID_EYE_UNITS)
        if isinstance(r, str):
            if r not in Pio._VALID_COLORS:
                return _err(Pio, 'set_eye_color', 'color', r, tuple(Pio._VALID_COLORS))
            rgb = Pio._VALID_COLORS[r]
        else:
            for name, val in (('r', r), ('g', g), ('b', b)):
                if not isinstance(val, (int, float)):
                    return _err(Pio, 'set_eye_color', name, val, 'int | float')
            rgb = [r, g, b]
        if unit in ('both', 'left'):
            self.write(Pio.LEFT_RGB, rgb)
        if unit in ('both', 'right'):
            self.write(Pio.RIGHT_RGB, rgb)

    def change_eye_color(self, unit: _EyeUnit, r: int, g: int, b: int):
        if unit not in Pio._VALID_EYE_UNITS:
            return _err(Pio, 'change_eye_color', 'unit', unit, Pio._VALID_EYE_UNITS)
        for name, val in (('r', r), ('g', g), ('b', b)):
            if not isinstance(val, (int, float)):
                return _err(Pio, 'change_eye_color', name, val, 'int | float')
        for side in ('left', 'right'):
            if unit in ('both', side):
                dev = Pio.LEFT_RGB if side == 'left' else Pio.RIGHT_RGB
                cur = [self.read(dev, 0), self.read(dev, 1), self.read(dev, 2)]
                self.write(dev, [cur[0] + r, cur[1] + g, cur[2] + b])

    def set_eye_pattern(self, pattern: _EyePattern, left: _EyeColor = 'white', right: _EyeColor = 'white'):
        if pattern not in Pio._VALID_EYE_PATTERN:
            return _err(Pio, 'set_eye_pattern', 'pattern', pattern, tuple(Pio._VALID_EYE_PATTERN))
        if left not in Pio._VALID_EYE_COLOR:
            return _err(Pio, 'set_eye_pattern', 'left', left, Pio._VALID_EYE_COLOR)
        if right not in Pio._VALID_EYE_COLOR:
            return _err(Pio, 'set_eye_pattern', 'right', right, Pio._VALID_EYE_COLOR)
        self.write(Pio.EYE_PATTERN, Pio._VALID_EYE_PATTERN[pattern])
        if pattern != 'reset':
            # Color → wire 인덱스 (EyeColor Literal 순서 기반)
            color_wire = {c: i for i, c in enumerate(get_args(_EyeColor))}
            lv = color_wire[left]
            rv = color_wire[right]
            if pattern == 'rainbow':
                if lv > 0: lv -= 1
                if rv > 0: rv -= 1
            self.write(Pio.LEFT_PATTERN_COLOR, lv)
            self.write(Pio.RIGHT_PATTERN_COLOR, rv)

    def turn_off(self, unit: _EyeUnit = 'both'):
        if unit not in Pio._VALID_EYE_UNITS:
            return _err(Pio, 'turn_off', 'unit', unit, Pio._VALID_EYE_UNITS)
        if unit in ('both', 'left'):
            self.write(Pio.LEFT_RGB, [0, 0, 0])
        if unit in ('both', 'right'):
            self.write(Pio.RIGHT_RGB, [0, 0, 0])

    # ── Sound ─────────────────────────────────────────────────────────────────
    def sound_buzz(self, hz: Union[int, float]):
        if not isinstance(hz, (int, float)):
            return _err(Pio, 'sound_buzz', 'hz', hz, 'int | float')
        self._stop_sound()
        self.write(Pio.SOUND_BUZZ, hz)

    def sound_note(self, note: _Note, octave: int = 4):
        if note not in Pio._VALID_NOTES:
            return _err(Pio, 'sound_note', 'note', note, tuple(Pio._VALID_NOTES))
        if not isinstance(octave, int) or not (1 <= octave <= 7):
            return _err(Pio, 'sound_note', 'octave', octave, 'int (1~7)')
        self._stop_sound()
        self.write(Pio.SOUND_NOTE, (octave - 1) * 12 + Pio._VALID_NOTES[note] + 4)

    def _sound_clip_impl(self, clip):
        self._stop_sound()
        self.write(Pio.SOUND_CLIP, Pio._VALID_CLIPS[clip])
        Runner.wait_until(self._evaluate_sound)

    def sound_clip(self, clip: _Clip, wait: bool = True):
        if clip not in Pio._VALID_CLIPS:
            return _err(Pio, 'sound_clip', 'clip', clip, tuple(Pio._VALID_CLIPS))
        Runner.dispatch(lambda: self._sound_clip_impl(clip), wait)

    def _sound_melody_impl(self, melody):
        self._stop_sound()
        self.write(Pio.SOUND_MELODY, Pio._VALID_MELODYS[melody])
        Runner.wait_until(self._evaluate_sound)

    def sound_melody(self, melody: _Melody, wait: bool = True):
        if melody not in Pio._VALID_MELODYS:
            return _err(Pio, 'sound_melody', 'melody', melody, tuple(Pio._VALID_MELODYS))
        Runner.dispatch(lambda: self._sound_melody_impl(melody), wait)

    def sound_off(self):
        self._stop_sound()

    def sound_playing(self) -> bool:
        return self.read(Pio.SOUND_PLAYING)

    # ── Sensors ───────────────────────────────────────────────────────────────
    def wheel_speed(self, unit: _SensorSide) -> Union[int, float]:
        if unit not in Pio._VALID_SENSOR_SIDES:
            return _err(Pio, 'wheel_speed', 'unit', unit, Pio._VALID_SENSOR_SIDES)
        value = self.read(Pio.LEFT_WHEEL if unit == 'left' else Pio.RIGHT_WHEEL)
        return 0 if value == -128 else value

    def signal_strength(self) -> Union[int, float]:
        return self.read(Pio.SIGNAL_STRENGTH)

    def battery(self) -> Union[int, float]:
        return self.read(Pio.BATTERY)

    def keypad(self, button: _KeypadButton) -> bool:
        if button not in Pio._VALID_KEYPAD:
            return _err(Pio, 'keypad', 'button', button, tuple(Pio._VALID_KEYPAD))
        return self.e(Pio.KEYPAD_STATE) and self.read(Pio.KEYPAD) == Pio._VALID_KEYPAD[button]
