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
_WheelUnit       = Literal['left', 'right', 'both']
_DistUnit        = Literal['cm', 'mm', 'inch']
_TurnDir         = Literal['left', 'right']
_PivotBase       = Literal['left_pen', 'right_pen', 'left_wheel', 'right_wheel']
_PivotDir        = Literal['forward', 'backward']
_CircleDir       = Literal['left_forward', 'left_backward', 'right_forward', 'right_backward']
_TraceLine       = Literal['black', 'red', 'green', 'blue', 'any']
_UntilLine       = Literal['black', 'red', 'green', 'blue', 'any']
_UntilColor      = Literal['black', 'red', 'yellow', 'green', 'cyan', 'blue', 'magenta', 'any']
_IntersectionDir = Literal['left', 'right', 'forward', 'uturn']
_Note            = Literal['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B']
_Clip            = Literal['mute', 'beep', 'beep2', 'beep3', 'beep_repeat', 'beep_random',
                            'beep2_random', 'beep3_random', 'beep_random_repeat',
                            'siren', 'siren_repeat', 'engine', 'engine_repeat', 'noise_random',
                            'robot', 'march', 'birthday', 'dibidibidip', 'good_job']
_SensorSide      = Literal['left', 'right']
_Axis            = Literal['x', 'y', 'z']
_CardColor       = Literal['unknown', 'red', 'yellow', 'green', 'cyan', 'blue', 'magenta', 'white', 'any']
_LEDColor        = Literal['black', 'red', 'yellow', 'green', 'cyan', 'blue', 'magenta', 'white', 'any']
_CardPattern     = Literal['red_yellow', 'red_green', 'red_cyan', 'red_blue', 'red_magenta',
                            'yellow_red', 'yellow_green', 'yellow_cyan', 'yellow_blue', 'yellow_magenta',
                            'green_red', 'green_yellow', 'green_cyan', 'green_blue',
                            'cyan_red', 'cyan_yellow', 'cyan_green', 'cyan_blue', 'cyan_magenta',
                            'blue_red', 'blue_yellow', 'blue_green', 'blue_cyan', 'blue_magenta',
                            'magenta_red', 'magenta_yellow', 'magenta_green', 'magenta_cyan', 'magenta_blue']
_ButtonEvent     = Literal['pressed', 'click', 'long_click']


class Turtle(Robot):
    ID = "kr.robomation.physical.robot.turtle"
    _robots = {}

    # ── Device IDs (Turtle 펌웨어 디바이스 식별자) ──────────
    # Effectors (0x009000xx)
    LEFT_WHEEL         = 0x00900000
    RIGHT_WHEEL        = 0x00900001
    LINE_TRACER_SPEED  = 0x00900002    
    LINE_TRACER_GAIN   = 0x00900003    
    HEAD_LED           = 0x00900004    
    SOUND_BUZZ         = 0x00900005

    # Commands (0x009001xx)
    WHEEL_MOVE         = 0x00900100
    LINE_TRACER_MODE   = 0x00900101    
    SOUND_NOTE         = 0x00900102
    SOUND_CLIP         = 0x00900103

    # Sensors (0x009002xx)
    WHEEL_COUNT        = 0x00900200
    SOUND_PLAYING      = 0x00900201
    FLOOR              = 0x00900202
    CARD_COLOR         = 0x00900203    
    CARD_PATTERN       = 0x00900204
    BUTTON_PRESSED     = 0x00900205    
    ACCELERATION_X     = 0x00900206
    ACCELERATION_Y     = 0x00900207
    ACCELERATION_Z     = 0x00900208
    TEMPERATURE        = 0x00900209
    SIGNAL_STRENGTH    = 0x0090020a
    BATTERY            = 0x0090020b
    
    # Events (0x009003xx)
    WHEEL_STATE        = 0x00900300
    LINE_TRACER_STATE  = 0x00900301
    SOUND_STATE        = 0x00900302
    BUTTON_CLICK       = 0x00900303
    BUTTON_LONG_CLICK      = 0x00900304
    BUTTON_LONG_LONG_CLICK = 0x00900305

    # ── Robot-specific constants ──────────────────────────────────────────────
    _SPEED = 50
    _VAL_TO_SPEED = 1
    _CM_TO_PULSE = 106.8
    _PARAM_TURN_LEFT = -15
    _PARAM_TURN_RIGHT = -7
    _PARAM_PIVOT_LEFT = -13
    _PARAM_PIVOT_RIGHT = -6
    _PARAM_INNER_VELOCITY = 2.3126

    # ── Valid values (derived from module-level Literals) ────────────────────
    _VALID_WHEEL_UNITS          = get_args(_WheelUnit)
    _VALID_DIST_UNITS           = get_args(_DistUnit)
    _VALID_TURN_DIRS            = get_args(_TurnDir)
    _VALID_PIVOT_BASES          = get_args(_PivotBase)
    _VALID_PIVOT_DIRS           = get_args(_PivotDir)
    _VALID_CIRCLE_DIRS          = get_args(_CircleDir)
    _VALID_TRACE_LINE           = get_args(_TraceLine)
    _VALID_UNTIL_LINE           = get_args(_UntilLine)
    _VALID_UNTIL_COLOR          = get_args(_UntilColor)
    _VALID_INTERSECTION_DIRS    = get_args(_IntersectionDir)
    _VALID_NOTES                = {n: i for i, n in enumerate(get_args(_Note))}
    _VALID_CLIPS                = {
        'mute': 0,                
        'beep': 1,                'beep2': 2,                'beep3': 3,                'beep_repeat': 4,         
        'beep_random': 5,         'beep2_random': 6,         'beep3_random': 7,         'beep_random_repeat': 8,  
        'siren': 16,              'siren_repeat': 17,
        'engine': 32,             'engine_repeat': 33,       'noise_random': 34,
        'robot': 48,              
        'march': 64,              'birthday': 65,            'dibidibidip': 66,         'good_job': 67,                   
    }
    _VALID_SENSOR_SIDES         = get_args(_SensorSide)
    _VALID_AXIS                 = get_args(_Axis)
    _VALID_COLORS               = {
        'black':   [  0,   0,   0],
        'red':     [255,   0,   0],
        'yellow':  [255, 255,   0],
        'green':   [  0, 255,   0],
        'cyan':    [  0, 255, 255],
        'blue':    [  0,   0, 255],
        'magenta': [255,   0, 255],
        'white':   [255, 255, 255],
    }
    _VALID_CARD_COLOR           = {c: i for i, c in enumerate(get_args(_CardColor))}
    _VALID_CARD_PATTERN         = {
        'red_yellow': 12,   'red_green': 13,        'red_cyan': 14,         'red_blue': 15,     'red_magenta': 16,
        'yellow_red': 21,   'yellow_green': 23,     'yellow_cyan': 24,      'yellow_blue': 25,  'yellow_magenta': 26,
        'green_red': 31,    'green_yellow': 32,     'green_cyan': 34,       'green_blue': 35,   'green_magenta': 36,
        'cyan_red': 41,     'cyan_yellow': 42,      'cyan_green': 43,       'cyan_blue': 45,    'cyan_magenta': 46,
        'blue_red': 51,     'blue_yellow': 52,      'blue_green': 53,       'blue_cyan': 54,    'blue_magenta': 56,
        'magenta_red': 61,  'magenta_yellow': 62,   'magenta_green': 63,    'magenta_cyan': 64, 'magenta_blue': 65,
    }
    _VALID_BUTTON_EVENTS        = get_args(_ButtonEvent)

    # Trace mode wire 값
    _TRACE_LINE_WIRE = {
        'black': 8,     'red': 9,       'green': 11,    'blue': 13,     'any': 15
    }
    _TRACE_INTERSECTION_WIRE = {
        'left': 16,     'right': 24,    'forward': 32,  'uturn': 40
    }
    _TRACE_LINE_COLOR_WIRE = {
        ('black', 'red'): 49,   ('black', 'yellow'): 50,    ('black', 'green'): 51,
        ('black', 'cyan'): 52,  ('black', 'blue'): 53,      ('black', 'magenta'): 54,   ('black', 'any'): 55,
        ('red', 'black'): 57,   ('green', 'black'): 59,     ('blue', 'black'): 61,      ('any', 'black'): 63,
    }

    # ── Robot lifecycle ───────────────────────────────────────────────────────
    def __init__(self, index=0, port_name=None):
        if isinstance(index, str):
            port_name = index
            index = 0
        if index in Turtle._robots:
            robot = Turtle._robots[index]
            if robot: robot.dispose()
        Turtle._robots[index] = self
        super(Turtle, self).__init__(Turtle.ID, "Turtle", index)

        # Wheel-speed snapshot 상태 
        self._saved_wheel = None
        self._wrote_wheel = None

        self._init(port_name)

    def dispose(self):
        Turtle._robots[self.get_index()] = None
        self._roboid._dispose()
        Runner.unregister_robot(self)

    def reset(self):
        self._roboid._reset()

    def _init(self, port_name):
        from robomation.roboids.turtle_roboid import TurtleRoboid
        self._roboid = TurtleRoboid(self.get_index())
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
        return self.e(Turtle.WHEEL_STATE)
    def _evaluate_line_tracer(self):
        return self.e(Turtle.LINE_TRACER_STATE)
    def _evaluate_sound(self):
        return self.e(Turtle.SOUND_STATE)

    # ── Wheel-speed snapshot helpers ──────
    # 사용자가 속도 미설정 상태로 이동을 시작 → 기본 속도로 동작 + 동작 후 미설정 상태로 복원.
    # 사용자가 속도 설정 후 이동 → 그 속도 유지. '정지' 후 다시 이동 시 직전 패턴 유지.

    def _begin_speed(self):
        if self._saved_wheel is not None:
            cur_l = self.read(Turtle.LEFT_WHEEL)
            cur_r = self.read(Turtle.RIGHT_WHEEL)
            if (self._wrote_wheel is not None
                    and cur_l == self._wrote_wheel[0]
                    and cur_r == self._wrote_wheel[1]):
                saved_l, saved_r = self._saved_wheel
                self.write(Turtle.LEFT_WHEEL, saved_l)
                self.write(Turtle.RIGHT_WHEEL, saved_r)
            self._saved_wheel = None
            self._wrote_wheel = None
        self._saved_wheel = (
            self.read(Turtle.LEFT_WHEEL),
            self.read(Turtle.RIGHT_WHEEL),
        )

    def _mark_speed(self):
        self._wrote_wheel = (
            self.read(Turtle.LEFT_WHEEL),
            self.read(Turtle.RIGHT_WHEEL),
        )

    def _restore_speed(self):
        if self._saved_wheel is not None:
            saved_l, saved_r = self._saved_wheel
            self.write(Turtle.LEFT_WHEEL, saved_l)
            self.write(Turtle.RIGHT_WHEEL, saved_r)
            self._saved_wheel = None
            self._wrote_wheel = None

    # ── Internal helpers ──────────────────────────────────────────────────────
    
    @staticmethod
    def _get_speed(value=None):
        if value is None:
            return Turtle._SPEED
        return value * Turtle._VAL_TO_SPEED

    @staticmethod
    def _get_distance(value, unit='cm'):
        d = value * Turtle._CM_TO_PULSE
        if unit == 'mm': d /= 10
        elif unit == 'inch': d *= 2.54
        return d

    def _stop_move(self):
        bounded = self.read(Turtle.WHEEL_MOVE) != 0
        self._begin_speed()
        self.write(Turtle.WHEEL_MOVE, 0)
        self.write(Turtle.LEFT_WHEEL, -128 if bounded else 0)
        self.write(Turtle.RIGHT_WHEEL, -128 if bounded else 0)
        self._mark_speed()

    def _stop_sound(self):
        for dev in (Turtle.SOUND_BUZZ, Turtle.SOUND_NOTE, Turtle.SOUND_CLIP):
            if self.read(dev) > 0:
                self.write(dev, 0)

    # ── Geometry helpers ─────────────────────────────────────────────────────

    @staticmethod
    def _calc_deg_to_pulse(degree, side):
        param = Turtle._PARAM_TURN_LEFT if side == 'left' else Turtle._PARAM_TURN_RIGHT
        return degree * (1570 + param) / 360

    @staticmethod
    def _calc_pivot_to_pulse(degree, side):
        param = Turtle._PARAM_PIVOT_LEFT if side == 'left' else Turtle._PARAM_PIVOT_RIGHT
        return degree * (3140 + param) / 360

    @staticmethod
    def _calc_inner_velocity(speed, radius):
        p = Turtle._PARAM_INNER_VELOCITY
        return speed * (radius - p) / (radius + p)

    @staticmethod
    def _calc_circle_to_pulse(degree, radius, side):
        param = Turtle._PARAM_TURN_LEFT if side == 'left' else Turtle._PARAM_TURN_RIGHT
        p = Turtle._PARAM_INNER_VELOCITY
        return degree * (1570 + param) * (radius + p) / 360 / p

    # ── Turn / Pivot impl ─────────────────────────────────────────────────────
    def _turn_degree_left(self, degree):
        if self.read(Turtle.WHEEL_MOVE) != 0:
            self.write(Turtle.WHEEL_MOVE, 0)
        self._begin_speed()
        speed = Turtle._SPEED
        self.write(Turtle.LEFT_WHEEL, -speed)
        self.write(Turtle.RIGHT_WHEEL, speed)
        self._mark_speed()
        self.write(Turtle.WHEEL_MOVE, self._calc_deg_to_pulse(degree, 'left'))
        Runner.wait_until(self._evaluate_wheel_state)
        self._restore_speed()

    def _turn_degree_right(self, degree):
        if self.read(Turtle.WHEEL_MOVE) != 0:
            self.write(Turtle.WHEEL_MOVE, 0)
        self._begin_speed()
        speed = Turtle._SPEED
        self.write(Turtle.LEFT_WHEEL, speed)
        self.write(Turtle.RIGHT_WHEEL, -speed)
        self._mark_speed()
        self.write(Turtle.WHEEL_MOVE, self._calc_deg_to_pulse(degree, 'right'))
        Runner.wait_until(self._evaluate_wheel_state)
        self._restore_speed()

    def _pivot(self, base, direction, degree):
        if self.read(Turtle.WHEEL_MOVE) != 0:
            self.write(Turtle.WHEEL_MOVE, 0)
        side = base.split('_')[0]
        self._begin_speed()
        speed = Turtle._SPEED
        signed = speed if direction == 'forward' else -speed
        if side == 'left':
            self.write(Turtle.LEFT_WHEEL, 0)
            self.write(Turtle.RIGHT_WHEEL, signed)
        else:
            self.write(Turtle.LEFT_WHEEL, signed)
            self.write(Turtle.RIGHT_WHEEL, 0)
        self._mark_speed()
        self.write(Turtle.WHEEL_MOVE, self._calc_pivot_to_pulse(degree, side))
        Runner.wait_until(self._evaluate_wheel_state)
        self._restore_speed()

    def _pivot_circle(self, direction, radius_pulse, degree):
        if self.read(Turtle.WHEEL_MOVE) != 0:
            self.write(Turtle.WHEEL_MOVE, 0)
        parts = direction.split('_')
        side, fwd = parts[0], parts[1]
        self._begin_speed()
        speed = Turtle._SPEED if fwd == 'forward' else -Turtle._SPEED
        radius = radius_pulse / Turtle._CM_TO_PULSE
        inner = self._calc_inner_velocity(speed, radius)
        if side == 'left':
            self.write(Turtle.LEFT_WHEEL, inner)
            self.write(Turtle.RIGHT_WHEEL, speed)
        else:
            self.write(Turtle.LEFT_WHEEL, speed)
            self.write(Turtle.RIGHT_WHEEL, inner)
        self._mark_speed()
        self.write(Turtle.WHEEL_MOVE, self._calc_circle_to_pulse(degree, radius, side))
        Runner.wait_until(self._evaluate_wheel_state)
        self._restore_speed()

    # ── Move ──────────────────────────────────────────────────────────────────
    def set_wheel_speed(self, unit: _WheelUnit, speed: Union[int, float]):
        if unit not in Turtle._VALID_WHEEL_UNITS:
            return _err(Turtle, 'set_wheel_speed', 'unit', unit, Turtle._VALID_WHEEL_UNITS)
        if not isinstance(speed, (int, float)):
            return _err(Turtle, 'set_wheel_speed', 'speed', speed, 'int | float')
        if self.read(Turtle.WHEEL_MOVE) != 0:
            self.write(Turtle.WHEEL_MOVE, 0)
        if unit in ('both', 'left'):
            self.write(Turtle.LEFT_WHEEL, self._get_speed(speed))
        if unit in ('both', 'right'):
            self.write(Turtle.RIGHT_WHEEL, self._get_speed(speed))

    def _move_distance_impl(self, data, unit):
        self._begin_speed()
        cur_l = self.read(Turtle.LEFT_WHEEL)
        cur_r = self.read(Turtle.RIGHT_WHEEL)
        base_l = Turtle._SPEED if cur_l == -128 else cur_l
        base_r = Turtle._SPEED if cur_r == -128 else cur_r
        self.write(Turtle.LEFT_WHEEL, base_l if data > 0 else -base_l)
        self.write(Turtle.RIGHT_WHEEL, base_r if data > 0 else -base_r)
        self._mark_speed()
        self.write(Turtle.WHEEL_MOVE, self._get_distance(abs(data), unit))
        Runner.wait_until(self._evaluate_wheel_state)
        self._restore_speed()

    def move_distance(self, data: Union[int, float], unit: _DistUnit = 'cm', wait: bool = True):
        if not isinstance(data, (int, float)):
            return _err(Turtle, 'move_distance', 'data', data, 'int | float')
        if unit not in Turtle._VALID_DIST_UNITS:
            return _err(Turtle, 'move_distance', 'unit', unit, Turtle._VALID_DIST_UNITS)
        Runner.dispatch(lambda: self._move_distance_impl(data, unit), wait)

    def _move_time_impl(self, seconds):
        self._begin_speed()
        if self.read(Turtle.LEFT_WHEEL) == -128:
            self.write(Turtle.LEFT_WHEEL, Turtle._SPEED)
        if self.read(Turtle.RIGHT_WHEEL) == -128:
            self.write(Turtle.RIGHT_WHEEL, Turtle._SPEED)
        self._mark_speed()
        Runner.wait(seconds)
        self._stop_move()
        Runner.wait(0.25)

    def move_time(self, data: Union[int, float], wait: bool = True):
        if not isinstance(data, (int, float)):
            return _err(Turtle, 'move_time', 'data', data, 'int | float')
        Runner.dispatch(lambda: self._move_time_impl(data), wait)

    def turn_degree(self, direction: _TurnDir, data: Union[int, float], wait: bool = True):
        if direction not in Turtle._VALID_TURN_DIRS:
            return _err(Turtle, 'turn_degree', 'direction', direction, Turtle._VALID_TURN_DIRS)
        if not isinstance(data, (int, float)):
            return _err(Turtle, 'turn_degree', 'data', data, 'int | float')
        impl = self._turn_degree_left if direction == 'left' else self._turn_degree_right
        Runner.dispatch(lambda: impl(data), wait)

    def change_wheel_speed(self, unit: _WheelUnit, speed: Union[int, float]):
        if unit not in Turtle._VALID_WHEEL_UNITS:
            return _err(Turtle, 'change_wheel_speed', 'unit', unit, Turtle._VALID_WHEEL_UNITS)
        if not isinstance(speed, (int, float)):
            return _err(Turtle, 'change_wheel_speed', 'speed', speed, 'int | float')
        if self.read(Turtle.WHEEL_MOVE) != 0:
            self.write(Turtle.WHEEL_MOVE, 0)
        if unit in ('both', 'left'):
            self.write(Turtle.LEFT_WHEEL, self.read(Turtle.LEFT_WHEEL) + self._get_speed(speed))
        if unit in ('both', 'right'):
            self.write(Turtle.RIGHT_WHEEL, self.read(Turtle.RIGHT_WHEEL) + self._get_speed(speed))

    def stop(self):
        self._stop_move()

    def wheel_moving(self) -> bool:
        return self.read(Turtle.LEFT_WHEEL) != 0 or self.read(Turtle.RIGHT_WHEEL) != 0

    # ── Pivot ─────────────────────────────────────────────────────────────────
    def pivot(self, base: _PivotBase, direction: _PivotDir, degree: Union[int, float], wait: bool = True):
        if base not in Turtle._VALID_PIVOT_BASES:
            return _err(Turtle, 'pivot', 'base', base, Turtle._VALID_PIVOT_BASES)
        if direction not in Turtle._VALID_PIVOT_DIRS:
            return _err(Turtle, 'pivot', 'direction', direction, Turtle._VALID_PIVOT_DIRS)
        if not isinstance(degree, (int, float)):
            return _err(Turtle, 'pivot', 'degree', degree, 'int | float')
        Runner.dispatch(lambda: self._pivot(base, direction, degree), wait)

    def pivot_circle(self, direction: _CircleDir, degree: Union[int, float], radius: Union[int, float], unit: _DistUnit = 'cm', wait: bool = True):
        if direction not in Turtle._VALID_CIRCLE_DIRS:
            return _err(Turtle, 'pivot_circle', 'direction', direction, Turtle._VALID_CIRCLE_DIRS)
        if not isinstance(degree, (int, float)):
            return _err(Turtle, 'pivot_circle', 'degree', degree, 'int | float')
        if not isinstance(radius, (int, float)):
            return _err(Turtle, 'pivot_circle', 'radius', radius, 'int | float')
        if unit not in Turtle._VALID_DIST_UNITS:
            return _err(Turtle, 'pivot_circle', 'unit', unit, Turtle._VALID_DIST_UNITS)
        Runner.dispatch(lambda: self._pivot_circle(direction, self._get_distance(radius, unit), degree), wait)

    # ── Trace ─────────────────────────────────────────────────────────────────
    def trace_line(self, line: _TraceLine = 'black'):
        if line not in Turtle._VALID_TRACE_LINE:
            return _err(Turtle, 'trace_line', 'line', line, Turtle._VALID_TRACE_LINE)
        if self.read(Turtle.WHEEL_MOVE) != 0:
            self._stop_move()
        self.write(Turtle.LINE_TRACER_MODE, Turtle._TRACE_LINE_WIRE[line])

    def _trace_until_impl(self, wire):
        if self.read(Turtle.WHEEL_MOVE) != 0:
            self._stop_move()
        self.write(Turtle.LINE_TRACER_MODE, wire)
        Runner.wait_until(self._evaluate_line_tracer)

    def trace_line_until_color(self, line: _UntilLine, color: _UntilColor, wait: bool = True):
        if line not in Turtle._VALID_UNTIL_LINE:
            return _err(Turtle, 'trace_line_until_color', 'line', line, Turtle._VALID_UNTIL_LINE)
        if color not in Turtle._VALID_UNTIL_COLOR:
            return _err(Turtle, 'trace_line_until_color', 'color', color, Turtle._VALID_UNTIL_COLOR)
        wire = Turtle._TRACE_LINE_COLOR_WIRE.get((line, color))
        if wire is None:
            return _err(Turtle, 'trace_line_until_color', '(line, color)', (line, color), '사용할 수 없는 조합')
        Runner.dispatch(lambda: self._trace_until_impl(wire), wait)

    def _trace_intersection_impl(self, wire):
        if self.read(Turtle.WHEEL_MOVE) != 0:
            self._stop_move()
        self.write(Turtle.LINE_TRACER_MODE, wire)
        Runner.wait_until(self._evaluate_line_tracer)

    def trace_intersection(self, direction: _IntersectionDir, wait: bool = True):
        if direction not in Turtle._VALID_INTERSECTION_DIRS:
            return _err(Turtle, 'trace_intersection', 'direction', direction, Turtle._VALID_INTERSECTION_DIRS)
        wire = Turtle._TRACE_INTERSECTION_WIRE[direction]
        Runner.dispatch(lambda: self._trace_intersection_impl(wire), wait)

    def set_trace_speed(self, data: Union[int, float]):
        if not isinstance(data, (int, float)):
            return _err(Turtle, 'set_trace_speed', 'data', data, 'int | float')
        self.write(Turtle.LINE_TRACER_SPEED, data)

    def set_trace_gain(self, data: Union[int, float]):
        if not isinstance(data, (int, float)):
            return _err(Turtle, 'set_trace_gain', 'data', data, 'int | float')
        self.write(Turtle.LINE_TRACER_GAIN, data)

    def stop_trace(self):
        self.write(Turtle.LINE_TRACER_MODE, 0)

    # ── LED ─────────────────────────────────────────────────────────────────
    def set_led_color(self, r: Union[_LEDColor, int, float], g: Union[int, None] = None, b: Union[int, None] = None):
        if isinstance(r, str):
            if r not in Turtle._VALID_COLORS:
                return _err(Turtle, 'set_led_color', 'color', r, tuple(Turtle._VALID_COLORS))
            rgb = Turtle._VALID_COLORS[r]
        else:
            for name, val in (('r', r), ('g', g), ('b', b)):
                if not isinstance(val, (int, float)):
                    return _err(Turtle, 'set_led_color', name, val, 'int | float')
            rgb = [r, g, b]
        self.write(Turtle.HEAD_LED, rgb)

    def change_led_color(self, r: int, g: int, b: int):
        for name, val in (('r', r), ('g', g), ('b', b)):
            if not isinstance(val, (int, float)):
                return _err(Turtle, 'change_led_color', name, val, 'int | float')
        cur = [self.read(Turtle.HEAD_LED, 0), self.read(Turtle.HEAD_LED, 1), self.read(Turtle.HEAD_LED, 2)]
        new_rgb = [cur[0] + r, cur[1] + g, cur[2] + b]
        self.write(Turtle.HEAD_LED, new_rgb)

    def turn_off(self):
        self.write(Turtle.HEAD_LED, [0, 0, 0])

    # ── Sound ─────────────────────────────────────────────────────────────────
    def sound_buzz(self, hz: Union[int, float]):
        if not isinstance(hz, (int, float)):
            return _err(Turtle, 'sound_buzz', 'hz', hz, 'int | float')
        self.write(Turtle.SOUND_BUZZ, hz)

    def sound_note(self, note: _Note, octave: int = 4):
        if note not in Turtle._VALID_NOTES:
            return _err(Turtle, 'sound_note', 'note', note, tuple(Turtle._VALID_NOTES))
        if not isinstance(octave, int) or not (1 <= octave <= 7):
            return _err(Turtle, 'sound_note', 'octave', octave, 'int (1~7)')
        self.write(Turtle.SOUND_NOTE, (octave - 1) * 12 + Turtle._VALID_NOTES[note] + 4)

    def _sound_clip_impl(self, clip):
        self.write(Turtle.SOUND_CLIP, Turtle._VALID_CLIPS[clip])
        Runner.wait_until(self._evaluate_sound)

    def sound_clip(self, clip: _Clip, wait: bool = True):
        if clip not in Turtle._VALID_CLIPS:
            return _err(Turtle, 'sound_clip', 'clip', clip, tuple(Turtle._VALID_CLIPS))
        Runner.dispatch(lambda: self._sound_clip_impl(clip), wait)

    def sound_off(self):
        self._stop_sound()

    def sound_playing(self) -> bool:
        return self.read(Turtle.SOUND_PLAYING)

    # ── Sensors ───────────────────────────────────────────────────────────────
    def wheel_speed(self, unit: _SensorSide) -> Union[int, float]:
        if unit not in Turtle._VALID_SENSOR_SIDES:
            return _err(Turtle, 'wheel_speed', 'unit', unit, Turtle._VALID_SENSOR_SIDES)
        value = self.read(Turtle.LEFT_WHEEL if unit == 'left' else Turtle.RIGHT_WHEEL)
        return 0 if value == -128 else value

    def floor(self) -> Union[int, float]:
        return self.read(Turtle.FLOOR)

    def card_color(self) -> str:
        val = self.read(Turtle.CARD_COLOR)
        for name, wire in Turtle._VALID_CARD_COLOR.items():
            if val == wire:
                return name
        return 'unknown'

    def card_pattern(self) -> str:
        val = self.read(Turtle.CARD_PATTERN)
        for name, wire in Turtle._VALID_CARD_PATTERN.items():
            if val == wire:
                return name
        return 'unknown'

    def is_card_color(self, color: _CardColor) -> bool:
        if color not in Turtle._VALID_CARD_COLOR:
            return _err(Turtle, 'is_card_color', 'color', color, tuple(Turtle._VALID_CARD_COLOR))
        return self.card_color() == color

    def is_card_pattern(self, pattern: _CardPattern) -> bool:
        if pattern not in Turtle._VALID_CARD_PATTERN:
            return _err(Turtle, 'is_card_pattern', 'pattern', pattern, tuple(Turtle._VALID_CARD_PATTERN))
        return self.card_pattern() == pattern

    def acceleration(self, unit: _Axis) -> Union[int, float]:
        if unit not in Turtle._VALID_AXIS:
            return _err(Turtle, 'acceleration', 'unit', unit, Turtle._VALID_AXIS)
        return self.read({'x': Turtle.ACCELERATION_X, 'y': Turtle.ACCELERATION_Y, 'z': Turtle.ACCELERATION_Z}[unit])

    def temperature(self) -> Union[int, float]:
        return self.read(Turtle.TEMPERATURE)

    def signal_strength(self) -> Union[int, float]:
        return self.read(Turtle.SIGNAL_STRENGTH)

    def battery(self) -> Union[int, float]:
        return self.read(Turtle.BATTERY)

    def button(self, event: _ButtonEvent) -> bool:
        if event not in Turtle._VALID_BUTTON_EVENTS:
            return _err(Turtle, 'button', 'event', event, Turtle._VALID_BUTTON_EVENTS)
        if event == 'pressed':
            return bool(self.read(Turtle.BUTTON_PRESSED))
        elif event == 'click':
            return self.e(Turtle.BUTTON_CLICK)
        elif event == 'long_click':
            return self.e(Turtle.BUTTON_LONG_CLICK)
        # elif event == 'long_long_click':
        #     return self.e(Turtle.BUTTON_LONG_LONG_CLICK)
