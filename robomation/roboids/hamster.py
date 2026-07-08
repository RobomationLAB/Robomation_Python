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
_GridDir         = Literal['left', 'right']
_Floor           = Literal['left', 'right', 'center']
_TraceLine       = Literal['black', 'white']
_IntersectionDir = Literal['left', 'right', 'forward', 'uturn']
_LedUnit         = Literal['left', 'right', 'both']
_LedColor        = Literal['black', 'blue', 'green', 'cyan', 'red', 'magenta', 'yellow', 'white']
_Note            = Literal['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B']
_SensorSide      = Literal['left', 'right']
_Axis            = Literal['x', 'y', 'z']
_IoUnit          = Literal['a', 'b', 'both']
_IoInUnit        = Literal['a', 'b']
_IoMode          = Literal['analog_input', 'digital_input', 'servo_output', 'pwm_output', 'digital_output']


class Hamster(Robot):
    ID = "kr.robomation.physical.robot.hamster"
    _robots = {}

    # ── Device IDs (Hamster 펌웨어 디바이스 식별자) ──────────
    # Effectors (0x004000xx)
    LEFT_WHEEL         = 0x00400000
    RIGHT_WHEEL        = 0x00400001
    LINE_TRACER_SPEED  = 0x00400002
    LEFT_LED           = 0x00400003
    RIGHT_LED          = 0x00400004
    SOUND_BUZZ         = 0x00400005
    SOUND_NOTE         = 0x00400006

    IO_A_MODE          = 0x00400007
    IO_A_OUTPUT        = 0x00400008
    IO_B_MODE          = 0x00400009
    IO_B_OUTPUT        = 0x0040000a
    GRIPPER            = 0x0040000b
    SHOOTER            = 0x0040000c

    CONFIG_IR_CURRENT  = 0x0040000d
    CONFIG_G_RANGE     = 0x0040000e
    CONFIG_G_BAND      = 0x0040000f

    # Commands (0x004001xx)
    LINE_TRACER_MODE   = 0x00400100

    # Sensors (0x004002xx)
    IO_A_INPUT         = 0x00400200
    IO_B_INPUT         = 0x00400201
    LEFT_PROXIMITY     = 0x00400202
    RIGHT_PROXIMITY    = 0x00400203
    LIGHT              = 0x00400204
    LEFT_FLOOR         = 0x00400205
    RIGHT_FLOOR        = 0x00400206
    ACCELERATION_X     = 0x00400207
    ACCELERATION_Y     = 0x00400208
    ACCELERATION_Z     = 0x00400209
    BATTERY            = 0x0040020a
    TEMPERATURE        = 0x0040020b
    SIGNAL_STRENGTH    = 0x0040020c

    # Events (0x004003xx)
    LINE_TRACER_STATE  = 0x00400300

    # ── Robot-specific constants ──────────────────────────────────────────────
    _SPEED = 30
    _VAL_TO_SPEED = 1

    # ── Valid values (derived from module-level Literals) ────────────────────
    _VALID_WHEEL_UNITS          = get_args(_WheelUnit)
    _VALID_GRID_DIRS            = get_args(_GridDir)
    _VALID_FLOOR                = get_args(_Floor)
    _VALID_TRACE_LINE           = get_args(_TraceLine)
    _VALID_INTERSECTION_DIRS    = get_args(_IntersectionDir)
    _VALID_LED_UNITS            = get_args(_LedUnit)
    _VALID_LED_COLORS           = {c: i for i, c in enumerate(get_args(_LedColor))}
    _VALID_NOTES                = {n: i for i, n in enumerate(get_args(_Note))}
    _VALID_SENSOR_SIDES         = get_args(_SensorSide)
    _VALID_AXIS                 = get_args(_Axis)
    _VALID_IO_UNITS             = get_args(_IoUnit)
    _VALID_IO_IN_UNITS          = get_args(_IoInUnit)
    _VALID_IO_MODES             = {
        'analog_input':           0,
        'digital_input':          1,
        'servo_output':           8,
        'pwm_output':             9,
        'digital_output':         10,
    }

    # (line, floor) → LINE_TRACER_MODE wire 값 — trace_line() 용
    _TRACE_LINE_WIRE = {
        ('black', 'left'):   1,   ('black', 'right'):  2,    ('black', 'center'): 3,
        ('white', 'left'):   9,   ('white', 'right'):  10,   ('white', 'center'): 11,
    }
    # (line, direction) → LINE_TRACER_MODE wire 값 — intersection() 용
    _INTERSECTION_WIRE = {
        ('black', 'left'):    4,   ('black', 'right'):   5,
        ('black', 'forward'): 6,   ('black', 'uturn'):   7,
        ('white', 'left'):    12,   ('white', 'right'):  13,
        ('white', 'forward'): 14,   ('white', 'uturn'):  15,
    }

    # ── Robot lifecycle ───────────────────────────────────────────────────────
    def __init__(self, index=0, port_name=None):
        if isinstance(index, str):
            port_name = index
            index = 0
        if index in Hamster._robots:
            robot = Hamster._robots[index]
            if robot: robot.dispose()
        Hamster._robots[index] = self
        super(Hamster, self).__init__(Hamster.ID, "Hamster", index)

        # Wheel-speed snapshot 상태
        self._saved_wheel = None
        self._wrote_wheel = None

        self._init(port_name)

    def dispose(self):
        Hamster._robots[self.get_index()] = None
        self._roboid._dispose()
        Runner.unregister_robot(self)

    def reset(self):
        self._roboid._reset()

    def _init(self, port_name):
        from robomation.roboids.hamster_roboid import HamsterRoboid
        self._roboid = HamsterRoboid(self.get_index())
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

    # ── 완료 감지 evaluator (Runner.wait_until 용) ────────────────────────────
    def _evaluate_line_tracer(self):
        return self.e(Hamster.LINE_TRACER_STATE)

    # ── Wheel-speed snapshot helpers ──────────────────────────────────────────
    # 사용자가 속도 미설정 상태로 이동을 시작 → 기본 속도로 동작 + 동작 후 미설정 상태로 복원.
    # 사용자가 속도 설정 후 이동 → 그 속도 유지. '정지' 후 다시 이동 시 직전 패턴 유지.

    def _begin_speed(self):
        if self._saved_wheel is not None:
            cur_l = self.read(Hamster.LEFT_WHEEL)
            cur_r = self.read(Hamster.RIGHT_WHEEL)
            if (self._wrote_wheel is not None
                    and cur_l == self._wrote_wheel[0]
                    and cur_r == self._wrote_wheel[1]):
                saved_l, saved_r = self._saved_wheel
                self.write(Hamster.LEFT_WHEEL, saved_l)
                self.write(Hamster.RIGHT_WHEEL, saved_r)
            self._saved_wheel = None
            self._wrote_wheel = None
        self._saved_wheel = (
            self.read(Hamster.LEFT_WHEEL),
            self.read(Hamster.RIGHT_WHEEL),
        )

    def _mark_speed(self):
        self._wrote_wheel = (
            self.read(Hamster.LEFT_WHEEL),
            self.read(Hamster.RIGHT_WHEEL),
        )

    def _restore_speed(self):
        if self._saved_wheel is not None:
            saved_l, saved_r = self._saved_wheel
            self.write(Hamster.LEFT_WHEEL, saved_l)
            self.write(Hamster.RIGHT_WHEEL, saved_r)
            self._saved_wheel = None
            self._wrote_wheel = None

    # ── Internal helpers ──────────────────────────────────────────────────────
    
    @staticmethod
    def _get_speed(value=None):
        if value is None:
            return Hamster._SPEED
        return value * Hamster._VAL_TO_SPEED

    def _stop_move(self):
        # Hamster 는 WHEEL_MOVE 펄스 대신 시간 기반이라 bounded 분기 없이 항상 -128 sentinel.
        self._begin_speed()
        self.write(Hamster.LEFT_WHEEL, -128)
        self.write(Hamster.RIGHT_WHEEL, -128)
        self._mark_speed()

    def _stop_sound(self):
        for dev in (Hamster.SOUND_BUZZ, Hamster.SOUND_NOTE):
            if self.read(dev) > 0:
                self.write(dev, 0)

    # ── Grid (floor-sensor line-tracking; tick-based state machine) ──────────

    def _grid_move_forward(self):
        self._begin_speed()   # snapshot only — 다음 mover 의 begin 이 restore 처리
        self.write(Hamster.LEFT_WHEEL, 45)
        self.write(Hamster.RIGHT_WHEEL, 45)
        # Phase 1: drive with differential correction; advance after both floor sensors < 50
        # for 2 consecutive ticks (intersection patch).
        counter = 0
        while counter < 2:
            left = self.read(Hamster.LEFT_FLOOR)
            right = self.read(Hamster.RIGHT_FLOOR)
            if left < 50 and right < 50:
                counter += 1
            else:
                counter = 0
            diff = (left - right) * 0.25
            self.write(Hamster.LEFT_WHEEL, 45 + diff)
            self.write(Hamster.RIGHT_WHEEL, 45 - diff)
            Runner.wait(0.01)
        # Phase 2: 10 more ticks of differential correction past the patch.
        counter = 0
        while counter < 10:
            counter += 1
            left = self.read(Hamster.LEFT_FLOOR)
            right = self.read(Hamster.RIGHT_FLOOR)
            diff = (left - right) * 0.25
            self.write(Hamster.LEFT_WHEEL, 45 + diff)
            self.write(Hamster.RIGHT_WHEEL, 45 - diff)
            Runner.wait(0.01)
        self.write(Hamster.LEFT_WHEEL, 0)
        self.write(Hamster.RIGHT_WHEEL, 0)

    def _grid_turn_impl(self, dir):
        self._begin_speed()   # snapshot only
        if dir == 'left':
            self.write(Hamster.LEFT_WHEEL, -45)
            self.write(Hamster.RIGHT_WHEEL, 45)
            main_dev = Hamster.LEFT_FLOOR
            sub_dev = Hamster.RIGHT_FLOOR
        else:
            self.write(Hamster.LEFT_WHEEL, 45)
            self.write(Hamster.RIGHT_WHEEL, -45)
            main_dev = Hamster.RIGHT_FLOOR
            sub_dev = Hamster.LEFT_FLOOR
        # State 1: count main > 50 for 2 consecutive ticks
        counter = 0
        while counter < 2:
            if self.read(main_dev) > 50:
                counter += 1
            Runner.wait(0.01)
        # State 2: wait until main < 20
        while self.read(main_dev) >= 20:
            Runner.wait(0.01)
        # State 3: count main < 20 for 2 consecutive ticks
        counter = 0
        while counter < 2:
            if self.read(main_dev) < 20:
                counter += 1
            Runner.wait(0.01)
        # State 4: wait until main > 50
        while self.read(main_dev) <= 50:
            Runner.wait(0.01)
        # State 5: differential turn until alignment
        while True:
            diff = (self.read(main_dev) - self.read(sub_dev)) * 0.5
            if diff > -15:
                self.write(Hamster.LEFT_WHEEL, 0)
                self.write(Hamster.RIGHT_WHEEL, 0)
                break
            self.write(Hamster.LEFT_WHEEL, diff)
            self.write(Hamster.RIGHT_WHEEL, -diff)
            Runner.wait(0.01)

    def _grid_turn_left(self):
        self._grid_turn_impl('left')

    def _grid_turn_right(self):
        self._grid_turn_impl('right')

    # ── Move ──────────────────────────────────────────────────────────────────

    def set_wheel_speed(self, unit: _WheelUnit, speed: Union[int, float]):
        if unit not in Hamster._VALID_WHEEL_UNITS:
            return _err(Hamster, 'set_wheel_speed', 'unit', unit, Hamster._VALID_WHEEL_UNITS)
        if not isinstance(speed, (int, float)):
            return _err(Hamster, 'set_wheel_speed', 'speed', speed, 'int | float')
        if unit in ('both', 'left'):
            self.write(Hamster.LEFT_WHEEL, self._get_speed(speed))
        if unit in ('both', 'right'):
            self.write(Hamster.RIGHT_WHEEL, self._get_speed(speed))

    def change_wheel_speed(self, unit: _WheelUnit, speed: Union[int, float]):
        if unit not in Hamster._VALID_WHEEL_UNITS:
            return _err(Hamster, 'change_wheel_speed', 'unit', unit, Hamster._VALID_WHEEL_UNITS)
        if not isinstance(speed, (int, float)):
            return _err(Hamster, 'change_wheel_speed', 'speed', speed, 'int | float')
        if unit in ('both', 'left'):
            self.write(Hamster.LEFT_WHEEL, self.read(Hamster.LEFT_WHEEL) + self._get_speed(speed))
        if unit in ('both', 'right'):
            self.write(Hamster.RIGHT_WHEEL, self.read(Hamster.RIGHT_WHEEL) + self._get_speed(speed))

    def _move_time_impl(self, seconds):
        self._begin_speed()
        if self.read(Hamster.LEFT_WHEEL) == -128:
            self.write(Hamster.LEFT_WHEEL, Hamster._SPEED)
        if self.read(Hamster.RIGHT_WHEEL) == -128:
            self.write(Hamster.RIGHT_WHEEL, Hamster._SPEED)
        self._mark_speed()
        Runner.wait(seconds)
        self._stop_move()   # 자체 begin/mark 로 -128 sentinel 복원

    def move_time(self, data: Union[int, float], wait: bool = True):
        if not isinstance(data, (int, float)):
            return _err(Hamster, 'move_time', 'data', data, 'int | float')
        Runner.dispatch(lambda: self._move_time_impl(data), wait)

    def stop(self):
        self._stop_move()

    # ── Grid ──────────────────────────────────────────────────────────────────

    def grid_move(self, wait: bool = True):
        Runner.dispatch(self._grid_move_forward, wait)

    def grid_turn(self, direction: _GridDir, wait: bool = True):
        if direction not in Hamster._VALID_GRID_DIRS:
            return _err(Hamster, 'grid_turn', 'direction', direction, Hamster._VALID_GRID_DIRS)
        impl = self._grid_turn_left if direction == 'left' else self._grid_turn_right
        Runner.dispatch(impl, wait)

    # ── Trace ─────────────────────────────────────────────────────────────────

    def trace_line(self, floor: _Floor, line: _TraceLine = 'black'):
        if floor not in Hamster._VALID_FLOOR:
            return _err(Hamster, 'trace_line', 'floor', floor, Hamster._VALID_FLOOR)
        if line not in Hamster._VALID_TRACE_LINE:
            return _err(Hamster, 'trace_line', 'line', line, Hamster._VALID_TRACE_LINE)
        self.write(Hamster.LINE_TRACER_MODE, Hamster._TRACE_LINE_WIRE[(line, floor)])

    def _intersection_impl(self, wire):
        self.write(Hamster.LINE_TRACER_MODE, wire)
        Runner.wait_until(self._evaluate_line_tracer)

    def trace_intersection(self, direction: _IntersectionDir, line: _TraceLine = 'black', wait: bool = True):
        if direction not in Hamster._VALID_INTERSECTION_DIRS:
            return _err(Hamster, 'trace_intersection', 'direction', direction, Hamster._VALID_INTERSECTION_DIRS)
        if line not in Hamster._VALID_TRACE_LINE:
            return _err(Hamster, 'trace_intersection', 'line', line, Hamster._VALID_TRACE_LINE)
        wire = Hamster._INTERSECTION_WIRE[(line, direction)]
        Runner.dispatch(lambda: self._intersection_impl(wire), wait)

    def set_trace_speed(self, data: Union[int, float]):
        if not isinstance(data, (int, float)):
            return _err(Hamster, 'set_trace_speed', 'data', data, 'int | float')
        self.write(Hamster.LINE_TRACER_SPEED, data)

    def stop_trace(self):
        self.write(Hamster.LINE_TRACER_MODE, 0)

    # ── LED ───────────────────────────────────────────────────────────────────

    def set_led_color(self, unit: _LedUnit, color: _LedColor):
        if unit not in Hamster._VALID_LED_UNITS:
            return _err(Hamster, 'set_led_color', 'unit', unit, Hamster._VALID_LED_UNITS)
        if color not in Hamster._VALID_LED_COLORS:
            return _err(Hamster, 'set_led_color', 'color', color, tuple(Hamster._VALID_LED_COLORS))
        wire = Hamster._VALID_LED_COLORS[color]
        if unit in ('both', 'left'):
            self.write(Hamster.LEFT_LED, wire)
        if unit in ('both', 'right'):
            self.write(Hamster.RIGHT_LED, wire)

    def turn_off(self, unit: _LedUnit = 'both'):
        if unit not in Hamster._VALID_LED_UNITS:
            return _err(Hamster, 'turn_off', 'unit', unit, Hamster._VALID_LED_UNITS)
        if unit in ('both', 'left'):
            self.write(Hamster.LEFT_LED, 0)
        if unit in ('both', 'right'):
            self.write(Hamster.RIGHT_LED, 0)

    # ── Sound ─────────────────────────────────────────────────────────────────

    def sound_buzz(self, hz: Union[int, float]):
        if not isinstance(hz, (int, float)):
            return _err(Hamster, 'sound_buzz', 'hz', hz, 'int | float')
        self.write(Hamster.SOUND_BUZZ, hz)

    def sound_note(self, note: _Note, octave: int = 4):
        if note not in Hamster._VALID_NOTES:
            return _err(Hamster, 'sound_note', 'note', note, tuple(Hamster._VALID_NOTES))
        if not isinstance(octave, int) or not (1 <= octave <= 7):
            return _err(Hamster, 'sound_note', 'octave', octave, 'int (1~7)')
        self.write(Hamster.SOUND_NOTE, (octave - 1) * 12 + Hamster._VALID_NOTES[note] + 4)

    def sound_off(self):
        self._stop_sound()

    # ── Sensors ───────────────────────────────────────────────────────────────

    def wheel_speed(self, unit: _SensorSide) -> Union[int, float]:
        if unit not in Hamster._VALID_SENSOR_SIDES:
            return _err(Hamster, 'wheel_speed', 'unit', unit, Hamster._VALID_SENSOR_SIDES)
        return self.read(Hamster.LEFT_WHEEL if unit == 'left' else Hamster.RIGHT_WHEEL)

    def proximity(self, unit: _SensorSide) -> Union[int, float]:
        if unit not in Hamster._VALID_SENSOR_SIDES:
            return _err(Hamster, 'proximity', 'unit', unit, Hamster._VALID_SENSOR_SIDES)
        return self.read(Hamster.LEFT_PROXIMITY if unit == 'left' else Hamster.RIGHT_PROXIMITY)

    def floor(self, unit: _SensorSide) -> Union[int, float]:
        if unit not in Hamster._VALID_SENSOR_SIDES:
            return _err(Hamster, 'floor', 'unit', unit, Hamster._VALID_SENSOR_SIDES)
        return self.read(Hamster.LEFT_FLOOR if unit == 'left' else Hamster.RIGHT_FLOOR)

    def acceleration(self, unit: _Axis) -> Union[int, float]:
        if unit not in Hamster._VALID_AXIS:
            return _err(Hamster, 'acceleration', 'unit', unit, Hamster._VALID_AXIS)
        return self.read({'x': Hamster.ACCELERATION_X, 'y': Hamster.ACCELERATION_Y, 'z': Hamster.ACCELERATION_Z}[unit])

    def light(self) -> Union[int, float]:
        return self.read(Hamster.LIGHT)

    def temperature(self) -> Union[int, float]:
        return self.read(Hamster.TEMPERATURE)

    def signal_strength(self) -> Union[int, float]:
        return self.read(Hamster.SIGNAL_STRENGTH)

    def battery(self) -> Union[int, float]:
        return self.read(Hamster.BATTERY)

    # ── IO ────────────────────────────────────────────────────────────────────

    def io_mode(self, unit: _IoUnit, option: _IoMode):
        if unit not in Hamster._VALID_IO_UNITS:
            return _err(Hamster, 'io_mode', 'unit', unit, Hamster._VALID_IO_UNITS)
        if option not in Hamster._VALID_IO_MODES:
            return _err(Hamster, 'io_mode', 'option', option, Hamster._VALID_IO_MODES)
        wire = Hamster._VALID_IO_MODES[option]
        if unit in ('both', 'a'):
            self.write(Hamster.IO_A_MODE, wire)
        if unit in ('both', 'b'):
            self.write(Hamster.IO_B_MODE, wire)

    def set_output(self, unit: _IoUnit, data: Union[int, float]):
        if unit not in Hamster._VALID_IO_UNITS:
            return _err(Hamster, 'set_output', 'unit', unit, Hamster._VALID_IO_UNITS)
        if not isinstance(data, (int, float)):
            return _err(Hamster, 'set_output', 'data', data, 'int | float')
        if unit in ('both', 'a'):
            self.write(Hamster.IO_A_OUTPUT, data)
        if unit in ('both', 'b'):
            self.write(Hamster.IO_B_OUTPUT, data)

    def change_output(self, unit: _IoUnit, data: Union[int, float]):
        if unit not in Hamster._VALID_IO_UNITS:
            return _err(Hamster, 'change_output', 'unit', unit, Hamster._VALID_IO_UNITS)
        if not isinstance(data, (int, float)):
            return _err(Hamster, 'change_output', 'data', data, 'int | float')
        if unit in ('both', 'a'):
            self.write(Hamster.IO_A_OUTPUT, self.read(Hamster.IO_A_OUTPUT) + data)
        if unit in ('both', 'b'):
            self.write(Hamster.IO_B_OUTPUT, self.read(Hamster.IO_B_OUTPUT) + data)

    def open_gripper(self):
        self.write(Hamster.GRIPPER, 1)

    def close_gripper(self):
        self.write(Hamster.GRIPPER, 2)

    def shooter(self, data: int):
        if not isinstance(data, (int, float)):
            return _err(Hamster, 'shooter', 'data', data, 'int | float')
        self.write(Hamster.SHOOTER, data)

    def get_input(self, unit: _IoInUnit) -> Union[int, float]:
        if unit not in Hamster._VALID_IO_IN_UNITS:
            return _err(Hamster, 'get_input', 'unit', unit, Hamster._VALID_IO_IN_UNITS)
        return self.read(Hamster.IO_A_INPUT if unit == 'a' else Hamster.IO_B_INPUT)
