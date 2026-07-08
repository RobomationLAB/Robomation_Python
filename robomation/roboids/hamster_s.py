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
_GridDir         = Literal['left', 'right']
_PivotBase       = Literal['left_pen', 'right_pen', 'left_wheel', 'right_wheel']
_PivotDir        = Literal['forward', 'backward']
_CircleBase      = Literal['left_pen', 'right_pen']
_CircleDir       = Literal['left_forward', 'left_backward', 'right_forward', 'right_backward']
_Floor           = Literal['left', 'right', 'center']
_TraceLine       = Literal['black', 'white']
_IntersectionDir = Literal['left', 'right', 'forward', 'uturn']
_LedUnit         = Literal['left', 'right', 'both']
_Color           = Literal['black', 'red', 'yellow', 'green', 'cyan', 'blue', 'magenta', 'white']
_Note            = Literal['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B']
_Clip            = Literal['mute', 'beep', 'beep2', 'beep3', 'beep_repeat', 'beep_random', 'beep_random_repeat',
                            'noise', 'noise_repeat', 'siren', 'siren_repeat', 'engine', 'engine_repeat',
                            'fart_a', 'fart_b', 'noise_random', 'noise_random_repeat', 'whistle', 'chop', 'chop_repeat',
                            'robot', 'dibidibidip', 'random_melody', 'good_job',
                            'happy', 'angry', 'sad', 'sleep', 'march', 'birthday']
_SensorSide      = Literal['left', 'right']
_Axis            = Literal['x', 'y', 'z']
_IoUnit          = Literal['a', 'b', 'both']
_IoInUnit        = Literal['a', 'b']
_IoMode          = Literal['analog_input', 'analog_input_voltage',
                            'digital_input', 'digital_input_pullup', 'digital_input_pulldown', 
                            'servo_output', 'pwm_output', 'digital_output']

class HamsterS(Robot):
    ID = "kr.robomation.roboids.hamster_s"
    _robots = {}
    # _UNSET = object()

    # ── Device IDs (HamsterS 펌웨어 디바이스 식별자) ─────────────────────────
    # Effectors (0x00E000xx)
    LEFT_WHEEL         = 0x00E00000
    RIGHT_WHEEL        = 0x00E00001
    LINE_TRACER_SPEED  = 0x00E00002
    LINE_TRACER_GAIN   = 0x00E00003
    LEFT_RGB           = 0x00E00004
    RIGHT_RGB          = 0x00E00005
    SOUND_BUZZ         = 0x00E00006
    
    IO_A_MODE          = 0x00E00007
    IO_A_OUTPUT        = 0x00E00008
    IO_B_MODE          = 0x00E00009
    IO_B_OUTPUT        = 0x00E0000a
    GRIPPER            = 0x00E0000b
    SHOOTER            = 0x00E0000c

    CONFIG_IR_CURRENT  = 0x00E0000d
    CONFIG_G_RANGE     = 0x00E0000e
    CONFIG_G_BAND      = 0x00E0000f

    # Commands (0x00E001xx)
    WHEEL_MOVE         = 0x00E00100
    LINE_TRACER_MODE   = 0x00E00101
    SOUND_NOTE         = 0x00E00102
    SOUND_CLIP         = 0x00E00103

    # Sensors (0x00E002xx)
    WHEEL_COUNT        = 0x00E00200
    WHEEL_MOVING       = 0x00E00201
    SOUND_PLAYING      = 0x00E00202
    IO_A_INPUT         = 0x00E00203
    IO_B_INPUT         = 0x00E00204

    LEFT_PROXIMITY     = 0x00E00205
    RIGHT_PROXIMITY    = 0x00E00206
    LIGHT              = 0x00E00207
    LEFT_FLOOR         = 0x00E00208
    RIGHT_FLOOR        = 0x00E00209
    ACCELERATION_X     = 0x00E0020a
    ACCELERATION_Y     = 0x00E0020b
    ACCELERATION_Z     = 0x00E0020c
    BATTERY            = 0x00E0020d
    TEMPERATURE        = 0x00E0020e
    SIGNAL_STRENGTH    = 0x00E0020f

    # Events (0x00E003xx)
    WHEEL_STATE        = 0x00E00300
    LINE_TRACER_STATE  = 0x00E00301
    SOUND_STATE        = 0x00E00302
    TAP_STATE          = 0x00E00304

    # ── Robot-specific constants (from CONST.HAMSTER_S in player_codes.js) ────
    _SPEED = 30
    _VAL_TO_SPEED = 1
    _CM_TO_PULSE = 138.2666
    _DEG_TO_PULSE = 4.039
    _WHEEL_ROTATION_TO_PULSE = 2.46032
    _WHEEL_CENTER_DISTANCE = 1.636
    _PEN_CENTER_DISTANCE = 2.462
    _DEG_TO_PULSE_PIVOT = 8.047
    _DEG_TO_PULSE_PIVOT_PEN = 3.984
    _DEG_TO_PULSE_CIRCLE = 3.908
    _DEG_TO_PULSE_CIRCLE_PEN_SAME = 3.833
    _DEG_TO_PULSE_CIRCLE_PEN_DIFF = 3.916

    # ── Valid values (derived from module-level Literals) ────────────────────
    _VALID_WHEEL_UNITS          = get_args(_WheelUnit)
    _VALID_DIST_UNITS           = get_args(_DistUnit)
    _VALID_TURN_DIRS            = get_args(_TurnDir)
    _VALID_GRID_DIRS            = get_args(_GridDir)
    _VALID_PIVOT_BASES          = get_args(_PivotBase)
    _VALID_PIVOT_DIRS           = get_args(_PivotDir)
    _VALID_CIRCLE_BASES         = get_args(_CircleBase)
    _VALID_CIRCLE_DIRS          = get_args(_CircleDir)
    _VALID_FLOOR                = get_args(_Floor)
    _VALID_TRACE_LINE           = get_args(_TraceLine)
    _VALID_INTERSECTION_DIRS    = get_args(_IntersectionDir)
    _VALID_LED_UNITS            = get_args(_LedUnit)
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
    _VALID_NOTES                = {n: i for i, n in enumerate(get_args(_Note))}
    _VALID_CLIPS                = {
        'mute': 0,                'beep': 1,                'beep2': 2,                'beep3': 3,
        'beep_repeat': 4,         'beep_random': 5,         'beep_random_repeat': 6,
        'noise': 7,               'noise_repeat': 8,
        'siren': 9,               'siren_repeat': 10,
        'engine': 11,             'engine_repeat': 12,
        'fart_a': 13,             'fart_b': 14,
        'noise_random': 15,       'noise_random_repeat': 16,
        'whistle': 17,            'chop': 18,               'chop_repeat': 19,
        'robot': 32,              'dibidibidip': 33,        'random_melody': 34,      'good_job': 35,
        'happy': 48,              'angry': 49,              'sad': 50,                'sleep': 51,
        'march': 52,              'birthday': 53,
    }
    _VALID_SENSOR_SIDES         = get_args(_SensorSide)
    _VALID_AXIS                 = get_args(_Axis)
    _VALID_IO_UNITS             = get_args(_IoUnit)
    _VALID_IO_IN_UNITS          = get_args(_IoInUnit)
    _VALID_IO_MODES             = {
        'analog_input':           0,
        'analog_input_voltage':   4,
        'digital_input':          1,
        'digital_input_pullup':   2,
        'digital_input_pulldown': 3,
        'servo_output':           8,
        'pwm_output':             9,
        'digital_output':         10,
    }
    # _VALID_STATE_RANGE        = range(0, 8)                       # state_change

    # (line, floor) → LINE_TRACER_MODE wire 값  — trace_line() 용
    _TRACE_LINE_WIRE = {
        ('black', 'left'):   1,   ('black', 'right'):  2,   ('black', 'center'): 3,
        ('white', 'left'):   9,   ('white', 'right'):  10,   ('white', 'center'): 11,
    }
    # (line, direction) → LINE_TRACER_MODE wire 값  — intersection() 용
    _INTERSECTION_WIRE = {
        ('black', 'left'):    4,   ('black', 'right'):   5,
        ('black', 'forward'): 6,   ('black', 'uturn'):   7,
        ('white', 'left'):    12,  ('white', 'right'):   13,
        ('white', 'forward'): 14,  ('white', 'uturn'):   15,
    }

    def __init__(self, index=0, port_name=None):
        if isinstance(index, str):
            port_name = index
            index = 0
        if index in HamsterS._robots:
            robot = HamsterS._robots[index]
            if robot: robot.dispose()
        HamsterS._robots[index] = self
        super(HamsterS, self).__init__(HamsterS.ID, "HamsterS", index)

        # Wheel-speed snapshot 상태 
        self._saved_wheel = None       # (left_val, right_val) snapshot or None
        self._wrote_wheel = None       # (left_val, right_val) mover 가 쓴 값

        self._init(port_name)

    def dispose(self):
        HamsterS._robots[self.get_index()] = None
        self._roboid._dispose()
        Runner.unregister_robot(self)

    def reset(self):
        self._roboid._reset()

    def _init(self, port_name):
        from robomation.roboids.hamster_s_roboid import HamsterSRoboid
        self._roboid = HamsterSRoboid(self.get_index())
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
    def _evaluate_wheel_state(self):
        return self.e(HamsterS.WHEEL_STATE)

    def _evaluate_line_tracer(self):
        return self.e(HamsterS.LINE_TRACER_STATE)

    def _evaluate_sound(self):
        return self.e(HamsterS.SOUND_STATE)

    # ── Wheel-speed snapshot helpers ──────────────────────────────────────────
    # 사용자가 속도 미설정 상태로 이동을 시작 → 기본 속도로 동작 + 동작 후 미설정 상태로 복원.
    # 사용자가 속도 설정 후 이동 → 그 속도 유지. '정지' 후 다시 이동 시 직전 패턴 유지.

    def _begin_speed(self):
        if self._saved_wheel is not None:
            cur_l = self.read(HamsterS.LEFT_WHEEL)
            cur_r = self.read(HamsterS.RIGHT_WHEEL)
            # 직전 mover 가 쓴 값 그대로면 (= 사용자가 안 건드림) 원래로 복원
            if (self._wrote_wheel is not None
                    and cur_l == self._wrote_wheel[0]
                    and cur_r == self._wrote_wheel[1]):
                saved_l, saved_r = self._saved_wheel
                self.write(HamsterS.LEFT_WHEEL, saved_l)
                self.write(HamsterS.RIGHT_WHEEL, saved_r)
            self._saved_wheel = None
            self._wrote_wheel = None
        # 새 스냅샷
        self._saved_wheel = (
            self.read(HamsterS.LEFT_WHEEL),
            self.read(HamsterS.RIGHT_WHEEL),
        )

    def _mark_speed(self):
        self._wrote_wheel = (
            self.read(HamsterS.LEFT_WHEEL),
            self.read(HamsterS.RIGHT_WHEEL),
        )

    def _restore_speed(self):
        if self._saved_wheel is not None:
            saved_l, saved_r = self._saved_wheel
            self.write(HamsterS.LEFT_WHEEL, saved_l)
            self.write(HamsterS.RIGHT_WHEEL, saved_r)
            self._saved_wheel = None
            self._wrote_wheel = None

    # ── Internal helpers ──────────────────────────────────────────────────────

    @staticmethod
    def _get_speed(value=None):
        if value is None:
            return HamsterS._SPEED
        return value * HamsterS._VAL_TO_SPEED

    @staticmethod
    def _get_speed_input(value):
        return value / HamsterS._VAL_TO_SPEED

    @staticmethod
    def _get_distance(value, unit='cm'):
        d = value * HamsterS._CM_TO_PULSE
        if unit == 'mm': d /= 10
        elif unit == 'inch': d *= 2.54
        return d

    def _stop_move(self):
        bounded = self.read(HamsterS.WHEEL_MOVE) != 0
        self._begin_speed()
        self.write(HamsterS.WHEEL_MOVE, 0)
        self.write(HamsterS.LEFT_WHEEL, -128 if bounded else 0)
        self.write(HamsterS.RIGHT_WHEEL, -128 if bounded else 0)
        self._mark_speed()

    def _stop_sound(self):
        for dev in (HamsterS.SOUND_BUZZ, HamsterS.SOUND_NOTE, HamsterS.SOUND_CLIP):
            if self.read(dev) > 0:
                self.write(dev, 0)

    @staticmethod
    def _calc_inner_velocity(speed, radius=None):
        if radius is None:
            radius = HamsterS._PEN_CENTER_DISTANCE
        wcd = HamsterS._WHEEL_CENTER_DISTANCE
        return speed * (radius * 1.052 - wcd) / (radius * 1.052 + wcd)

    @staticmethod
    def _calc_circle_to_pulse(degree, radius, same_side):
        if not radius:
            radius = HamsterS._PEN_CENTER_DISTANCE
        option = 2.3008 if same_side else HamsterS._WHEEL_ROTATION_TO_PULSE
        return degree * (option * radius + 4.234)

    def _turn_degree_left(self, degree):
        # 항상 default speed 로 회전. 사용자가 set 한 wheel speed 는 begin/restore 로 보존만.
        if self.read(HamsterS.WHEEL_MOVE) != 0:
            self.write(HamsterS.WHEEL_MOVE, 0)
        self._begin_speed()
        speed = HamsterS._SPEED
        self.write(HamsterS.LEFT_WHEEL, -speed)
        self.write(HamsterS.RIGHT_WHEEL, speed)
        self._mark_speed()
        self.write(HamsterS.WHEEL_MOVE, degree * HamsterS._DEG_TO_PULSE)
        Runner.wait_until(self._evaluate_wheel_state)
        self._restore_speed()

    def _turn_degree_right(self, degree):
        if self.read(HamsterS.WHEEL_MOVE) != 0:
            self.write(HamsterS.WHEEL_MOVE, 0)
        self._begin_speed()
        speed = HamsterS._SPEED
        self.write(HamsterS.LEFT_WHEEL, speed)
        self.write(HamsterS.RIGHT_WHEEL, -speed)
        self._mark_speed()
        self.write(HamsterS.WHEEL_MOVE, degree * HamsterS._DEG_TO_PULSE)
        Runner.wait_until(self._evaluate_wheel_state)
        self._restore_speed()

    # ── Grid (floor-sensor line-tracking until both sensors < 20, then 1.3 cm push) ──

    def _grid_move_forward(self):
        if self.read(HamsterS.WHEEL_MOVE) != 0:
            self.write(HamsterS.WHEEL_MOVE, 0)
        self._begin_speed()   # snapshot only — 다음 mover 의 begin 이 restore 처리
        self.write(HamsterS.LEFT_WHEEL, 45)
        self.write(HamsterS.RIGHT_WHEEL, 45)
        # Track black line (45,45 baseline + differential correction) until both
        # floor sensors read < 20 (intersection patch).
        while True:
            left = self.read(HamsterS.LEFT_FLOOR)
            right = self.read(HamsterS.RIGHT_FLOOR)
            if left < 20 and right < 20:
                break
            if left < 50 and right < 50:
                self.write(HamsterS.LEFT_WHEEL, 45)
                self.write(HamsterS.RIGHT_WHEEL, 45)
            else:
                diff = (left - right) * 0.25
                self.write(HamsterS.LEFT_WHEEL, 45 + diff)
                self.write(HamsterS.RIGHT_WHEEL, 45 - diff)
            Runner.wait(0.01)
        # Final push: 1.3 cm forward to center on the intersection.
        self.write(HamsterS.LEFT_WHEEL, 45)
        self.write(HamsterS.RIGHT_WHEEL, 45)
        self.write(HamsterS.WHEEL_MOVE, self._get_distance(1.3, 'cm'))
        Runner.wait_until(self._evaluate_wheel_state)

    def _grid_turn_left(self):
        if self.read(HamsterS.WHEEL_MOVE) != 0:
            self.write(HamsterS.WHEEL_MOVE, 0)
        self._begin_speed()   # snapshot only
        self.write(HamsterS.LEFT_WHEEL, -45)
        self.write(HamsterS.RIGHT_WHEEL, 45)
        self.write(HamsterS.WHEEL_MOVE, self._get_distance(2.6, 'cm'))
        Runner.wait_until(self._evaluate_wheel_state)

    def _grid_turn_right(self):
        if self.read(HamsterS.WHEEL_MOVE) != 0:
            self.write(HamsterS.WHEEL_MOVE, 0)
        self._begin_speed()   # snapshot only
        self.write(HamsterS.LEFT_WHEEL, 45)
        self.write(HamsterS.RIGHT_WHEEL, -45)
        self.write(HamsterS.WHEEL_MOVE, self._get_distance(2.6, 'cm'))
        Runner.wait_until(self._evaluate_wheel_state)

    # ── Pivot ────────────────────────────────────────────────────────────────

    def _pivot(self, base, direction, degree):
        # 항상 default speed 사용. 사용자 wheel speed 는 begin/restore 로 보존만.
        if self.read(HamsterS.WHEEL_MOVE) != 0:
            self.write(HamsterS.WHEEL_MOVE, 0)
        side, mode = base.split('_')                 # 'left'/'right', 'pen'/'wheel'
        self._begin_speed()
        speed = HamsterS._SPEED
        signed = speed if direction == 'forward' else -speed
        if mode == 'pen':
            inner = self._calc_inner_velocity(signed)
            if side == 'left':
                self.write(HamsterS.LEFT_WHEEL, inner)
                self.write(HamsterS.RIGHT_WHEEL, signed)
            else:
                self.write(HamsterS.LEFT_WHEEL, signed)
                self.write(HamsterS.RIGHT_WHEEL, inner)
            self.write(HamsterS.WHEEL_MOVE, self._calc_circle_to_pulse(degree, 0, False))
        else:                                         # 'wheel'
            if side == 'left':
                self.write(HamsterS.LEFT_WHEEL, 0)
                self.write(HamsterS.RIGHT_WHEEL, signed)
            else:
                self.write(HamsterS.LEFT_WHEEL, signed)
                self.write(HamsterS.RIGHT_WHEEL, 0)
            self.write(HamsterS.WHEEL_MOVE, degree * HamsterS._DEG_TO_PULSE_PIVOT)
        self._mark_speed()
        Runner.wait_until(self._evaluate_wheel_state)
        self._restore_speed()

    def _pivot_circle(self, base, direction, radius, degree):
        # 항상 default speed 사용.
        if self.read(HamsterS.WHEEL_MOVE) != 0:
            self.write(HamsterS.WHEEL_MOVE, 0)
        pen = base.split('_')[0]                          # 'left' / 'right'
        turn_dir = direction.split('_')[0]                # 'left' / 'right'
        forward = direction.split('_')[1] == 'forward'
        self._begin_speed()
        speed = HamsterS._SPEED if forward else -HamsterS._SPEED
        radius_cm = radius / HamsterS._CM_TO_PULSE
        pen_dist = HamsterS._PEN_CENTER_DISTANCE

        if pen == 'left':
            if turn_dir == 'left':                        # left pen + left turn (same side)
                r = radius_cm + pen_dist
                self.write(HamsterS.LEFT_WHEEL, self._calc_inner_velocity(speed, r))
                self.write(HamsterS.RIGHT_WHEEL, speed)
                self.write(HamsterS.WHEEL_MOVE, self._calc_circle_to_pulse(degree, r, True))
            else:                                          # left pen + right turn (different side)
                if radius_cm >= pen_dist:
                    r = radius_cm - pen_dist
                    self.write(HamsterS.LEFT_WHEEL, speed)
                    self.write(HamsterS.RIGHT_WHEEL, self._calc_inner_velocity(speed, r))
                else:
                    r = (radius_cm - pen_dist) * -1
                    self.write(HamsterS.LEFT_WHEEL, self._calc_inner_velocity(speed, r) * -1)
                    self.write(HamsterS.RIGHT_WHEEL, speed * -1)
                self.write(HamsterS.WHEEL_MOVE, self._calc_circle_to_pulse(degree, r, False))
        else:                                              # pen == 'right'
            if turn_dir == 'left':                        # right pen + left turn (different side)
                if radius_cm >= pen_dist:
                    r = radius_cm - pen_dist
                    self.write(HamsterS.LEFT_WHEEL, self._calc_inner_velocity(speed, r))
                    self.write(HamsterS.RIGHT_WHEEL, speed)
                else:
                    r = (radius_cm - pen_dist) * -1
                    self.write(HamsterS.LEFT_WHEEL, speed * -1)
                    self.write(HamsterS.RIGHT_WHEEL, self._calc_inner_velocity(speed, r) * -1)
                self.write(HamsterS.WHEEL_MOVE, self._calc_circle_to_pulse(degree, r, False))
            else:                                          # right pen + right turn (same side)
                r = radius_cm + pen_dist
                self.write(HamsterS.LEFT_WHEEL, speed)
                self.write(HamsterS.RIGHT_WHEEL, self._calc_inner_velocity(speed, r))
                self.write(HamsterS.WHEEL_MOVE, self._calc_circle_to_pulse(degree, r, True))
        self._mark_speed()
        Runner.wait_until(self._evaluate_wheel_state)
        self._restore_speed()

    # ── Move ──────────────────────────────────────────────────────────────────

    def set_wheel_speed(self, unit: _WheelUnit, speed: Union[int, float]):
        if not isinstance(unit, str):
            return _err(HamsterS, 'set_wheel_speed', 'unit', unit, 'str')
        if unit not in HamsterS._VALID_WHEEL_UNITS:
            return _err(HamsterS, 'set_wheel_speed', 'unit', unit, HamsterS._VALID_WHEEL_UNITS)
        if not isinstance(speed, (int, float)):
            return _err(HamsterS, 'set_wheel_speed', 'speed', speed, 'int | float')
        if self.read(HamsterS.WHEEL_MOVE) != 0:
            self.write(HamsterS.WHEEL_MOVE, 0)
        if unit in ('both', 'left'):
            self.write(HamsterS.LEFT_WHEEL, self._get_speed(speed))
        if unit in ('both', 'right'):
            self.write(HamsterS.RIGHT_WHEEL, self._get_speed(speed))

    def _move_distance_impl(self, data, unit):
        self._begin_speed()
        # -128 (sentinel) → default, 아니면 사용자 값 그대로 (sign 은 data 부호로 결정)
        cur_l = self.read(HamsterS.LEFT_WHEEL)
        cur_r = self.read(HamsterS.RIGHT_WHEEL)
        base_l = HamsterS._SPEED if cur_l == -128 else cur_l
        base_r = HamsterS._SPEED if cur_r == -128 else cur_r
        self.write(HamsterS.LEFT_WHEEL, base_l if data > 0 else -base_l)
        self.write(HamsterS.RIGHT_WHEEL, base_r if data > 0 else -base_r)
        self._mark_speed()
        self.write(HamsterS.WHEEL_MOVE, self._get_distance(abs(data), unit))
        Runner.wait_until(self._evaluate_wheel_state)
        self._restore_speed()

    def move_distance(self, data: Union[int, float], unit: _DistUnit = 'cm', wait: bool = True):
        if not isinstance(data, (int, float)):
            return _err(HamsterS, 'move_distance', 'data', data, 'int | float')
        if unit not in HamsterS._VALID_DIST_UNITS:
            return _err(HamsterS, 'move_distance', 'unit', unit, HamsterS._VALID_DIST_UNITS)
        Runner.dispatch(lambda: self._move_distance_impl(data, unit), wait)

    def _move_time_impl(self, seconds):
        self._begin_speed()
        if self.read(HamsterS.LEFT_WHEEL) == -128:
            self.write(HamsterS.LEFT_WHEEL, HamsterS._SPEED)
        if self.read(HamsterS.RIGHT_WHEEL) == -128:
            self.write(HamsterS.RIGHT_WHEEL, HamsterS._SPEED)
        self._mark_speed()
        Runner.wait(seconds)
        self._stop_move()   # 자체 begin/mark 로 -128 sentinel 복원

    def move_time(self, data: Union[int, float], wait: bool = True):
        if not isinstance(data, (int, float)):
            return _err(HamsterS, 'move_time', 'data', data, 'int | float')
        Runner.dispatch(lambda: self._move_time_impl(data), wait)

    def turn_degree(self, direction: _TurnDir, data: Union[int, float], wait: bool = True):
        if not isinstance(data, (int, float)):
            return _err(HamsterS, 'turn_degree', 'data', data, 'int | float')
        if direction not in HamsterS._VALID_TURN_DIRS:
            return _err(HamsterS, 'turn_degree', 'direction', direction, HamsterS._VALID_TURN_DIRS)
        impl = self._turn_degree_left if direction == 'left' else self._turn_degree_right
        Runner.dispatch(lambda: impl(data), wait)

    def change_wheel_speed(self, unit: _WheelUnit, speed: Union[int, float]):
        if not isinstance(unit, str):
            return _err(HamsterS, 'change_wheel_speed', 'unit', unit, 'str')
        if unit not in HamsterS._VALID_WHEEL_UNITS:
            return _err(HamsterS, 'change_wheel_speed', 'unit', unit, HamsterS._VALID_WHEEL_UNITS)
        if not isinstance(speed, (int, float)):
            return _err(HamsterS, 'change_wheel_speed', 'speed', speed, 'int | float')
        if self.read(HamsterS.WHEEL_MOVE) != 0:
            self.write(HamsterS.WHEEL_MOVE, 0)
        if unit in ('both', 'left'):
            self.write(HamsterS.LEFT_WHEEL, self.read(HamsterS.LEFT_WHEEL) + self._get_speed(speed))
        if unit in ('both', 'right'):
            self.write(HamsterS.RIGHT_WHEEL, self.read(HamsterS.RIGHT_WHEEL) + self._get_speed(speed))

    def stop(self):
        self._stop_move()

    def wheel_moving(self) -> bool:
        return self.read(HamsterS.WHEEL_MOVING)

    # ── Grid ──────────────────────────────────────────────────────────────────

    def grid_move(self, wait: bool = True):
        Runner.dispatch(self._grid_move_forward, wait)

    def grid_turn(self, direction: _GridDir, wait: bool = True):
        if direction not in HamsterS._VALID_GRID_DIRS:
            return _err(HamsterS, 'grid_turn', 'direction', direction, HamsterS._VALID_GRID_DIRS)
        impl = self._grid_turn_left if direction == 'left' else self._grid_turn_right
        Runner.dispatch(impl, wait)

    def pivot(self, base: _PivotBase, direction: _PivotDir, degree: Union[int, float], wait: bool = True):
        if base not in HamsterS._VALID_PIVOT_BASES:
            return _err(HamsterS, 'pivot', 'base', base, HamsterS._VALID_PIVOT_BASES)
        if direction not in HamsterS._VALID_PIVOT_DIRS:
            return _err(HamsterS, 'pivot', 'direction', direction, HamsterS._VALID_PIVOT_DIRS)
        if not isinstance(degree, (int, float)):
            return _err(HamsterS, 'pivot', 'degree', degree, 'int | float')
        Runner.dispatch(lambda: self._pivot(base, direction, degree), wait)

    def pivot_circle(self, base: _CircleBase, direction: _CircleDir, degree: Union[int, float], radius: Union[int, float], unit: _DistUnit = 'cm', wait: bool = True):
        if base not in HamsterS._VALID_CIRCLE_BASES:
            return _err(HamsterS, 'pivot_circle', 'base', base, HamsterS._VALID_CIRCLE_BASES)
        if direction not in HamsterS._VALID_CIRCLE_DIRS:
            return _err(HamsterS, 'pivot_circle', 'direction', direction, HamsterS._VALID_CIRCLE_DIRS)
        if not isinstance(degree, (int, float)):
            return _err(HamsterS, 'pivot_circle', 'degree', degree, 'int | float')
        if not isinstance(radius, (int, float)):
            return _err(HamsterS, 'pivot_circle', 'radius', radius, 'int | float')
        if unit not in HamsterS._VALID_DIST_UNITS:
            return _err(HamsterS, 'pivot_circle', 'unit', unit, HamsterS._VALID_DIST_UNITS)
        Runner.dispatch(lambda: self._pivot_circle(base, direction, self._get_distance(radius, unit), degree), wait)

    # ── Trace ─────────────────────────────────────────────────────────────────

    def trace_line(self, floor: _Floor, line: _TraceLine = 'black'):
        if floor not in HamsterS._VALID_FLOOR:
            return _err(HamsterS, 'trace_line', 'floor', floor, HamsterS._VALID_FLOOR)
        if line not in HamsterS._VALID_TRACE_LINE:
            return _err(HamsterS, 'trace_line', 'line', line, HamsterS._VALID_TRACE_LINE)
        if self.read(HamsterS.WHEEL_MOVE) != 0:
            self._stop_move()
        self.write(HamsterS.LINE_TRACER_MODE, HamsterS._TRACE_LINE_WIRE[(line, floor)])

    def _intersection_impl(self, wire):
        if self.read(HamsterS.WHEEL_MOVE) != 0:
            self._stop_move()
        self.write(HamsterS.LINE_TRACER_MODE, wire)
        Runner.wait_until(self._evaluate_line_tracer)

    def trace_intersection(self, direction: _IntersectionDir, line: _TraceLine = 'black', wait: bool = True):
        if direction not in HamsterS._VALID_INTERSECTION_DIRS:
            return _err(HamsterS, 'trace_intersection', 'direction', direction, HamsterS._VALID_INTERSECTION_DIRS)
        if line not in HamsterS._VALID_TRACE_LINE:
            return _err(HamsterS, 'trace_intersection', 'line', line, HamsterS._VALID_TRACE_LINE)
        wire = HamsterS._INTERSECTION_WIRE[(line, direction)]
        Runner.dispatch(lambda: self._intersection_impl(wire), wait)

    def set_trace_speed(self, data: Union[int, float]):
        if not isinstance(data, (int, float)):
            return _err(HamsterS, 'set_trace_speed', 'data', data, 'int | float')
        self.write(HamsterS.LINE_TRACER_SPEED, data)

    def set_trace_gain(self, data: Union[int, float]):
        if not isinstance(data, (int, float)):
            return _err(HamsterS, 'set_trace_gain', 'data', data, 'int | float')
        self.write(HamsterS.LINE_TRACER_GAIN, data)

    def stop_trace(self):
        self.write(HamsterS.LINE_TRACER_MODE, 0)

    # ── LED ───────────────────────────────────────────────────────────────────

    def set_led_color(self, unit: _LedUnit, r: Union[_Color, int, float], g: Union[int, None] = None, b: Union[int, None] = None):
        if unit not in HamsterS._VALID_LED_UNITS:
            return _err(HamsterS, 'set_led_color', 'unit', unit, HamsterS._VALID_LED_UNITS)
        if isinstance(r, str):
            if r not in HamsterS._VALID_COLORS:
                return _err(HamsterS, 'set_led_color', 'color', r, tuple(HamsterS._VALID_COLORS))
            rgb = HamsterS._VALID_COLORS[r]
        else:
            for name, val in (('r', r), ('g', g), ('b', b)):
                if not isinstance(val, (int, float)):
                    return _err(HamsterS, 'set_led_color', name, val, 'int | float')
            rgb = [r, g, b]
        if unit in ('both', 'left'):
            self.write(HamsterS.LEFT_RGB, rgb)
        if unit in ('both', 'right'):
            self.write(HamsterS.RIGHT_RGB, rgb)

    def change_led_color(self, unit: _LedUnit, r: int, g: int, b: int):
        if unit not in HamsterS._VALID_LED_UNITS:
            return _err(HamsterS, 'change_led_color', 'unit', unit, HamsterS._VALID_LED_UNITS)
        for name, val in (('r', r), ('g', g), ('b', b)):
            if not isinstance(val, (int, float)):
                return _err(HamsterS, 'change_led_color', name, val, 'int | float')
        for side in ('left', 'right'):
            if unit in ('both', side):
                dev = HamsterS.LEFT_RGB if side == 'left' else HamsterS.RIGHT_RGB
                cur = [self.read(dev, 0), self.read(dev, 1), self.read(dev, 2)]
                self.write(dev, [cur[0] + r, cur[1] + g, cur[2] + b])

    def turn_off(self, unit: _LedUnit = 'both'):
        if not isinstance(unit, str):
            return _err(HamsterS, 'turn_off', 'unit', unit, 'str')
        if unit not in HamsterS._VALID_LED_UNITS:
            return _err(HamsterS, 'turn_off', 'unit', unit, HamsterS._VALID_LED_UNITS)
        if unit in ('both', 'left'):
            self.write(HamsterS.LEFT_RGB, [0, 0, 0])
        if unit in ('both', 'right'):
            self.write(HamsterS.RIGHT_RGB, [0, 0, 0])

    # ── Sound ─────────────────────────────────────────────────────────────────

    def sound_buzz(self, hz: Union[int, float]):
        if not isinstance(hz, (int, float)):
            return _err(HamsterS, 'sound_buzz', 'hz', hz, 'int | float')
        self.write(HamsterS.SOUND_BUZZ, hz)

    def sound_note(self, note: _Note, octave: int = 4):
        if note not in HamsterS._VALID_NOTES:
            return _err(HamsterS, 'sound_note', 'note', note, tuple(HamsterS._VALID_NOTES))
        if not isinstance(octave, int) or not (3 <= octave <= 7):
            return _err(HamsterS, 'sound_note', 'octave', octave, 'int (3~7)')
        self.write(HamsterS.SOUND_NOTE, (octave - 1) * 12 + HamsterS._VALID_NOTES[note] + 4)

    def _sound_clip_impl(self, clip):
        self.write(HamsterS.SOUND_CLIP, HamsterS._VALID_CLIPS[clip])
        Runner.wait_until(self._evaluate_sound)

    def sound_clip(self, clip: _Clip, wait: bool = True):
        if clip not in HamsterS._VALID_CLIPS:
            return _err(HamsterS, 'sound_clip', 'clip', clip, HamsterS._VALID_CLIPS)
        Runner.dispatch(lambda: self._sound_clip_impl(clip), wait)

    def sound_off(self):
        self._stop_sound()

    def sound_playing(self) -> bool:
        return self.read(HamsterS.SOUND_PLAYING)

    # ── Sensors ───────────────────────────────────────────────────────────────

    def wheel_speed(self, unit: _SensorSide) -> Union[int, float]:
        if unit not in HamsterS._VALID_SENSOR_SIDES:
            return _err(HamsterS, 'wheel_speed', 'unit', unit, HamsterS._VALID_SENSOR_SIDES)
        return self._get_speed_input(self.read(HamsterS.LEFT_WHEEL if unit == 'left' else HamsterS.RIGHT_WHEEL))

    def proximity(self, unit: _SensorSide) -> Union[int, float]:
        if unit not in HamsterS._VALID_SENSOR_SIDES:
            return _err(HamsterS, 'proximity', 'unit', unit, HamsterS._VALID_SENSOR_SIDES)
        return self.read(HamsterS.LEFT_PROXIMITY if unit == 'left' else HamsterS.RIGHT_PROXIMITY)

    def floor(self, unit: _SensorSide) -> Union[int, float]:
        if unit not in HamsterS._VALID_SENSOR_SIDES:
            return _err(HamsterS, 'floor', 'unit', unit, HamsterS._VALID_SENSOR_SIDES)
        return self.read(HamsterS.LEFT_FLOOR if unit == 'left' else HamsterS.RIGHT_FLOOR)

    def acceleration(self, unit: _Axis) -> Union[int, float]:
        if unit not in HamsterS._VALID_AXIS:
            return _err(HamsterS, 'acceleration', 'unit', unit, HamsterS._VALID_AXIS)
        return self.read({'x': HamsterS.ACCELERATION_X, 'y': HamsterS.ACCELERATION_Y, 'z': HamsterS.ACCELERATION_Z}[unit])

    def tap(self) -> bool:
        return self.e(HamsterS.TAP_STATE)

    def light(self) -> Union[int, float]:
        return self.read(HamsterS.LIGHT)

    def temperature(self) -> Union[int, float]:
        return self.read(HamsterS.TEMPERATURE)

    def signal_strength(self) -> Union[int, float]:
        return self.read(HamsterS.SIGNAL_STRENGTH)

    def battery(self) -> Union[int, float]:
        return self.read(HamsterS.BATTERY)

    # def state_change(self, unit=0):
    #     if not isinstance(unit, int) or unit not in HamsterS._VALID_STATE_RANGE:
    #         _err(HamsterS, 'state_change', 'unit', unit, 'int (0~7)')
    #     urn = self._urn
    #     conditions = [
    #         __(urn + 'acceleration.x').d > 5000,
    #         __(urn + 'acceleration.x').d < -5000,
    #         __(urn + 'acceleration.y').d > 5000,
    #         __(urn + 'acceleration.y').d < -5000,
    #         __(urn + 'acceleration.z').d > 0,
    #         __(urn + 'acceleration.z').d < -3000,
    #         __(urn + 'proximity.left').d > 50 or __(urn + 'proximity.right').d > 50,
    #         __(urn + 'acceleration.tap').e,
    #     ]
    #     return conditions[unit]

    # ── IO ────────────────────────────────────────────────────────────────────

    def io_mode(self, unit: _IoUnit, option: _IoMode):
        if unit not in HamsterS._VALID_IO_UNITS:
            return _err(HamsterS, 'io_mode', 'unit', unit, HamsterS._VALID_IO_UNITS)
        if option not in HamsterS._VALID_IO_MODES:
            return _err(HamsterS, 'io_mode', 'option', option, HamsterS._VALID_IO_MODES)
        wire = HamsterS._VALID_IO_MODES[option]
        if unit in ('both', 'a'):
            self.write(HamsterS.IO_A_MODE, wire)
        if unit in ('both', 'b'):
            self.write(HamsterS.IO_B_MODE, wire)

    def set_output(self, unit: _IoUnit, data: Union[int, float]):
        if unit not in HamsterS._VALID_IO_UNITS:
            return _err(HamsterS, 'set_output', 'unit', unit, HamsterS._VALID_IO_UNITS)
        if not isinstance(data, (int, float)):
            return _err(HamsterS, 'set_output', 'data', data, 'int | float')
        if unit in ('both', 'a'):
            self.write(HamsterS.IO_A_OUTPUT, data)
        if unit in ('both', 'b'):
            self.write(HamsterS.IO_B_OUTPUT, data)

    def change_output(self, unit: _IoUnit, data: Union[int, float]):
        if unit not in HamsterS._VALID_IO_UNITS:
            return _err(HamsterS, 'change_output', 'unit', unit, HamsterS._VALID_IO_UNITS)
        if not isinstance(data, (int, float)):
            return _err(HamsterS, 'change_output', 'data', data, 'int | float')
        if unit in ('both', 'a'):
            self.write(HamsterS.IO_A_OUTPUT, self.read(HamsterS.IO_A_OUTPUT) + data)
        if unit in ('both', 'b'):
            self.write(HamsterS.IO_B_OUTPUT, self.read(HamsterS.IO_B_OUTPUT) + data)

    def open_gripper(self):
        self.write(HamsterS.GRIPPER, 1)    

    def close_gripper(self):
        self.write(HamsterS.GRIPPER, 2)    
        
    def shooter(self, data: int):
        if not isinstance(data, (int, float)):
            return _err(HamsterS, 'shooter', 'data', data, 'int | float')
        self.write(HamsterS.SHOOTER, data)

    def get_input(self, unit: _IoInUnit) -> Union[int, float]:
        if unit not in HamsterS._VALID_IO_IN_UNITS:
            return _err(HamsterS, 'get_input', 'unit', unit, HamsterS._VALID_IO_IN_UNITS)
        return self.read(HamsterS.IO_A_INPUT if unit == 'a' else HamsterS.IO_B_INPUT)
