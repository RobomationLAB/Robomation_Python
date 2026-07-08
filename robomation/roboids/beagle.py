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
_Note            = Literal['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B']
_Clip            = Literal['mute', 'beep', 'beep2', 'beep3', 'beep_repeat', 'beep_random', 'beep_random_repeat',
                            'noise', 'noise_repeat', 'siren', 'siren_repeat', 'engine', 'engine_repeat',
                            'fart_a', 'fart_b', 'noise_random', 'noise_random_repeat', 'whistle', 'chop', 'chop_repeat',
                            'robot', 'dibidibidip', 'random_melody', 'good_job'
                            'happy', 'angry', 'sad', 'sleep', 'march', 'birthday']
_SensorSide      = Literal['left', 'right']
_Axis            = Literal['x', 'y', 'z']
_LidarDir        = Literal['front', 'left_front', 'left', 'left_back', 'back', 'right_back', 'right', 'right_front']


class Beagle(Robot):
    ID = "kr.robomation.physical.robot.beagle"
    _robots = {}

    # ── Device IDs (Beagle 펌웨어 디바이스 식별자) ───────────
    # Effectors (0x014000xx)
    MOTOR_SLEEP             = 0x01400000
    MOTOR_ACCELERATION      = 0x01400001
    LEFT_WHEEL              = 0x01400002
    RIGHT_WHEEL             = 0x01400003
    SOUND_BUZZ              = 0x01400004
    
    SERVO_A_SPEED           = 0x01400005
    SERVO_A_ANGLE           = 0x01400006
    SERVO_B_SPEED           = 0x01400007
    SERVO_B_ANGLE           = 0x01400008
    SERVO_C_SPEED           = 0x01400009
    SERVO_C_ANGLE           = 0x0140000a
    
    GYROSCOPE_ODR_BW        = 0x0140000b
    GYROSCOPE_RANGE         = 0x0140000c
    ACCELEROMETER_ODR_BW    = 0x0140000d
    ACCELEROMETER_RANGE     = 0x0140000e
    MAGNETOMETER_ODR        = 0x0140000f
    MAGNETOMETER_REP_XY     = 0x01400010
    MAGNETOMETER_REP_Z      = 0x01400011

    LIDAR_MODE              = 0x01400012
    LIDAR_CONNECT           = 0x01400013

    # Commands (0x014001xx)
    WHEEL_MOVE              = 0x01400100
    SOUND_NOTE              = 0x01400101
    SOUND_CLIP              = 0x01400102

    # Sensors (0x014002xx)
    INDEX                   = 0x01400200
    TIMESTAMP               = 0x01400201
    WHEEL_COUNT             = 0x01400202
    SOUND_PLAYING           = 0x01400203
    LEFT_ENCODER            = 0x01400204   
    RIGHT_ENCODER           = 0x01400205
    
    GYROSCOPE_INDEX         = 0x01400206
    GYROSCOPE_X             = 0x01400207
    GYROSCOPE_Y             = 0x01400208
    GYROSCOPE_Z             = 0x01400209
    ACCELEROMETER_INDEX     = 0x0140020a
    ACCELEROMETER_X         = 0x0140020b
    ACCELEROMETER_Y         = 0x0140020c
    ACCELEROMETER_Z         = 0x0140020d
    MAGNETOMETER_INDEX      = 0x0140020e
    MAGNETOMETER_X          = 0x0140020f
    MAGNETOMETER_Y          = 0x01400210
    MAGNETOMETER_Z          = 0x01400211
    
    BATTERY                 = 0x01400212
    TEMPERATURE             = 0x01400213
    SIGNAL_STRENGTH         = 0x01400214    
    
    LIDAR_INDEX             = 0x01400215
    LIDAR_READY             = 0x01400216
    LIDAR_ARRAY             = 0x01400217   
    LIDAR_DIRECTIONS        = 0x01400218
    LIDAR_VALID             = 0x01400219  
    
    # Events (0x014003xx)
    WHEEL_STATE             = 0x01400300
    SOUND_STATE             = 0x01400301

    # ── Robot-specific constants ──────────────────────────────────────────────
    _SPEED = 20
    _VAL_TO_SPEED = 1
    _CM_TO_PULSE = 92.59924
    _DEG_TO_PULSE = 7.777

    # ── Valid values (derived from module-level Literals) ────────────────────
    _VALID_WHEEL_UNITS      = get_args(_WheelUnit)
    _VALID_DIST_UNITS       = get_args(_DistUnit)
    _VALID_TURN_DIRS        = get_args(_TurnDir)
    _VALID_NOTES            = {n: i for i, n in enumerate(get_args(_Note))}
    _VALID_CLIPS            = {
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
    _VALID_SENSOR_SIDES     = get_args(_SensorSide)
    _VALID_AXIS             = get_args(_Axis)
    _VALID_LIDAR_DIRS       = get_args(_LidarDir)

    # ── Robot lifecycle ───────────────────────────────────────────────────────
    def __init__(self, index=0, port_name=None):
        if isinstance(index, str):
            port_name = index
            index = 0
        if index in Beagle._robots:
            robot = Beagle._robots[index]
            if robot: robot.dispose()
        Beagle._robots[index] = self
        super(Beagle, self).__init__(Beagle.ID, "Beagle", index)

        # Wheel-speed snapshot 상태 
        self._saved_wheel = None
        self._wrote_wheel = None

        self._init(port_name)

    def dispose(self):
        Beagle._robots[self.get_index()] = None
        self._roboid._dispose()
        Runner.unregister_robot(self)

    def reset(self):
        self._roboid._reset()

    def _init(self, port_name):
        from robomation.roboids.beagle_roboid import BeagleRoboid
        self._roboid = BeagleRoboid(self.get_index())
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
        return self.e(Beagle.WHEEL_STATE)
    def _evaluate_sound(self):
        return self.e(Beagle.SOUND_STATE)

    # ── Wheel-speed snapshot helpers (hamster_s 와 동일) ──────────────────────
    def _begin_speed(self):
        if self._saved_wheel is not None:
            cur_l = self.read(Beagle.LEFT_WHEEL)
            cur_r = self.read(Beagle.RIGHT_WHEEL)
            if (self._wrote_wheel is not None
                    and cur_l == self._wrote_wheel[0]
                    and cur_r == self._wrote_wheel[1]):
                saved_l, saved_r = self._saved_wheel
                self.write(Beagle.LEFT_WHEEL, saved_l)
                self.write(Beagle.RIGHT_WHEEL, saved_r)
            self._saved_wheel = None
            self._wrote_wheel = None
        self._saved_wheel = (
            self.read(Beagle.LEFT_WHEEL),
            self.read(Beagle.RIGHT_WHEEL),
        )

    def _mark_speed(self):
        self._wrote_wheel = (
            self.read(Beagle.LEFT_WHEEL),
            self.read(Beagle.RIGHT_WHEEL),
        )

    def _restore_speed(self):
        if self._saved_wheel is not None:
            saved_l, saved_r = self._saved_wheel
            self.write(Beagle.LEFT_WHEEL, saved_l)
            self.write(Beagle.RIGHT_WHEEL, saved_r)
            self._saved_wheel = None
            self._wrote_wheel = None

    # ── Internal helpers ──────────────────────────────────────────────────────
    @staticmethod
    def _get_speed(value=None):
        if value is None:
            return Beagle._SPEED
        return value * Beagle._VAL_TO_SPEED

    @staticmethod
    def _get_distance(value, unit='cm'):
        d = value * Beagle._CM_TO_PULSE
        if unit == 'mm': d /= 10
        elif unit == 'inch': d *= 2.54
        return d

    def _stop_move(self):
        bounded = self.read(Beagle.WHEEL_MOVE) != 0
        self._begin_speed()
        self.write(Beagle.WHEEL_MOVE, 0)
        self.write(Beagle.LEFT_WHEEL, -128 if bounded else 0)
        self.write(Beagle.RIGHT_WHEEL, -128 if bounded else 0)
        self._mark_speed()

    def _stop_sound(self):
        for dev in (Beagle.SOUND_BUZZ, Beagle.SOUND_NOTE, Beagle.SOUND_CLIP):
            if self.read(dev) > 0:
                self.write(dev, 0)

    # ── Turn impl ─────────────────────────────────────────────────────────────
    def _turn_degree_left(self, degree):
        if self.read(Beagle.WHEEL_MOVE) != 0:
            self.write(Beagle.WHEEL_MOVE, 0)
        self._begin_speed()
        speed = Beagle._SPEED
        self.write(Beagle.LEFT_WHEEL, -speed)
        self.write(Beagle.RIGHT_WHEEL, speed)
        self._mark_speed()
        self.write(Beagle.WHEEL_MOVE, degree * Beagle._DEG_TO_PULSE)
        Runner.wait_until(self._evaluate_wheel_state)
        self._restore_speed()

    def _turn_degree_right(self, degree):
        if self.read(Beagle.WHEEL_MOVE) != 0:
            self.write(Beagle.WHEEL_MOVE, 0)
        self._begin_speed()
        speed = Beagle._SPEED
        self.write(Beagle.LEFT_WHEEL, speed)
        self.write(Beagle.RIGHT_WHEEL, -speed)
        self._mark_speed()
        self.write(Beagle.WHEEL_MOVE, degree * Beagle._DEG_TO_PULSE)
        Runner.wait_until(self._evaluate_wheel_state)
        self._restore_speed()

    # ── Move ──────────────────────────────────────────────────────────────────
    def set_wheel_speed(self, unit: _WheelUnit, speed: Union[int, float]):
        if unit not in Beagle._VALID_WHEEL_UNITS:
            return _err(Beagle, 'set_wheel_speed', 'unit', unit, Beagle._VALID_WHEEL_UNITS)
        if not isinstance(speed, (int, float)):
            return _err(Beagle, 'set_wheel_speed', 'speed', speed, 'int | float')
        if self.read(Beagle.WHEEL_MOVE) != 0:
            self.write(Beagle.WHEEL_MOVE, 0)
        if unit in ('both', 'left'):
            self.write(Beagle.LEFT_WHEEL, self._get_speed(speed))
        if unit in ('both', 'right'):
            self.write(Beagle.RIGHT_WHEEL, self._get_speed(speed))

    def _move_distance_impl(self, data, unit):
        self._begin_speed()
        cur_l = self.read(Beagle.LEFT_WHEEL)
        cur_r = self.read(Beagle.RIGHT_WHEEL)
        base_l = Beagle._SPEED if cur_l == -128 else cur_l
        base_r = Beagle._SPEED if cur_r == -128 else cur_r
        self.write(Beagle.LEFT_WHEEL, base_l if data > 0 else -base_l)
        self.write(Beagle.RIGHT_WHEEL, base_r if data > 0 else -base_r)
        self._mark_speed()
        self.write(Beagle.WHEEL_MOVE, self._get_distance(abs(data), unit))
        Runner.wait_until(self._evaluate_wheel_state)
        self._restore_speed()

    def move_distance(self, data: Union[int, float], unit: _DistUnit = 'cm', wait: bool = True):
        if not isinstance(data, (int, float)):
            return _err(Beagle, 'move_distance', 'data', data, 'int | float')
        if unit not in Beagle._VALID_DIST_UNITS:
            return _err(Beagle, 'move_distance', 'unit', unit, Beagle._VALID_DIST_UNITS)
        Runner.dispatch(lambda: self._move_distance_impl(data, unit), wait)

    def _move_time_impl(self, seconds):
        self._begin_speed()
        if self.read(Beagle.LEFT_WHEEL) == -128:
            self.write(Beagle.LEFT_WHEEL, Beagle._SPEED)
        if self.read(Beagle.RIGHT_WHEEL) == -128:
            self.write(Beagle.RIGHT_WHEEL, Beagle._SPEED)
        self._mark_speed()
        Runner.wait(seconds)
        self._stop_move()

    def move_time(self, data: Union[int, float], wait: bool = True):
        if not isinstance(data, (int, float)):
            return _err(Beagle, 'move_time', 'data', data, 'int | float')
        Runner.dispatch(lambda: self._move_time_impl(data), wait)

    def turn_degree(self, direction: _TurnDir, data: Union[int, float], wait: bool = True):
        if direction not in Beagle._VALID_TURN_DIRS:
            return _err(Beagle, 'turn_degree', 'direction', direction, Beagle._VALID_TURN_DIRS)
        if not isinstance(data, (int, float)):
            return _err(Beagle, 'turn_degree', 'data', data, 'int | float')
        impl = self._turn_degree_left if direction == 'left' else self._turn_degree_right
        Runner.dispatch(lambda: impl(data), wait)

    def change_wheel_speed(self, unit: _WheelUnit, speed: Union[int, float]):
        if unit not in Beagle._VALID_WHEEL_UNITS:
            return _err(Beagle, 'change_wheel_speed', 'unit', unit, Beagle._VALID_WHEEL_UNITS)
        if not isinstance(speed, (int, float)):
            return _err(Beagle, 'change_wheel_speed', 'speed', speed, 'int | float')
        if self.read(Beagle.WHEEL_MOVE) != 0:
            self.write(Beagle.WHEEL_MOVE, 0)
        if unit in ('both', 'left'):
            self.write(Beagle.LEFT_WHEEL, self.read(Beagle.LEFT_WHEEL) + self._get_speed(speed))
        if unit in ('both', 'right'):
            self.write(Beagle.RIGHT_WHEEL, self.read(Beagle.RIGHT_WHEEL) + self._get_speed(speed))

    def stop(self):
        self._stop_move()

    def wheel_moving(self) -> bool:
        return self.read(Beagle.LEFT_WHEEL) != 0 or self.read(Beagle.RIGHT_WHEEL) != 0

    # ── Sound ─────────────────────────────────────────────────────────────────
    def sound_buzz(self, hz: Union[int, float]):
        if not isinstance(hz, (int, float)):
            return _err(Beagle, 'sound_buzz', 'hz', hz, 'int | float')
        self._stop_sound()
        self.write(Beagle.SOUND_BUZZ, hz)

    def sound_note(self, note: _Note, octave: int = 4):
        if note not in Beagle._VALID_NOTES:
            return _err(Beagle, 'sound_note', 'note', note, tuple(Beagle._VALID_NOTES))
        if not isinstance(octave, int) or not (1 <= octave <= 7):
            return _err(Beagle, 'sound_note', 'octave', octave, 'int (1~7)')
        self._stop_sound()
        self.write(Beagle.SOUND_NOTE, (octave - 1) * 12 + Beagle._VALID_NOTES[note] + 4)

    def _sound_clip_impl(self, clip):
        self._stop_sound()
        self.write(Beagle.SOUND_CLIP, Beagle._VALID_CLIPS[clip])
        Runner.wait_until(self._evaluate_sound)

    def sound_clip(self, clip: _Clip, wait: bool = True):
        if clip not in Beagle._VALID_CLIPS:
            return _err(Beagle, 'sound_clip', 'clip', clip, tuple(Beagle._VALID_CLIPS))
        Runner.dispatch(lambda: self._sound_clip_impl(clip), wait)

    def sound_off(self):
        self._stop_sound()

    def sound_playing(self) -> bool:
        return self.read(Beagle.SOUND_PLAYING)

    # ── Sensors ───────────────────────────────────────────────────────────────
    def wheel_speed(self, unit: _SensorSide) -> Union[int, float]:
        if unit not in Beagle._VALID_SENSOR_SIDES:
            return _err(Beagle, 'wheel_speed', 'unit', unit, Beagle._VALID_SENSOR_SIDES)
        return self.read(Beagle.LEFT_WHEEL if unit == 'left' else Beagle.RIGHT_WHEEL)

    def encoder(self, unit: _SensorSide) -> Union[int, float]:
        if unit not in Beagle._VALID_SENSOR_SIDES:
            return _err(Beagle, 'encoder', 'unit', unit, Beagle._VALID_SENSOR_SIDES)
        return self.read(Beagle.LEFT_ENCODER if unit == 'left' else Beagle.RIGHT_ENCODER)

    def gyroscope(self, unit: _Axis) -> Union[int, float]:
        if unit not in Beagle._VALID_AXIS:
            return _err(Beagle, 'gyroscope', 'unit', unit, Beagle._VALID_AXIS)
        return self.read({'x': Beagle.GYROSCOPE_X, 'y': Beagle.GYROSCOPE_Y, 'z': Beagle.GYROSCOPE_Z}[unit])

    def accelerometer(self, unit: _Axis) -> Union[int, float]:
        if unit not in Beagle._VALID_AXIS:
            return _err(Beagle, 'accelerometer', 'unit', unit, Beagle._VALID_AXIS)
        return self.read({'x': Beagle.ACCELEROMETER_X, 'y': Beagle.ACCELEROMETER_Y, 'z': Beagle.ACCELEROMETER_Z}[unit])

    def magnetometer(self, unit: _Axis) -> Union[int, float]:
        if unit not in Beagle._VALID_AXIS:
            return _err(Beagle, 'magnetometer', 'unit', unit, Beagle._VALID_AXIS)
        return self.read({'x': Beagle.MAGNETOMETER_X, 'y': Beagle.MAGNETOMETER_Y, 'z': Beagle.MAGNETOMETER_Z}[unit])

    def temperature(self) -> Union[int, float]:
        return self.read(Beagle.TEMPERATURE)

    def signal_strength(self) -> Union[int, float]:
        return self.read(Beagle.SIGNAL_STRENGTH)

    def battery(self) -> Union[int, float]:
        return self.read(Beagle.BATTERY)

    # ── LIDAR ─────────────────────────────────────────────────────────────────
    def lidar_power(self, on: bool = True):
        if not isinstance(on, bool):
            return _err(Beagle, 'lidar_power', 'on', on, 'bool')
        self.write(Beagle.LIDAR_CONNECT, 1 if on else 0)

    def lidar_value(self, unit: int) -> Union[int, float]:
        if not isinstance(unit, int) or not (0 <= unit <= 359):
            return _err(Beagle, 'lidar_value', 'unit', unit, 'int (0~359)')
        return self.read(Beagle.LIDAR_ARRAY, unit)

    def lidar_directions(self, direction: _LidarDir) -> Union[int, float]:
        if direction not in Beagle._VALID_LIDAR_DIRS:
            return _err(Beagle, 'lidar_directions', 'direction', direction, Beagle._VALID_LIDAR_DIRS)
        return self.read(Beagle.LIDAR_DIRECTIONS, Beagle._VALID_LIDAR_DIRS.index(direction))

    def lidar_ready(self) -> bool:
        return bool(self.read(Beagle.LIDAR_READY))
