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

import math

from typing import Literal, Union, get_args

from robomation.core.error import _err
from robomation.core.runner import Runner
from robomation.core.model import Robot

from robomation.cheesestick_ext.csd01 import CSD01 as _CSD01
from robomation.cheesestick_ext.csd02 import CSD02 as _CSD02
from robomation.cheesestick_ext.csd03 import CSD03 as _CSD03
from robomation.cheesestick_ext.csd07 import CSD07 as _CSD07
from robomation.cheesestick_ext.csd09 import CSD09 as _CSD09
from robomation.cheesestick_ext.csd10 import CSD10 as _CSD10
from robomation.cheesestick_ext.hat010 import HAT010 as _HAT010
from robomation.cheesestick_ext.hat022 import HAT022 as _HAT022
# from robomation.cheesestick_ext.pid10 import PID10 as _PID10
from robomation.cheesestick_ext.pid13 import PID13 as _PID13
from robomation.cheesestick_ext.pid26 import PID26 as _PID26
from robomation.cheesestick_ext.neopixel import NeoPixel as _NeoPixel


# ── Module-level Literal aliases (for IDE auto-complete) ─────────────────────
_InputMode          = Literal['makey', 'button', 'digital_pullup', 'digital_pulldown', 'analog', 'analog_voltage']
_OutputMode         = Literal['digital', 'pwm', 'analog_servo']
_PulseOpt           = Literal['default', 'pull-up', 'pull-down']
_BgvOpt             = Literal['default', 'voltage']
_InputUnit          = Literal['Sa', 'Sb', 'Sc', 'La', 'Lb', 'Lc']
_PulseInputUnit     = Literal['Sc', 'Lc']
_DigitalOutputUnit  = Literal['Sa', 'Sb', 'Sc', 'La', 'Lb', 'Lc', 'Mab', 'Mcd']
_PwmOutputUnit      = Literal['Sa', 'Sb', 'Sc', 'La', 'Lb', 'Lc']
_Note               = Literal['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B']
_Clip               = Literal['mute', 'beep', 'beep2', 'beep3', 'beep_repeat', 'beep_random', 'beep_random_repeat',
                            'noise', 'noise_repeat', 'siren', 'siren_repeat', 'engine', 'engine_repeat',
                            'fart_a', 'fart_b', 'noise_random', 'noise_random_repeat', 'whistle', 'chop', 'chop_repeat',
                            'robot', 'dibidibidip', 'random_melody', 'good_job',
                            'happy', 'angry', 'sad', 'sleep', 'march', 'birthday']
_Axis               = Literal['x', 'y', 'z']


class CheeseStick(Robot):
    ID = "kr.robomation.physical.robot.cheesestick"
    _robots = {}

    # ── Device IDs (CheeseStick 펌웨어 디바이스 식별자) ──────
    # Effectors (0x00D000xx)
    # S 포트
    S_PULSE_DETECT      = 0x00D00000
    S_NEO               = 0x00D00001

    SA_MODE             = 0x00D00002
    SA_PULL             = 0x00D00003
    SA_BGV              = 0x00D00004
    SA_SRC_RANGE        = 0x00D00005
    SA_DST_RANGE        = 0x00D00006
    SA_OUTPUT_DIGITAL   = 0x00D00007
    SA_OUTPUT_PWM       = 0x00D00008
    SA_OUTPUT_SERVO     = 0x00D00009

    SB_MODE             = 0x00D0000a
    SB_PULL             = 0x00D0000b
    SB_BGV              = 0x00D0000c
    SB_SRC_RANGE        = 0x00D0000d
    SB_DST_RANGE        = 0x00D0000e
    SB_OUTPUT_DIGITAL   = 0x00D0000f
    SB_OUTPUT_PWM       = 0x00D00010
    SB_OUTPUT_SERVO     = 0x00D00011

    SC_MODE             = 0x00D00012
    SC_PULL             = 0x00D00013
    SC_BGV              = 0x00D00014
    SC_SRC_RANGE        = 0x00D00015
    SC_DST_RANGE        = 0x00D00016
    SC_OUTPUT_DIGITAL   = 0x00D00017
    SC_OUTPUT_PWM       = 0x00D00018
    SC_OUTPUT_SERVO     = 0x00D00019

    # L 포트
    L_PULSE_DETECT      = 0x00D0001a

    LA_MODE             = 0x00D0001b
    LA_PULL             = 0x00D0001c
    LA_BGV              = 0x00D0001d
    LA_SRC_RANGE        = 0x00D0001e
    LA_DST_RANGE        = 0x00D0001f
    LA_OUTPUT_DIGITAL   = 0x00D00020
    LA_OUTPUT_PWM       = 0x00D00021
    LA_OUTPUT_SERVO     = 0x00D00022

    LB_MODE             = 0x00D00023
    LB_PULL             = 0x00D00024
    LB_BGV              = 0x00D00025
    LB_SRC_RANGE        = 0x00D00026
    LB_DST_RANGE        = 0x00D00027
    LB_OUTPUT_DIGITAL   = 0x00D00028
    LB_OUTPUT_PWM       = 0x00D00029
    LB_OUTPUT_SERVO     = 0x00D0002a

    LC_MODE             = 0x00D0002b
    LC_PULL             = 0x00D0002c
    LC_BGV              = 0x00D0002d
    LC_SRC_RANGE        = 0x00D0002e
    LC_DST_RANGE        = 0x00D0002f
    LC_OUTPUT_DIGITAL   = 0x00D00030
    LC_OUTPUT_PWM       = 0x00D00031
    LC_OUTPUT_SERVO     = 0x00D00032

    # M 포트
    M_MODE              = 0x00D00033
    M_DRIVER            = 0x00D00034
    M_STEP_PPS          = 0x00D00035

    MAB_MODE            = 0x00D00036
    MAB_MOTOR_A         = 0x00D00037
    MAB_MOTOR_B         = 0x00D00038
    MAB_OUTPUT_DIGITAL  = 0x00D00039
    MAB_OUTPUT_PWM      = 0x00D0003a
    MAB_OUTPUT_SERVO    = 0x00D0003b

    MCD_MODE            = 0x00D0003c
    MCD_MOTOR_C         = 0x00D0003d
    MCD_MOTOR_D         = 0x00D0003e
    MCD_OUTPUT_DIGITAL  = 0x00D0003f
    MCD_OUTPUT_PWM      = 0x00D00040
    MCD_OUTPUT_SERVO    = 0x00D00041

    # 이외
    SOUND_BUZZ          = 0x00D00042
    ACCEL_G_RANGE       = 0x00D00043
    ACCEL_BANDWIDTH     = 0x00D00044

    # Commands (0x00D001xx)
    M_STEP_MOVE         = 0x00D00100
    SOUND_NOTE          = 0x00D00101
    SOUND_CLIP          = 0x00D00102

    # Sensors (0x00D002xx)
    SA_INPUT            = 0x00D00200
    SB_INPUT            = 0x00D00201
    SC_INPUT            = 0x00D00202
    SC_PULSE_INPUT_STATE    = 0x00D00203
    SC_PULSE_INPUT_COUNT    = 0x00D00204

    LA_INPUT            = 0x00D00205
    LB_INPUT            = 0x00D00206
    LC_INPUT            = 0x00D00207
    LC_PULSE_INPUT_STATE    = 0x00D00208
    LC_PULSE_INPUT_COUNT    = 0x00D00209

    ACCELERATION_X      = 0x00D0020a
    ACCELERATION_Y      = 0x00D0020b
    ACCELERATION_Z      = 0x00D0020c
    TEMPERATURE         = 0x00D0020d
    SIGNAL_STRENGTH     = 0x00D0020e
    BATTERY             = 0x00D0020f

    # Events (0x00D003xx)
    M_STEP_STATE        = 0x00D00300
    SOUND_STATE         = 0x00D00301
    SC_PULSE_INPUT_DETECT   = 0x00D00302
    LC_PULSE_INPUT_DETECT   = 0x00D00303

    TAP_STATE           = 0x00D00304
    FALL_STATE          = 0x00D00305

    # ── HAT 디바이스 ──────
    HAT                         = 0x00D20000

    # ── HAT010 (packet 타입: 0x2A) ──────
    HAT010_X                    = 0x00D2A000
    HAT010_Y                    = 0x00D2A001
    HAT010_ORIGIN_X             = 0x00D2A002
    HAT010_ORIGIN_Y             = 0x00D2A003
    HAT010_BRIGHTNESS           = 0x00D2A004

    HAT010_LED                  = 0x00D2A100
    HAT010_DRAW                 = 0x00D2A101
    HAT010_CLEAR                = 0x00D2A102

    HAT010_BUTTON_A             = 0x00D2A200
    HAT010_BUTTON_B             = 0x00D2A201

    HAT010_LED_STATE            = 0x00D2A300
    HAT010_DRAW_STATE           = 0x00D2A301
    HAT010_BUTTON_A_STATE       = 0x00D2A302
    HAT010_BUTTON_B_STATE       = 0x00D2A303

    # ── HAT022 (packet 타입: 0x26) ──────
    HAT022_C                    = 0x00D26200
    HAT022_C_SHARP              = 0x00D26201
    HAT022_D                    = 0x00D26202
    HAT022_D_SHARP              = 0x00D26203
    HAT022_E                    = 0x00D26204
    HAT022_F                    = 0x00D26205
    HAT022_F_SHARP              = 0x00D26206
    HAT022_G                    = 0x00D26207
    HAT022_G_SHARP              = 0x00D26208
    HAT022_A                    = 0x00D26209
    HAT022_A_SHARP              = 0x00D2620a
    HAT022_B                    = 0x00D2620b
    HAT022_LEFT                 = 0x00D2620c
    HAT022_RIGHT                = 0x00D2620d
    HAT022_FN                   = 0x00D2620e

    # ── PID 디바이스 ──────
    PID                         = 0x00D30000

    # PID10_DISTANCE              = 0x00D3A200
    # PID10_ECHOTIME              = 0x00D3A201

    PID13_X                     = 0x00D3D200
    PID13_Y                     = 0x00D3D201
    PID13_BUTTON_A              = 0x00D3D202
    PID13_BUTTON_B              = 0x00D3D203

    PID13_BUTTON_A_STATE        = 0x00D3D300
    PID13_BUTTON_B_STATE        = 0x00D3D301

    PID26_PRESSURE              = 0x00D4A200
    PID26_TEMPERATURE           = 0x00D4A201
    PID26_HUMIDITY              = 0x00D4A202

    # ── NeoPixel (packet 타입: 0x40 + 0x05) ──────
    NEO_MODE                    = 0x00D45000
    NEO_FROM                    = 0x00D45001
    NEO_TO                      = 0x00D45002
    NEO_INCREMENT               = 0x00D45003
    NEO_BRIGHTNESS              = 0x00D45004

    NEO_RED                     = 0x00D45005
    NEO_RED_CHANGE              = 0x00D45006
    NEO_GREEN                   = 0x00D45007
    NEO_GREEN_CHANGE            = 0x00D45008
    NEO_BLUE                    = 0x00D45009
    NEO_BLUE_CHANGE             = 0x00D4500a
    NEO_WHITE                   = 0x00D4500b
    NEO_WHITE_CHANGE            = 0x00D4500c

    NEO_PATTERN_MODE            = 0x00D4500d
    NEO_PATTERN_BLOCK           = 0x00D4500e
    NEO_PATTERN_SKIP            = 0x00D4500f
    NEO_PATTERN_CLEAR           = 0x00D45010

    NEO_SHIFT_MODE              = 0x00D45011
    NEO_SHIFT_DIRECTION         = 0x00D45012
    NEO_SHIFT_PIXEL             = 0x00D45013

    NEO_COMMAND                 = 0x00D45100

    # ── Port → device ID 매핑 (각 포트 dict 는 작업별로 분리) ────────────────
    _INPUT_DEVICE_IDS = {
        'Sa': SA_INPUT, 'Sb': SB_INPUT, 'Sc': SC_INPUT,
        'La': LA_INPUT, 'Lb': LB_INPUT, 'Lc': LC_INPUT,
    }
    _MODE_DEVICE_IDS = {
        'Sa': SA_MODE, 'Sb': SB_MODE, 'Sc': SC_MODE,
        'La': LA_MODE, 'Lb': LB_MODE, 'Lc': LC_MODE,
        'Mab': MAB_MODE, 'Mcd': MCD_MODE,
    }
    _PULL_DEVICE_IDS = {
        'Sa': SA_PULL, 'Sb': SB_PULL, 'Sc': SC_PULL,
        'La': LA_PULL, 'Lb': LB_PULL, 'Lc': LC_PULL,
    }
    _BGV_DEVICE_IDS = {
        'Sa': SA_BGV, 'Sb': SB_BGV, 'Sc': SC_BGV,
        'La': LA_BGV, 'Lb': LB_BGV, 'Lc': LC_BGV,
    }
    _SRC_RANGE_DEVICE_IDS = {
        'Sa': SA_SRC_RANGE, 'Sb': SB_SRC_RANGE, 'Sc': SC_SRC_RANGE,
        'La': LA_SRC_RANGE, 'Lb': LB_SRC_RANGE, 'Lc': LC_SRC_RANGE,
    }
    _DST_RANGE_DEVICE_IDS = {
        'Sa': SA_DST_RANGE, 'Sb': SB_DST_RANGE, 'Sc': SC_DST_RANGE,
        'La': LA_DST_RANGE, 'Lb': LB_DST_RANGE, 'Lc': LC_DST_RANGE,
    }
    _DIGITAL_OUT_DEVICE_IDS = {
        'Sa': SA_OUTPUT_DIGITAL, 'Sb': SB_OUTPUT_DIGITAL, 'Sc': SC_OUTPUT_DIGITAL,
        'La': LA_OUTPUT_DIGITAL, 'Lb': LB_OUTPUT_DIGITAL, 'Lc': LC_OUTPUT_DIGITAL,
        'Mab': MAB_OUTPUT_DIGITAL, 'Mcd': MCD_OUTPUT_DIGITAL,
    }
    _PWM_OUT_DEVICE_IDS = {
        'Sa': SA_OUTPUT_PWM, 'Sb': SB_OUTPUT_PWM, 'Sc': SC_OUTPUT_PWM,
        'La': LA_OUTPUT_PWM, 'Lb': LB_OUTPUT_PWM, 'Lc': LC_OUTPUT_PWM,
    }
    _SERVO_OUT_DEVICE_IDS = {
        'Sa': SA_OUTPUT_SERVO, 'Sb': SB_OUTPUT_SERVO, 'Sc': SC_OUTPUT_SERVO,
        'La': LA_OUTPUT_SERVO, 'Lb': LB_OUTPUT_SERVO, 'Lc': LC_OUTPUT_SERVO,
    }
    _PULSE_INPUT_DETECT_DEVICE_IDS = {
        'Sc': SC_PULSE_INPUT_DETECT,
        'Lc': LC_PULSE_INPUT_DETECT,
    }
    _PULSE_INPUT_STATE_DEVICE_IDS = {
        'Sc': SC_PULSE_INPUT_STATE,
        'Lc': LC_PULSE_INPUT_STATE,
    }

    # ── Valid values (derived from module-level Literals) ────────────────────
    _VALID_INPUT_MODES          = {
        'makey': 0,             'button': 0,    
        'digital_pullup': 0,    'digital_pulldown': 0,
        'analog': 1,            'analog_voltage': 1,
    }
    _VALID_OUTPUT_MODES         = {
        'digital': 2,           'pwm': 2,                  'analog_servo': 3,
    }
    _VALID_PULSE_OPTS           = {p: i for i, p in enumerate(get_args(_PulseOpt))}
    _VALID_BGV_OPTS             = {b: i for i, b in enumerate(get_args(_BgvOpt))}
    _VALID_INPUT_UNITS          = get_args(_InputUnit)
    _VALID_PULSE_INPUT_UNITS    = get_args(_PulseInputUnit)
    _VALID_DIGITAL_OUTPUT_UNITS = get_args(_DigitalOutputUnit)
    _VALID_PWM_OUTPUT_UNITS     = get_args(_PwmOutputUnit)
    _VALID_NOTES                = {n: i for i, n in enumerate(get_args(_Note))}
    _VALID_CLIPS                = {
        'mute': 0,               'beep': 1,                 'beep2': 2,                'beep3': 3,
        'beep_repeat': 4,        'beep_random': 5,          'beep_random_repeat': 6,
        'noise': 7,              'noise_repeat': 8,
        'siren': 9,              'siren_repeat': 10,
        'engine': 11,            'engine_repeat': 12,
        'fart_a': 13,            'fart_b': 14,
        'noise_random': 15,      'noise_random_repeat': 16,
        'whistle': 17,           'chop': 18,                'chop_repeat': 19,
        'robot': 32,             'dibidibidip': 33,         'random_melody': 34,      'good_job': 35,
        'happy': 48,             'angry': 49,               'sad': 50,                'sleep': 51,
        'march': 52,             'birthday': 53,
    }
    _VALID_AXIS                 = get_args(_Axis)

    # ── Robot lifecycle ───────────────────────────────────────────────────────
    def __init__(self, index=0, port_name=None):
        if isinstance(index, str):
            port_name = index
            index = 0
        if index in CheeseStick._robots:
            robot = CheeseStick._robots[index]
            if robot: robot.dispose()
        CheeseStick._robots[index] = self
        super(CheeseStick, self).__init__(CheeseStick.ID, "CheeseStick", index)

        self._init(port_name)

    def dispose(self):
        CheeseStick._robots[self.get_index()] = None
        self._roboid._dispose()
        Runner.unregister_robot(self)

    def reset(self):
        self._roboid._reset()

    def _init(self, port_name):
        from robomation.roboids.cheesestick_roboid import CheeseStickRoboid
        self._roboid = CheeseStickRoboid(self.get_index())
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
    def _evaluate_sound(self):
        return self.e(CheeseStick.SOUND_STATE)
    def _evaluate_m_step(self):
        return self.e(CheeseStick.M_STEP_STATE)

    # ── Internal helpers ──────────────────────────────────────────────────────

    @staticmethod
    def _analog_to_pwm(value):
        # 0-100 analog → 0-255 PWM byte
        if value < 0: value = 0
        if value > 100: value = 100
        return math.floor(value * 255 / 100 + 0.5)

    @staticmethod
    def _pwm_to_analog(value):
        # 0-255 PWM byte → 0-100 analog
        if value < 0: value = 0
        if value > 255: value = 255
        return math.floor(value * 100 / 255)

    def _set_io_range(self, unit, src_min, src_median, src_max, dst_min, dst_median, dst_max):
        if unit not in CheeseStick._SRC_RANGE_DEVICE_IDS:
            return
        self.write(CheeseStick._SRC_RANGE_DEVICE_IDS[unit], [src_min, src_median, src_max])
        self.write(CheeseStick._DST_RANGE_DEVICE_IDS[unit], [dst_min, dst_median, dst_max])

    def _stop_sound(self):
        for dev in (CheeseStick.SOUND_BUZZ, CheeseStick.SOUND_NOTE, CheeseStick.SOUND_CLIP):
            if self.read(dev) > 0:
                self.write(dev, 0)

    # ── Extension module factories ──────────────────────────────────────────
    def CSD01(self):    return _CSD01(parent=self)
    def CSD02(self):    return _CSD02(parent=self)
    def CSD03(self):    return _CSD03(parent=self)
    def CSD07(self):    return _CSD07(parent=self)
    def CSD09(self):    return _CSD09(parent=self)
    def CSD10(self):    return _CSD10(parent=self)
    def HAT010(self):   return _HAT010(parent=self)
    def HAT022(self):   return _HAT022(parent=self)
    # def PID10(self):    return _PID10(parent=self)
    def PID13(self):    return _PID13(parent=self)
    def PID26(self):    return _PID26(parent=self)
    def NeoPixel(self): return _NeoPixel(parent=self)

    # ── Port I/O ──────────────────────────────────────────────────────────────
    def set_input_mode(self, unit: _InputUnit, option: _InputMode):
        if unit not in CheeseStick._VALID_INPUT_UNITS:
            return _err(CheeseStick, 'set_input_mode', 'unit', unit, CheeseStick._VALID_INPUT_UNITS)
        if option not in CheeseStick._VALID_INPUT_MODES:
            return _err(CheeseStick, 'set_input_mode', 'option', option, CheeseStick._VALID_INPUT_MODES)
        wire = CheeseStick._VALID_INPUT_MODES[option]
        self.write(CheeseStick._MODE_DEVICE_IDS[unit], wire)

        # digital_pullup / digital_pulldown 의 경우 PULL 도 동시 설정
        if option == 'digital_pullup':
            self.write(CheeseStick._PULL_DEVICE_IDS[unit], CheeseStick._VALID_PULSE_OPTS['pull-up'])
        elif option == 'digital_pulldown':
            self.write(CheeseStick._PULL_DEVICE_IDS[unit], CheeseStick._VALID_PULSE_OPTS['pull-down'])
        elif option == 'analog_voltage':  # analog_voltage 의 경우 BGV 도 동시 설정
            self.write(CheeseStick._BGV_DEVICE_IDS[unit], CheeseStick._VALID_BGV_OPTS['voltage'])

    def set_input_range(self, unit: _InputUnit, 
                        src_min: Union[int, float], src_max: Union[int, float], 
                        dst_min: Union[int, float], dst_max: Union[int, float]):
        if unit not in CheeseStick._VALID_INPUT_UNITS:
            return _err(CheeseStick, 'set_input_range', 'unit', unit, CheeseStick._VALID_INPUT_UNITS)
        self._set_io_range(unit, src_min, None, src_max, dst_min, None, dst_max)

    def set_input_range_median(self, unit: _InputUnit, 
                                src_min: Union[int, float], src_median: Union[int, float], src_max: Union[int, float], 
                                dst_min: Union[int, float], dst_median: Union[int, float], dst_max: Union[int, float]):
        if unit not in CheeseStick._VALID_INPUT_UNITS:
            return _err(CheeseStick, 'set_input_range_median', 'unit', unit, CheeseStick._VALID_INPUT_UNITS)
        self._set_io_range(unit, src_min, src_median, src_max, dst_min, dst_median, dst_max)

    def get_input(self, unit: _InputUnit) -> Union[int, float]:
        if unit not in CheeseStick._VALID_INPUT_UNITS:
            return _err(CheeseStick, 'get_input', 'unit', unit, CheeseStick._VALID_INPUT_UNITS)
        return self.read(CheeseStick._INPUT_DEVICE_IDS[unit])

    def set_pulse_input_mode(self, unit: _PulseInputUnit, option: _PulseOpt):
        if unit not in CheeseStick._VALID_PULSE_INPUT_UNITS:
            return _err(CheeseStick, 'set_pulse_input_mode', 'unit', unit, CheeseStick._VALID_PULSE_INPUT_UNITS)
        if option not in CheeseStick._VALID_PULSE_OPTS:
            return _err(CheeseStick, 'set_pulse_input_mode', 'option', option, CheeseStick._VALID_PULSE_OPTS)
        self.write(CheeseStick._PULL_DEVICE_IDS[unit], CheeseStick._VALID_PULSE_OPTS[option])

    def get_pulse_input(self, unit: _PulseInputUnit) -> bool:
        if unit not in CheeseStick._VALID_PULSE_INPUT_UNITS:
            return _err(CheeseStick, 'get_pulse_input', 'unit', unit, CheeseStick._VALID_PULSE_INPUT_UNITS)
        return self.e(CheeseStick._PULSE_INPUT_DETECT_DEVICE_IDS[unit])

    def set_digital_output(self, unit: _DigitalOutputUnit, value: int):
        if unit not in CheeseStick._VALID_DIGITAL_OUTPUT_UNITS:
            return _err(CheeseStick, 'set_digital_output', 'unit', unit, CheeseStick._VALID_DIGITAL_OUTPUT_UNITS)
        if not isinstance(value, (int, float)):
            return _err(CheeseStick, 'set_digital_output', 'value', value, 'int | float')
        self.write(CheeseStick._MODE_DEVICE_IDS[unit], CheeseStick._VALID_OUTPUT_MODES['digital'])
        self.write(CheeseStick._DIGITAL_OUT_DEVICE_IDS[unit], value)

    def set_pwm_output(self, unit: _PwmOutputUnit, value: Union[int, float]):
        if unit not in CheeseStick._VALID_PWM_OUTPUT_UNITS:
            return _err(CheeseStick, 'set_pwm_output', 'unit', unit, CheeseStick._VALID_PWM_OUTPUT_UNITS)
        if not isinstance(value, (int, float)):
            return _err(CheeseStick, 'set_pwm_output', 'value', value, 'int | float')
        self.write(CheeseStick._MODE_DEVICE_IDS[unit], CheeseStick._VALID_OUTPUT_MODES['pwm'])
        self.write(CheeseStick._PWM_OUT_DEVICE_IDS[unit], value)

    def change_pwm_output(self, unit: _PwmOutputUnit, value: Union[int, float]):
        if unit not in CheeseStick._VALID_PWM_OUTPUT_UNITS:
            return _err(CheeseStick, 'change_pwm_output', 'unit', unit, CheeseStick._VALID_PWM_OUTPUT_UNITS)
        if not isinstance(value, (int, float)):
            return _err(CheeseStick, 'change_pwm_output', 'value', value, 'int | float')
        self.write(CheeseStick._MODE_DEVICE_IDS[unit], CheeseStick._VALID_OUTPUT_MODES['pwm'])
        dev = CheeseStick._PWM_OUT_DEVICE_IDS[unit]
        self.write(dev, self.read(dev) + value)

    # ── Sound ─────────────────────────────────────────────────────────────────
    def sound_buzz(self, hz: Union[int, float]):
        if not isinstance(hz, (int, float)):
            return _err(CheeseStick, 'sound_buzz', 'hz', hz, 'int | float')
        self.write(CheeseStick.SOUND_BUZZ, hz)

    def sound_note(self, note: _Note, octave: int = 4):
        if note not in CheeseStick._VALID_NOTES:
            return _err(CheeseStick, 'sound_note', 'note', note, tuple(CheeseStick._VALID_NOTES))
        if not isinstance(octave, int) or not (1 <= octave <= 7):
            return _err(CheeseStick, 'sound_note', 'octave', octave, 'int (1~7)')
        self.write(CheeseStick.SOUND_NOTE, (octave - 1) * 12 + CheeseStick._VALID_NOTES[note] + 4)

    def _sound_clip_impl(self, clip):
        self.write(CheeseStick.SOUND_CLIP, CheeseStick._VALID_CLIPS[clip])
        Runner.wait_until(self._evaluate_sound)

    def sound_clip(self, clip: _Clip, wait: bool = True):
        if clip not in CheeseStick._VALID_CLIPS:
            return _err(CheeseStick, 'sound_clip', 'clip', clip, tuple(CheeseStick._VALID_CLIPS))
        Runner.dispatch(lambda: self._sound_clip_impl(clip), wait)

    def sound_off(self):
        self._stop_sound()

    # ── Sensors ───────────────────────────────────────────────────────────────
    def acceleration(self, unit: _Axis) -> Union[int, float]:
        if unit not in CheeseStick._VALID_AXIS:
            return _err(CheeseStick, 'acceleration', 'unit', unit, CheeseStick._VALID_AXIS)
        return self.read({
            'x': CheeseStick.ACCELERATION_X,
            'y': CheeseStick.ACCELERATION_Y,
            'z': CheeseStick.ACCELERATION_Z,
        }[unit])

    def tap(self) -> bool:
        return self.e(CheeseStick.TAP_STATE)

    def fall(self) -> bool:
        return self.e(CheeseStick.FALL_STATE)

    def temperature(self) -> Union[int, float]:
        return self.read(CheeseStick.TEMPERATURE)

    def signal_strength(self) -> Union[int, float]:
        return self.read(CheeseStick.SIGNAL_STRENGTH)

    def battery(self) -> Union[int, float]:
        return self.read(CheeseStick.BATTERY)
