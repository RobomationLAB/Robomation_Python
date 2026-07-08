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

import math
from typing import Literal, Optional, Union, get_args

from robomation.core.error import _err
# from robomation.core.utils import Utils
from robomation.core.model import Robot
from robomation.core.runner import Runner


# ── Module-level Literal aliases (for IDE auto-complete) ─────────────────────
_Mode          = Literal['speed', 'angle']
_JointId       = Literal[1, 2, 3, 4, -1]
_XyzOrigin     = Literal['wrist', 'end_effector']
_XyzPos        = Literal['x', 'y', 'z']
_DistUnit      = Literal['cm', 'mm', 'inch']
_AnglePos      = Literal['zero', 'park', 'home']
_LockMode      = Literal['none', 'horizontal', 'vertical']
_Note          = Literal['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B']
_Clip          = Literal['mute', 'beep', 'beep2', 'beep3', 'beep_repeat', 'beep_random', 'beep_random_repeat',
                            'noise', 'noise_repeat', 'siren', 'siren_repeat', 'engine', 'engine_repeat',
                            'fart_a', 'fart_b', 'noise_random', 'noise_random_repeat', 'whistle', 'chop', 'chop_repeat',
                            'random_melody', 'robot', 'connect', 'dibidibidip', 'good_job', 'wake_up', 'start', 'bye']
_Button        = Literal['teach', 'play', 'power', 'delete', 'any']
_ButtonEvent   = Literal['pressed', 'click', 'long_click']
_Peripheral    = Literal['none', 'conveyor', 'slider']
_ConveyorMode  = Literal['speed', 'distance']


class RaccoonBot(Robot):
    ID = "kr.robomation.roboids.raccoonbot"
    _robots = {}

    # ── Device IDs (RaccoonBot 펌웨어 디바이스 식별자) ─────────────────────────
    # Effectors (0x030000xx)
    MODE                    = 0x03000000
    MOTOR_OFF               = 0x03000001
    SPEED                   = 0x03000002
    ANGLE_MAX_SPEED         = 0x03000003
    END_EFFECTOR_LOCK       = 0x03000004
    END_EFFECTOR_CONTROL    = 0x03000005
    CONVEYOR_MODE           = 0x03000006
    CONVEYOR_SPEED          = 0x03000007
    PERIPHERAL              = 0x03000008

    # Commands (0x030001xx)
    ANGLE_J1                = 0x03000100
    ANGLE_J2                = 0x03000101
    ANGLE_J3                = 0x03000102
    ANGLE_J4                = 0x03000103
    SOUND_NOTE              = 0x03000104
    SOUND_CLIP              = 0x03000105
    CONVEYOR_DISTANCE       = 0x03000106

    # Sensors (0x030002xx)
    ENCODER_J1              = 0x03000200
    ENCODER_J2              = 0x03000201
    ENCODER_J3              = 0x03000202
    ENCODER_J4              = 0x03000203
    END_EFFECTOR_DEVICE     = 0x03000204
    END_EFFECTOR_STATE      = 0x03000205
    TEACH_PRESSED           = 0x03000206
    PLAY_PRESSED            = 0x03000207
    POWER_PRESSED           = 0x03000208
    DELETE_PRESSED          = 0x03000209
    WARNING                 = 0x0300020a
    SIGNAL_STRENGTH         = 0x0300020b
    BATTERY                 = 0x0300020c
    CONVEYOR_COUNT          = 0x0300020d
    CONVEYOR_DIRECTION      = 0x0300020e
    CONVEYOR_RUNNING        = 0x0300020f
    CONVEYOR_BUTTON_PRESSED = 0x03000210

    # Events (0x030003xx)
    TEACH_CLICK             = 0x03000300
    PLAY_CLICK              = 0x03000301
    POWER_CLICK             = 0x03000302
    DELETE_CLICK            = 0x03000303
    TEACH_LONG_CLICK        = 0x03000304
    PLAY_LONG_CLICK         = 0x03000305
    DELETE_LONG_CLICK       = 0x03000306
    CONVEYOR_BUTTON_CLICK   = 0x03000307
    CONVEYOR_BUTTON_LONG_CLICK = 0x03000308

    ANGLE_STATE             = 0x03000309
    SOUND_STATE             = 0x0300030a
    CONVEYOR_STATE          = 0x0300030b

    # ── Robot-specific constants (from CONST.RACCOON4 in player_codes.js) ────
    _CONVEYOR_CM_TO_PULSE = 145.45
    # _SLIDER_CM_TO_PULSE = 333.3

    # ── Valid values (derived from module-level Literals) ────────────────────
    _VALID_MODES                = {n: i for i, n in enumerate(get_args(_Mode))}
    _VALID_JOINTS               = get_args(_JointId)
    _VALID_XYZ_ORIGIN           = get_args(_XyzOrigin)
    _VALID_XYZ_POS              = {n: i for i, n in enumerate(get_args(_XyzPos))}
    _VALID_DIST_UNITS           = get_args(_DistUnit)
    _VALID_ANGLE_POS            = get_args(_AnglePos)
    _VALID_LOCK_MODES           = {n: i for i, n in enumerate(get_args(_LockMode))}
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
        'random_melody': 20,      'robot': 21,              'connect': 22,            
        'dibidibidip': 33,        'good_job': 34,           
        'wake_up': 35,            'start': 36,              'bye': 37
    }
    _VALID_BUTTONS              = get_args(_Button)
    _VALID_BUTTON_EVENTS        = get_args(_ButtonEvent)
    _VALID_PERIPHERALS          = {n: i for i, n in enumerate(get_args(_Peripheral))}
    _VALID_CONVEYOR_MODES       = {n: i for i, n in enumerate(get_args(_ConveyorMode))}
    _VALID_CONTROL              = {'place': 0, 'pick': 1}
    _DEFAULT_ANGLE              = {
        'zero': [0, 0, 0, 0],
        'park': [0, 25.0, -145.0, -60.0],
        'home': [0, -10.0, -140.0, 60.0],
    }

    # ── 버튼 unit/event → device id 매핑 ──────────────────────────────────────
    # device ID 상수가 위에서 이미 정의됐기 때문.
    _BUTTON_PRESSED_IDS = {
        'teach':  TEACH_PRESSED,
        'play':   PLAY_PRESSED,
        'power':  POWER_PRESSED,
        'delete': DELETE_PRESSED,
    }
    _BUTTON_CLICK_IDS = {
        'teach':  TEACH_CLICK,
        'play':   PLAY_CLICK,
        'power':  POWER_CLICK,
        'delete': DELETE_CLICK,
    }
    _BUTTON_LONG_CLICK_IDS = {
        'teach':  TEACH_LONG_CLICK,
        'play':   PLAY_LONG_CLICK,
        'delete': DELETE_LONG_CLICK,
        # 'power' — 카탈로그에 LONG_CLICK 미정의
    }

    # IK/FK 상수 (player_codes.js 의 joint 구조 그대로)
    # arm.l4 = END_EFFECTOR_DEVICE 값별 l4 geometry dict.
    #   key 0 = none (장착 안됨 / 미감지 / origin='wrist'). length=0, zOffset=0.
    #   key 1~4 = 장착된 end-effector 종류 (gripper / vacuum_gripper / servo_gripper / dc_gripper)
    _JOINT = {
        'arm': {
            'l1': 8.25,
            'l2': 10.0,
            'l3': 10.0,
            'l4': {
                0: {'length': 0,   'zOffset': 0},      # none
                1: {'length': 8,   'zOffset': -0.6},   # gripper
                2: {'length': 7.5, 'zOffset':  0  },   # vacuum_gripper
                3: {'length': 8,   'zOffset': -0.6},   # servo_gripper
                4: {'length': 8,   'zOffset': -0.6},   # dc_gripper
            },
        },
        'limits': {
            'J1': {'min': -120, 'max': 120},
            'J2': {'min':  -90, 'max':  30},
            'J3': {'min': -150, 'max':   0},
            'J4': {'min': -105, 'max': 105},
        },
    }

    # ── Robot lifecycle ───────────────────────────────────────────────────────
    def __init__(self, index=0, port_name=None):
        if isinstance(index, str):
            port_name = index
            index = 0
        if index in RaccoonBot._robots:
            robot = RaccoonBot._robots[index]
            if robot: robot.dispose()
        RaccoonBot._robots[index] = self
        super(RaccoonBot, self).__init__(RaccoonBot.ID, "RaccoonBot", index)

        # Conveyor-speed snapshot 상태 (hamster_s wheel 패턴과 동일 — flag 기반)
        # catalog CONVEYOR_SPEED range = -100~100 이라 -128 sentinel 못 박음 → 별도 플래그로 set 추적.
        self._conveyor_set = False
        self._saved_conveyor = None    # (set_bool, val) snapshot or None
        self._wrote_conveyor = None    # val (mover 가 쓴 값)

        self._init(port_name)

    def dispose(self):
        RaccoonBot._robots[self.get_index()] = None
        self._roboid._dispose()
        Runner.unregister_robot(self)

    def reset(self):
        self._roboid._reset()

    def _init(self, port_name):
        from robomation.roboids.raccoonbot_roboid import RaccoonBotRoboid
        self._roboid = RaccoonBotRoboid(self.get_index())
        self._add_roboid(self._roboid)
        Runner.register_robot(self)
        Runner.start()
        self._roboid._init(port_name)

    def find_device_by_id(self, device_id):
        return self._roboid.find_device_by_id(device_id)

    # ── 완료 감지 evaluator (Runner.wait_until 용) ────────────────────────────
    def _evaluate_angle_state(self):
        return self.e(RaccoonBot.ANGLE_STATE)

    def _evaluate_sound(self):
        return self.e(RaccoonBot.SOUND_STATE)

    def _evaluate_conveyor(self):
        return self.e(RaccoonBot.CONVEYOR_STATE)

    # ── Joint / Encoder id 매핑 헬퍼 ──────────────────────────────────────────
    @staticmethod
    def _angle_id(joint):
        return (RaccoonBot.ANGLE_J1, RaccoonBot.ANGLE_J2, RaccoonBot.ANGLE_J3, RaccoonBot.ANGLE_J4)[joint - 1]

    @staticmethod
    def _encoder_id(joint):
        return (RaccoonBot.ENCODER_J1, RaccoonBot.ENCODER_J2, RaccoonBot.ENCODER_J3, RaccoonBot.ENCODER_J4)[joint - 1]

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _stop_sound(self):
        if self.read(RaccoonBot.SOUND_NOTE) > 0:
            self.write(RaccoonBot.SOUND_NOTE, 0)
        if self.read(RaccoonBot.SOUND_CLIP) > 0:
            self.write(RaccoonBot.SOUND_CLIP, 0)

    # ── Conveyor-speed snapshot helpers ──────────────────────────────────────
    # 사용자가 컨베이어 속도 미설정 → 기본 속도 100 으로 이동, 완료 후 미설정 상태로 복원.
    # 사용자가 설정 후 이동 → 그 속도 유지. 정지 후 재이동 시 직전 패턴 유지.

    def _begin_conveyor_speed(self):
        if self._saved_conveyor is not None:
            cur = self.read(RaccoonBot.CONVEYOR_SPEED)
            if self._wrote_conveyor is not None and cur == self._wrote_conveyor:
                saved_set, saved_val = self._saved_conveyor
                self.write(RaccoonBot.CONVEYOR_SPEED, saved_val)
                self._conveyor_set = saved_set
            self._saved_conveyor = None
            self._wrote_conveyor = None
        self._saved_conveyor = (self._conveyor_set, self.read(RaccoonBot.CONVEYOR_SPEED))

    def _mark_conveyor_speed(self):
        self._wrote_conveyor = self.read(RaccoonBot.CONVEYOR_SPEED)

    def _restore_conveyor_speed(self):
        if self._saved_conveyor is not None:
            saved_set, saved_val = self._saved_conveyor
            self.write(RaccoonBot.CONVEYOR_SPEED, saved_val)
            self._conveyor_set = saved_set
            self._saved_conveyor = None
            self._wrote_conveyor = None

    # ── IK/FK helpers (ported from player_codes.js __angles_to_xyz / __xyz_to_angles) ─
    
    @staticmethod
    def _to_rad(n): 
        return n / 180 * math.pi

    @staticmethod
    def _to_deg(n): 
        return n / math.pi * 180
    
    @staticmethod
    def _coord_to_cm(value, unit):
        if unit == 'mm': return value * 0.1
        if unit == 'inch': return value * 2.54
        return value  # 'cm'
    
    @staticmethod
    def _check_angles_limit(J1, J2, J3, J4):
        limits = RaccoonBot._JOINT['limits']
        for ang, key in zip((J1, J2, J3, J4), ('J1', 'J2', 'J3', 'J4')):
            lim = limits[key]
            if ang < lim['min'] or ang > lim['max']:
                return False
        return True

    def _arm(self, origin):
        # origin='wrist' → l4[0] (length=0, zOffset=0) 로 wrist 까지의 운동학만 계산.
        # origin='end_effector' → END_EFFECTOR_DEVICE 가 가리키는 l4[device] (미장착/알수없음이면 l4[0]).
        arm = RaccoonBot._JOINT['arm']
        if origin == 'wrist':
            l4 = arm['l4'][0]
        else:
            try:
                device = int(self.read(RaccoonBot.END_EFFECTOR_DEVICE))
            except (TypeError, ValueError):
                device = 0
            l4 = arm['l4'].get(device, arm['l4'][0])
        return {
            'l1': arm['l1'],
            'l2': arm['l2'],
            'l3': arm['l3'],
            'l4': l4,
        }

    def _angles_to_xyz(self, angles, origin):
        arm = self._arm(origin)
        J1 = angles[0]
        J2 = angles[1] + 90
        J3 = angles[2]
        J4 = angles[3]
        l1, l2, l3, l4 = arm['l1'], arm['l2'], arm['l3'], arm['l4']
        l4_len = l4['length']
        l4_zoff = l4['zOffset']
        to_rad = RaccoonBot._to_rad
        w = (l2 * math.cos(to_rad(J2))
             + l3 * math.cos(to_rad(J2 + J3))
             + l4_len * math.cos(to_rad(J2 + J3 + J4))
             - l4_zoff * math.sin(to_rad(J2 + J3 + J4)))
        x = w * math.cos(to_rad(J1))
        y = w * math.sin(to_rad(J1))
        z = (l1
             + l2 * math.sin(to_rad(J2))
             + l3 * math.sin(to_rad(J2 + J3))
             + l4_len * math.sin(to_rad(J2 + J3 + J4))
             + l4_zoff * math.cos(to_rad(J2 + J3 + J4)))
        return [round(-y, 3), round(x, 3), round(z, 3)]

    def _xyz_to_angles(self, coordinate, origin):
        """좌표 → 4개 joint 각도. 도달 불가/범위 밖이면 None 반환."""
        arm = self._arm(origin)
        l1, l2, l3 = arm['l1'], arm['l2'], arm['l3']
        l4_len = arm['l4']['length']
        l4_zoff = arm['l4']['zOffset']
        x = coordinate[1]
        y = -coordinate[0]
        z = coordinate[2]
        to_rad = RaccoonBot._to_rad
        to_deg = RaccoonBot._to_deg
        # joint 1
        J1 = to_deg(math.atan2(y, x))
        # joint 3
        lock = self.read(RaccoonBot.END_EFFECTOR_LOCK)
        vertical = RaccoonBot._VALID_LOCK_MODES['vertical']
        theta = -180 if int(lock) == vertical else -90
        w = (math.sqrt(x * x + y * y)
             - l4_len * math.cos(to_rad(theta + 90))
             + l4_zoff * math.sin(to_rad(theta + 90)))
        h = (z - l1
             - l4_len * math.sin(to_rad(theta + 90))
             - l4_zoff * math.cos(to_rad(theta + 90)))
        D = (w * w + h * h - l2 * l2 - l3 * l3) / (2 * l2 * l3)
        if D < -1 or D > 1:
            return None  # 도달 불가 (기하학적 reach 한계 초과)
        cos3 = D
        sin3 = -math.sqrt(1 - cos3 * cos3)
        J3 = to_deg(math.atan2(sin3, cos3))
        # joint 2
        alpha = math.atan2(h, w)
        beta = math.atan2(l3 * math.sin(to_rad(J3)), l2 + l3 * math.cos(to_rad(J3)))
        J2 = to_deg(alpha - beta) - 90
        # joint 4
        j4_lim = RaccoonBot._JOINT['limits']['J4']
        J4 = max(min(theta - (J2 + J3), j4_lim['max']), j4_lim['min'])
        if not self._check_angles_limit(J1, J2, J3, J4):
            return None  # joint 한도 위반 (J1/J2/J3)
        return [J1, J2, J3, J4]

    def _current_xyz(self, origin):
        encoders = [
            self.read(RaccoonBot.ENCODER_J1),
            self.read(RaccoonBot.ENCODER_J2),
            self.read(RaccoonBot.ENCODER_J3),
            self.read(RaccoonBot.ENCODER_J4),
        ]
        return self._angles_to_xyz(encoders, origin)

    # ── Mode ──────────────────────────────────────────────────────────────────
    
    # def mode(self, mode: _Mode):
    #     if mode not in RaccoonBot._VALID_MODES:
    #         return _err(RaccoonBot, 'mode', 'mode', mode, tuple(RaccoonBot._VALID_MODES))
    #     self.write(RaccoonBot.MODE, RaccoonBot._VALID_MODES[mode])

    def motor(self, unit: _JointId, on: bool = True):
        if unit not in RaccoonBot._VALID_JOINTS:
            return _err(RaccoonBot, 'motor', 'unit', unit, RaccoonBot._VALID_JOINTS)
        if not isinstance(on, bool):
            return _err(RaccoonBot, 'motor', 'on', on, 'bool')
        val = 0 if on else 1   # MOTOR_OFF: 1 = off, 0 = on
        if unit == -1:
            self.write(RaccoonBot.MOTOR_OFF, [val, val, val, val])
        else:
            self.write(RaccoonBot.MOTOR_OFF, unit - 1, val)

    # ── Speed ─────────────────────────────────────────────────────────────────
    def set_speed_joint(self, joint: _JointId, data: Union[int, float]):
        # if not self._require_mode('set_speed_joint', 'speed'):
        #     return
        if joint not in RaccoonBot._VALID_JOINTS:
            return _err(RaccoonBot, 'set_speed_joint', 'joint', joint, RaccoonBot._VALID_JOINTS)
        if not isinstance(data, (int, float)):
            return _err(RaccoonBot, 'set_speed_joint', 'data', data, 'int | float')
        self.write(RaccoonBot.MODE, RaccoonBot._VALID_MODES['speed'])
        if joint == -1:
            self.write(RaccoonBot.SPEED, [data, data, data, data])
        else:
            self.write(RaccoonBot.SPEED, joint - 1, data)

    def change_speed_joint(self, joint: _JointId, data: Union[int, float]):
        # if not self._require_mode('change_speed_joint', 'speed'):
        #     return
        if joint not in RaccoonBot._VALID_JOINTS:
            return _err(RaccoonBot, 'change_speed_joint', 'joint', joint, RaccoonBot._VALID_JOINTS)
        if not isinstance(data, (int, float)):
            return _err(RaccoonBot, 'change_speed_joint', 'data', data, 'int | float')
        self.write(RaccoonBot.MODE, RaccoonBot._VALID_MODES['speed'])
        if joint == -1:
            for i in range(4):
                self.write(RaccoonBot.SPEED, i, self.read(RaccoonBot.SPEED, i) + data)
        else:
            i = joint - 1
            self.write(RaccoonBot.SPEED, i, self.read(RaccoonBot.SPEED, i) + data)

    def set_speed_joints(self, j1: Union[int, float], j2: Union[int, float], j3: Union[int, float], j4: Union[int, float]):
        # if not self._require_mode('set_speed_joints', 'speed'):
        #     return
        for name, val in (('j1', j1), ('j2', j2), ('j3', j3), ('j4', j4)):
            if not isinstance(val, (int, float)):
                return _err(RaccoonBot, 'set_speed_joints', name, val, 'int | float')
        self.write(RaccoonBot.MODE, RaccoonBot._VALID_MODES['speed'])
        self.write(RaccoonBot.SPEED, [j1, j2, j3, j4])

    # ── Angle ─────────────────────────────────────────────────────────────────
    def angle_max_speed(self, data: Union[int, float]):
        if not isinstance(data, (int, float)):
            return _err(RaccoonBot, 'angle_max_speed', 'data', data, 'int | float')
        self.write(RaccoonBot.ANGLE_MAX_SPEED, data)

    def _set_angle_joint_impl(self, joint, data):
        
        self.write(RaccoonBot.MODE, RaccoonBot._VALID_MODES['angle'])
        if joint == -1:
            self.write(RaccoonBot.ANGLE_J1, data)
            self.write(RaccoonBot.ANGLE_J2, data)
            self.write(RaccoonBot.ANGLE_J3, data)
            self.write(RaccoonBot.ANGLE_J4, data)
        else:
            self.write(RaccoonBot._angle_id(joint), data)
        Runner.wait_until(self._evaluate_angle_state)

    def set_angle_joint(self, joint: _JointId, data: Union[int, float], wait: bool = True):
        # if not self._require_mode('set_angle_joint', 'angle'):
        #     return
        if joint not in RaccoonBot._VALID_JOINTS:
            return _err(RaccoonBot, 'set_angle_joint', 'joint', joint, RaccoonBot._VALID_JOINTS)
        if not isinstance(data, (int, float)):
            return _err(RaccoonBot, 'set_angle_joint', 'data', data, 'int | float')
        Runner.dispatch(lambda: self._set_angle_joint_impl(joint, data), wait)

    def _change_angle_joint_impl(self, joint, data):
        self.write(RaccoonBot.MODE, RaccoonBot._VALID_MODES['angle'])
        if joint == -1:
            for i in (1, 2, 3, 4):
                dev = RaccoonBot._angle_id(i)
                self.write(dev, self.read(dev) + data)
        else:
            dev = RaccoonBot._angle_id(joint)
            self.write(dev, self.read(dev) + data)
        Runner.wait_until(self._evaluate_angle_state)

    def change_angle_joint(self, joint: _JointId, data: Union[int, float], wait: bool = True):
        # if not self._require_mode('change_angle_joint', 'angle'):
        #     return
        if joint not in RaccoonBot._VALID_JOINTS:
            return _err(RaccoonBot, 'change_angle_joint', 'joint', joint, RaccoonBot._VALID_JOINTS)
        if not isinstance(data, (int, float)):
            return _err(RaccoonBot, 'change_angle_joint', 'data', data, 'int | float')
        Runner.dispatch(lambda: self._change_angle_joint_impl(joint, data), wait)

    def _set_angle_joints_impl(self, j1, j2, j3, j4):
        self.write(RaccoonBot.MODE, RaccoonBot._VALID_MODES['angle'])
        self.write(RaccoonBot.ANGLE_J1, j1)
        self.write(RaccoonBot.ANGLE_J2, j2)
        self.write(RaccoonBot.ANGLE_J3, j3)
        self.write(RaccoonBot.ANGLE_J4, j4)
        Runner.wait_until(self._evaluate_angle_state)

    def set_angle_joints(self, j1: Union[_AnglePos, int, float], j2: Union[int, float, None] = None, j3: Union[int, float, None] = None, j4: Union[int, float, None] = None, wait: bool = True):
        # if not self._require_mode('set_angle_joints', 'angle'):
        #     return
        # Case 1: preset name ('home', 'park', 'zero') — j2/j3/j4 무시
        if isinstance(j1, str):
            if j1 not in RaccoonBot._VALID_ANGLE_POS:
                return _err(RaccoonBot, 'set_angle_joints', 'j1', j1, RaccoonBot._VALID_ANGLE_POS)
            default = RaccoonBot._DEFAULT_ANGLE.get(j1, [0, 0, 0, 0])
            Runner.dispatch(lambda: self._set_angle_joints_impl(*default), wait)
            return
        # Case 2: 4 numeric joint angles
        for name, val in (('j1', j1), ('j2', j2), ('j3', j3), ('j4', j4)):
            if not isinstance(val, (int, float)):
                return _err(RaccoonBot, 'set_angle_joints', name, val, 'int | float')
        Runner.dispatch(lambda: self._set_angle_joints_impl(j1, j2, j3, j4), wait)

    # def _move_xyz_impl(self, x, y, z, origin):
    #     self.write(RaccoonBot.MODE, RaccoonBot._VALID_MODES['angle'])
    #     angles = self._xyz_to_angles([x, y, z], origin)
    #     self.write(RaccoonBot.ANGLE_J1, angles[0])
    #     self.write(RaccoonBot.ANGLE_J2, angles[1])
    #     self.write(RaccoonBot.ANGLE_J3, angles[2])
    #     self.write(RaccoonBot.ANGLE_J4, angles[3])
    #     Runner.wait_until(self._evaluate_angle_state)

    # def move_xyz(self, origin: _XyzOrigin = 'wrist', x: Union[int, float, None] = None, y: Union[int, float, None] = None, z: Union[int, float, None] = None, wait: bool = True):
    #     if origin not in RaccoonBot._VALID_XYZ_ORIGIN:
    #         return _err(RaccoonBot, 'move_xyz', 'origin', origin, RaccoonBot._VALID_XYZ_ORIGIN)
    #     for name, val in (('x', x), ('y', y), ('z', z)):
    #         if not isinstance(val, (int, float)):
    #             return _err(RaccoonBot, 'move_xyz', name, val, 'int | float')
    #     Runner.dispatch(lambda: self._move_xyz_impl(x*0.1, y*0.1, z*0.1, origin), wait)

    # ── Coordinate set / change (per-axis) ────────────────────────────────────
    # 좌표→IK 결과 angle 이 _JOINT['limits'] 또는 reach 범위를 벗어나면 _err 출력 후 동작 스킵.
    def set_coordinate(self, origin: _XyzOrigin, pos: _XyzPos, data: Union[int, float], unit: _DistUnit = 'cm', wait: bool = True):
        # if not self._require_mode('set_coordinate', 'angle'):
        #     return
        if origin not in RaccoonBot._VALID_XYZ_ORIGIN:
            return _err(RaccoonBot, 'set_coordinate', 'origin', origin, RaccoonBot._VALID_XYZ_ORIGIN)
        if pos not in RaccoonBot._VALID_XYZ_POS:
            return _err(RaccoonBot, 'set_coordinate', 'pos', pos, tuple(RaccoonBot._VALID_XYZ_POS))
        if not isinstance(data, (int, float)):
            return _err(RaccoonBot, 'set_coordinate', 'data', data, 'int | float')
        if unit not in RaccoonBot._VALID_DIST_UNITS:
            return _err(RaccoonBot, 'set_coordinate', 'unit', unit, RaccoonBot._VALID_DIST_UNITS)
        cm = RaccoonBot._coord_to_cm(data, unit)
        pos_idx = RaccoonBot._VALID_XYZ_POS[pos]
        xyz = self._current_xyz(origin)
        new_xyz = [cm if i == pos_idx else v for i, v in enumerate(xyz)]
        angles = self._xyz_to_angles(new_xyz, origin)
        if angles is None:
            return _err(RaccoonBot, 'set_coordinate', '(x, y, z)', new_xyz, 'coordinates within the workspace')
        Runner.dispatch(lambda: self._set_angle_joints_impl(*angles), wait)

    def change_coordinate(self, origin: _XyzOrigin, pos: _XyzPos, data: Union[int, float], unit: _DistUnit = 'cm', wait: bool = True):
        # if not self._require_mode('change_coordinate', 'angle'):
        #     return
        if origin not in RaccoonBot._VALID_XYZ_ORIGIN:
            return _err(RaccoonBot, 'change_coordinate', 'origin', origin, RaccoonBot._VALID_XYZ_ORIGIN)
        if pos not in RaccoonBot._VALID_XYZ_POS:
            return _err(RaccoonBot, 'change_coordinate', 'pos', pos, tuple(RaccoonBot._VALID_XYZ_POS))
        if not isinstance(data, (int, float)):
            return _err(RaccoonBot, 'change_coordinate', 'data', data, 'int | float')
        if unit not in RaccoonBot._VALID_DIST_UNITS:
            return _err(RaccoonBot, 'change_coordinate', 'unit', unit, RaccoonBot._VALID_DIST_UNITS)
        cm = RaccoonBot._coord_to_cm(data, unit)
        pos_idx = RaccoonBot._VALID_XYZ_POS[pos]
        xyz = self._current_xyz(origin)
        new_xyz = [v + cm if i == pos_idx else v for i, v in enumerate(xyz)]
        angles = self._xyz_to_angles(new_xyz, origin)
        if angles is None:
            return _err(RaccoonBot, 'change_coordinate', '(x, y, z)', new_xyz, 'coordinates within the workspace')
        Runner.dispatch(lambda: self._set_angle_joints_impl(*angles), wait)

    def set_coordinates(self, origin: _XyzOrigin, x: Union[int, float], y: Union[int, float], z: Union[int, float], unit: _DistUnit = 'cm', wait: bool = True):
        # if not self._require_mode('set_coordinates', 'angle'):
        #     return
        if origin not in RaccoonBot._VALID_XYZ_ORIGIN:
            return _err(RaccoonBot, 'set_coordinates', 'origin', origin, RaccoonBot._VALID_XYZ_ORIGIN)
        for name, val in (('x', x), ('y', y), ('z', z)):
            if not isinstance(val, (int, float)):
                return _err(RaccoonBot, 'set_coordinates', name, val, 'int | float')
        if unit not in RaccoonBot._VALID_DIST_UNITS:
            return _err(RaccoonBot, 'set_coordinates', 'unit', unit, RaccoonBot._VALID_DIST_UNITS)
        xyz_cm = [RaccoonBot._coord_to_cm(v, unit) for v in (x, y, z)]
        angles = self._xyz_to_angles(xyz_cm, origin)
        if angles is None:
            return _err(RaccoonBot, 'set_coordinates', '(x, y, z)', xyz_cm, 'coordinates within the workspace')
        Runner.dispatch(lambda: self._set_angle_joints_impl(*angles), wait)

    # ── End Effector ──────────────────────────────────────────────────────────
    def lock(self, mode: _LockMode):
        if mode not in RaccoonBot._VALID_LOCK_MODES:
            return _err(RaccoonBot, 'lock', 'mode', mode, tuple(RaccoonBot._VALID_LOCK_MODES))
        self.write(RaccoonBot.END_EFFECTOR_LOCK, RaccoonBot._VALID_LOCK_MODES[mode])

    def pick(self):
        self.write(RaccoonBot.END_EFFECTOR_CONTROL, RaccoonBot._VALID_CONTROL['pick'])
        Runner.wait(0.01)

    def place(self):
        self.write(RaccoonBot.END_EFFECTOR_CONTROL, RaccoonBot._VALID_CONTROL['place'])
        Runner.wait(0.01)

    def end_effector_device(self) -> int:
        return self.read(RaccoonBot.END_EFFECTOR_DEVICE)

    def end_effector_status(self) -> int:
        return self.read(RaccoonBot.END_EFFECTOR_STATE)

    # ── Sound ─────────────────────────────────────────────────────────────────
    def sound_note(self, note: _Note, octave: int = 4):
        if note not in RaccoonBot._VALID_NOTES:
            return _err(RaccoonBot, 'sound_note', 'note', note, tuple(RaccoonBot._VALID_NOTES))
        if not isinstance(octave, int) or not (1 <= octave <= 7):
            return _err(RaccoonBot, 'sound_note', 'octave', octave, 'int (1~7)')
        self.write(RaccoonBot.SOUND_NOTE, (octave - 1) * 12 + RaccoonBot._VALID_NOTES[note] + 4)

    def _sound_clip_impl(self, clip):
        self.write(RaccoonBot.SOUND_CLIP, RaccoonBot._VALID_CLIPS[clip])
        Runner.wait_until(self._evaluate_sound)

    def sound_clip(self, clip: _Clip, wait: bool = True):
        if clip not in RaccoonBot._VALID_CLIPS:
            return _err(RaccoonBot, 'sound_clip', 'clip', clip, RaccoonBot._VALID_CLIPS)
        Runner.dispatch(lambda: self._sound_clip_impl(clip), wait)

    def sound_off(self):
        self._stop_sound()

    # ── Sensors ───────────────────────────────────────────────────────────────
    def encoder(self, joint: _JointId = -1) -> Union[int, float]:
        if joint not in RaccoonBot._VALID_JOINTS:
            return _err(RaccoonBot, 'encoder', 'joint', joint, RaccoonBot._VALID_JOINTS)
        if joint == -1:
            return [self.read(RaccoonBot._encoder_id(i)) for i in (1, 2, 3, 4)]
        return self.read(RaccoonBot._encoder_id(joint))

    # Only used in RobomationLAB
    def save_encoder(self, data: list) -> list:
        # Identity passthrough. Wrapping a captured encoder array via this method lets
        # python_to_blocks distinguish the save_encoder block from a plain variable assignment.
        # The actual variable binding is performed by the outer `var = ...` statement.
        return data

    def get_xyz(self, origin: _XyzOrigin = 'wrist', pos: Optional[_XyzPos] = None) -> Union[int, float, list]:
        if origin not in RaccoonBot._VALID_XYZ_ORIGIN:
            return _err(RaccoonBot, 'get_xyz', 'origin', origin, RaccoonBot._VALID_XYZ_ORIGIN)
        if pos is not None and pos not in RaccoonBot._VALID_XYZ_POS:
            return _err(RaccoonBot, 'get_xyz', 'pos', pos, tuple(RaccoonBot._VALID_XYZ_POS) + (None,))
        angles = [self.read(RaccoonBot._encoder_id(i)) for i in (1, 2, 3, 4)]
        xyz = self._angles_to_xyz(angles, origin)
        if pos is None:
            return xyz
        return xyz[RaccoonBot._VALID_XYZ_POS[pos]]

    def button(self, unit: _Button, event: _ButtonEvent) -> Union[int, bool]:
        if unit not in RaccoonBot._VALID_BUTTONS:
            return _err(RaccoonBot, 'button', 'unit', unit, RaccoonBot._VALID_BUTTONS)
        if event not in RaccoonBot._VALID_BUTTON_EVENTS:
            return _err(RaccoonBot, 'button', 'event', event, RaccoonBot._VALID_BUTTON_EVENTS)
        if event == 'pressed':
            if unit == 'any':
                return any(self.read(d) for d in RaccoonBot._BUTTON_PRESSED_IDS.values())
            dev = RaccoonBot._BUTTON_PRESSED_IDS.get(unit)
            return self.read(dev) if dev is not None else 0
        elif event == 'click':
            if unit == 'any':
                return any(self.e(d) for d in RaccoonBot._BUTTON_CLICK_IDS.values())
            dev = RaccoonBot._BUTTON_CLICK_IDS.get(unit)
            return self.e(dev) if dev is not None else False
        elif event == 'long_click':
            if unit == 'any':
                return any(self.e(d) for d in RaccoonBot._BUTTON_LONG_CLICK_IDS.values())
            dev = RaccoonBot._BUTTON_LONG_CLICK_IDS.get(unit)
            return self.e(dev) if dev is not None else False
        return False

    def signal_strength(self) -> Union[int, float]:
        return self.read(RaccoonBot.SIGNAL_STRENGTH)

    def battery(self) -> Union[int, float]:
        return self.read(RaccoonBot.BATTERY)

    # ── Peripheral  ─────────────────────────────────────────────────
    # def peripheral(self, unit: _Peripheral):
    #     if unit not in RaccoonBot._VALID_PERIPHERALS:
    #         return _err(RaccoonBot, 'peripheral', 'unit', unit, tuple(RaccoonBot._VALID_PERIPHERALS))
    #     self.write(RaccoonBot.PERIPHERAL, RaccoonBot._VALID_PERIPHERALS[unit])

    # def conveyor_mode(self, mode: _ConveyorMode):
    #     if mode not in RaccoonBot._VALID_CONVEYOR_MODES:
    #         return _err(RaccoonBot, 'conveyor_mode', 'mode', mode, tuple(RaccoonBot._VALID_CONVEYOR_MODES))
    #     self.write(RaccoonBot.CONVEYOR_MODE, RaccoonBot._VALID_CONVEYOR_MODES[mode])

    def conveyor_speed(self, data: Union[int, float]):
        if not isinstance(data, (int, float)):
            return _err(RaccoonBot, 'conveyor_speed', 'data', data, 'int | float')
        if self.read(RaccoonBot.CONVEYOR_DISTANCE) != 0:
            self.write(RaccoonBot.CONVEYOR_DISTANCE, 0)
        self.write(RaccoonBot.CONVEYOR_SPEED, data)
        self._conveyor_set = True

    def change_conveyor_speed(self, data: Union[int, float]):
        if not isinstance(data, (int, float)):
            return _err(RaccoonBot, 'change_conveyor_speed', 'data', data, 'int | float')
        if self.read(RaccoonBot.CONVEYOR_DISTANCE) != 0:
            self.write(RaccoonBot.CONVEYOR_DISTANCE, 0)
        self.write(RaccoonBot.CONVEYOR_SPEED, self.read(RaccoonBot.CONVEYOR_SPEED) + data)
        self._conveyor_set = True

    def _conveyor_move_impl(self, data, unit):
        self._begin_conveyor_speed()
        # 미설정 → default 100, 설정됨 → 사용자 값 그대로 (sign 은 data 부호로 결정)
        base = 100 if self._conveyor_set is not True else self.read(RaccoonBot.CONVEYOR_SPEED)
        self.write(RaccoonBot.CONVEYOR_SPEED, base if data > 0 else -base)
        self._mark_conveyor_speed()
        distance = abs(data) * RaccoonBot._CONVEYOR_CM_TO_PULSE
        if unit == 'mm': distance /= 10
        elif unit == 'inch': distance *= 2.54
        self.write(RaccoonBot.CONVEYOR_DISTANCE, int(distance))
        Runner.wait_until(self._evaluate_conveyor)
        self._restore_conveyor_speed()

    def conveyor_move(self, data: Union[int, float], unit: _DistUnit = 'cm', wait: bool = True):
        if not isinstance(data, (int, float)):
            return _err(RaccoonBot, 'conveyor_move', 'data', data, 'int | float')
        if unit not in RaccoonBot._VALID_DIST_UNITS:
            return _err(RaccoonBot, 'conveyor_move', 'unit', unit, RaccoonBot._VALID_DIST_UNITS)
        Runner.dispatch(lambda: self._conveyor_move_impl(data, unit), wait)

    def stop_conveyor(self):
        bounded = self.read(RaccoonBot.CONVEYOR_DISTANCE) != 0
        self._begin_conveyor_speed()
        self.write(RaccoonBot.CONVEYOR_DISTANCE, 0)
        self.write(RaccoonBot.CONVEYOR_SPEED, 0)
        if bounded:
            self._conveyor_set = False
        self._mark_conveyor_speed()

    def conveyor_running(self) -> bool:
        return self.read(RaccoonBot.CONVEYOR_RUNNING) != 0

    def conveyor_button(self, event: _ButtonEvent) -> Union[int, bool]:
        if event not in RaccoonBot._VALID_BUTTON_EVENTS:
            return _err(RaccoonBot, 'conveyor_button', 'event', event, RaccoonBot._VALID_BUTTON_EVENTS)
        if event == 'pressed':
            return self.read(RaccoonBot.CONVEYOR_BUTTON_PRESSED)
        elif event == 'click':
            return self.e(RaccoonBot.CONVEYOR_BUTTON_CLICK)
        elif event == 'long_click':
            return self.e(RaccoonBot.CONVEYOR_BUTTON_LONG_CLICK)
        return False
