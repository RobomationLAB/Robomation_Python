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

# AI 확장 모듈 - 손 찾기(HandDetection)

import math
import time

from robomation.core.error import _err
from robomation.core.runner import Runner
from robomation.core.model import Robot
from robomation.ai._internal import _display
from typing import Literal, Union, get_args


# ── Module-level Literal aliases (for IDE auto-complete) ─────────────────────
_DistType  = Literal['horizontal', 'vertical']
_Pos       = Literal['x', 'y', 'min_x', 'max_x', 'min_y', 'max_y', 'width', 'height', 'area']
_FingerPos = Literal['x', 'y']
_Side      = Literal['left', 'right']
_HandPart  = Literal['palm', 'wrist', 'hand']
_Finger    = Literal['thumb', 'index', 'middle', 'ring', 'pinky']
_Joint     = Literal['first', 'second', 'third', 'last']
_MaxHands  = Literal['one', 'both']


# ── 좌표 키 생성 (좌/우 × wrist/palm/hand/손가락) ────────────────────────────
# part별 보유 좌표: wrist=x,y / palm=x,y+박스 / hand=박스만 / 손가락 joint=x,y
_SQUARE  = ('min_x', 'max_x', 'min_y', 'max_y', 'width', 'height', 'area')
_FINGERS = ('thumb', 'index', 'middle', 'ring', 'pinky')
_JOINTS  = ('first', 'second', 'third', 'last')

def _gen_coord_keys():
    keys = []
    for side in ('left', 'right'):
        keys += [f'{side}.wrist.x', f'{side}.wrist.y']
        keys += [f'{side}.palm.x', f'{side}.palm.y']
        keys += [f'{side}.palm.{s}' for s in _SQUARE]
        keys += [f'{side}.hand.{s}' for s in _SQUARE]
        for f in _FINGERS:
            for j in _JOINTS:
                keys += [f'{side}.{f}.{j}.x', f'{side}.{f}.{j}.y']
    return keys

_COORD_KEYS = _gen_coord_keys()
# 좌표 디바이스 ID: sensor 영역 0x_2xx, MODEL_STATE/DETECTED 다음(0x202)부터 순차
_COORD_DEVICE = {k: 0xA0500202 + i for i, k in enumerate(_COORD_KEYS)}


class HandDetection(Robot):
    """Hand detection AI extension."""
    ID = "kr.robomation.virtual.ai.hand_detection"
    _instances = {}

    # ── Device IDs (product_id=5 → 0xA05xxxxx). eff 0x0xx / cmd 0x1xx / sensor 0x2xx / event 0x3xx
    # Effectors
    CAMERA_DEVICE      = 0xA0500000
    DISPLAY            = 0xA0500001
    MAX_COUNT          = 0xA0500002

    # Commands
    LOAD_MODEL         = 0xA0500100
    DETECT_ONCE        = 0xA0500101
    DETECT_CONTINUOUS  = 0xA0500102

    # Sensors
    MODEL_STATE        = 0xA0500200
    DETECTED           = 0xA0500201
    # 좌표 (left/right.part[.joint].pos → device id)
    _COORD_DEVICE = _COORD_DEVICE

    # Event
    LOAD_MODEL_STATE   = 0xA0500300

    # ── Valid values ─────────────────────────────────────────────────────────
    _VALID_DISTANCE_TYPE = get_args(_DistType)
    _VALID_POS           = get_args(_Pos)
    _VALID_FINGER_POS    = get_args(_FingerPos)
    _VALID_SIDE          = get_args(_Side)
    _VALID_HAND_PART     = get_args(_HandPart)
    _VALID_FINGERS       = get_args(_Finger)
    _VALID_JOINTS        = get_args(_Joint)
    _VALID_HANDS         = {'one': 1, 'both': 2}

    # ── Robot lifecycle ──────────────────────────────────────────────────────
    def __init__(self, index=0):
        if isinstance(index, str):
            index = 0
        if index in HandDetection._instances:
            robot = HandDetection._instances[index]
            if robot: robot.dispose()
        HandDetection._instances[index] = self
        super(HandDetection, self).__init__(HandDetection.ID, "HandDetection", index)
        self._title = f"HandDetection {index}"
        self._init()

    def dispose(self):
        HandDetection._instances[self.get_index()] = None
        _display.remove_view(self._title)
        self._roboid._dispose()
        Runner.unregister_robot(self)

    def reset(self):
        self._roboid._reset()

    def _init(self):
        from robomation.ai.hand_detection_roboid import HandDetectionRoboid
        self._roboid = HandDetectionRoboid(self.get_index())
        self._add_roboid(self._roboid)
        Runner.register_robot(self)
        Runner.start()
        self._roboid._init()

    def find_device_by_id(self, device_id):
        return self._roboid._find_device_by_id(device_id)

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

    # ── Internal helpers ─────────────────────────────────────────────────────
    def _view(self):
        roboid = getattr(self, '_roboid', None)
        engine = getattr(roboid, '_engine', None) if roboid else None
        if engine is None:
            return None
        frame = engine._read_frame()
        if frame is None:
            return None
        # 인식 결과(손 연결선 + 관절점)는 DISPLAY 가 켜져 있고 검출됐을 때만 오버레이.
        if self.read(HandDetection.DISPLAY) and engine._has_drawing():
            disp = frame.copy()
            engine._draw_overlay(disp)
            return (self._title, disp)
        return (self._title, frame)

    def _read_key(self, key):
        dev_id = HandDetection._COORD_DEVICE.get(key)
        return self.read(dev_id) if dev_id is not None else 0

    @staticmethod
    def _build_path(spec):
        if not isinstance(spec, str):
            return None
        parts = spec.split('_')
        if len(parts) == 2:
            side, unit = parts
            if side not in HandDetection._VALID_SIDE:
                return None
            if unit not in HandDetection._VALID_HAND_PART:
                return None
            return f'{side}.{unit}'
        if len(parts) == 3:
            side, unit, joint = parts
            if side not in HandDetection._VALID_SIDE:
                return None
            if unit not in HandDetection._VALID_FINGERS:
                return None
            if joint not in HandDetection._VALID_JOINTS:
                return None
            return f'{side}.{unit}.{joint}'
        return None

    # ── Public API ───────────────────────────────────────────────────────────
    def device(self, unit):
        from robomation.ai._internal._camera import Camera, resolve
        index = unit._index if isinstance(unit, Camera) else resolve(unit)
        self.write(HandDetection.CAMERA_DEVICE, index)

        # 카메라를 붙이면 일단 카메라 화면(프레임)부터 표시한다(모델 로드 전에도).
        if index >= 0:
            _display.add_view(self._title, self._view)
        else:
            _display.remove_view(self._title)

    def load_model(self, wait: bool = True):
        self.write(HandDetection.LOAD_MODEL, 1)
        if wait:
            timeout = time.time() + 15
            while self.model_state() != 2 and time.time() < timeout:
                time.sleep(0.01)

    def max_hands(self, unit: _MaxHands):
        if unit not in HandDetection._VALID_HANDS:
            return _err(HandDetection, 'max_hands', 'unit', unit, tuple(HandDetection._VALID_HANDS))
        self.write(HandDetection.MAX_COUNT, HandDetection._VALID_HANDS[unit])

    def detect_once(self):
        self.write(HandDetection.DETECT_ONCE, 1)

    def detect_continuous(self):
        self.write(HandDetection.DETECT_CONTINUOUS, 1)

    def stop(self):
        self.write(HandDetection.DETECT_CONTINUOUS, 0)

    def display(self, on: Union[bool, Literal[1, 0]] = True):
        if on not in (True, False, 1, 0):
            return _err(HandDetection, 'display', 'on', on, 'bool | 1 | 0')
        self.write(HandDetection.DISPLAY, 1 if on else 0)

    # ── Hand / Finger ─────────────────────────────────────────────────────────
    def hand(self, side: _Side, unit: _HandPart, pos: _Pos) -> int:
        if side not in HandDetection._VALID_SIDE:
            return _err(HandDetection, 'hand', 'side', side, HandDetection._VALID_SIDE)
        if unit not in HandDetection._VALID_HAND_PART:
            return _err(HandDetection, 'hand', 'unit', unit, HandDetection._VALID_HAND_PART)
        if pos not in HandDetection._VALID_POS:
            return _err(HandDetection, 'hand', 'pos', pos, HandDetection._VALID_POS)
        return self._read_key(side + '.' + unit + '.' + pos)

    def finger(self, side: _Side, unit: _Finger, joint: _Joint, pos: _FingerPos) -> int:
        if side not in HandDetection._VALID_SIDE:
            return _err(HandDetection, 'finger', 'side', side, HandDetection._VALID_SIDE)
        if unit not in HandDetection._VALID_FINGERS:
            return _err(HandDetection, 'finger', 'unit', unit, HandDetection._VALID_FINGERS)
        if joint not in HandDetection._VALID_JOINTS:
            return _err(HandDetection, 'finger', 'joint', joint, HandDetection._VALID_JOINTS)
        if pos not in HandDetection._VALID_FINGER_POS:
            return _err(HandDetection, 'finger', 'pos', pos, HandDetection._VALID_FINGER_POS)
        return self._read_key(side + '.' + unit + '.' + joint + '.' + pos)

    def get_distance(self, unit1: str, unit2: str, type: _DistType = None) -> Union[int, float]:
        if type is not None and type not in HandDetection._VALID_DISTANCE_TYPE:
            return _err(HandDetection, 'get_distance', 'type', type, HandDetection._VALID_DISTANCE_TYPE)
        p1 = HandDetection._build_path(unit1)
        p2 = HandDetection._build_path(unit2)
        if p1 is None:
            return _err(HandDetection, 'get_distance', 'unit1', unit1, "'side_unit' or 'side_unit_joint'")
        if p2 is None:
            return _err(HandDetection, 'get_distance', 'unit2', unit2, "'side_unit' or 'side_unit_joint'")
        dx = self._read_key(p2 + '.x') - self._read_key(p1 + '.x')
        dy = self._read_key(p2 + '.y') - self._read_key(p1 + '.y')
        if type is None:
            return math.sqrt(dx * dx + dy * dy)
        if type == 'horizontal':
            return math.fabs(dx)
        return math.fabs(dy)

    def model_state(self) -> int:
        return self.read(HandDetection.MODEL_STATE)

    def detected(self) -> bool:
        return self.read(HandDetection.DETECTED) == 1
