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

# AI 확장 모듈 - 몸 찾기(BodyDetection)

import math
import time

from robomation.core.error import _err
from robomation.core.runner import Runner
from robomation.core.model import Robot
from robomation.ai._internal import _display
from typing import Literal, Union, get_args


# ── Module-level Literal aliases (for IDE auto-complete) ─────────────────────
_DistType = Literal['horizontal', 'vertical']
_Pos      = Literal['x', 'y']
_PartName = Literal['left_eye', 'right_eye', 'left_ear', 'right_ear',
                    'left_shoulder', 'right_shoulder', 'left_elbow', 'right_elbow', 
                    'left_wrist', 'right_wrist', 'left_hand', 'right_hand',
                    'left_hip', 'right_hip', 'left_knee', 'right_knee',
                    'left_ankle', 'right_ankle', 'left_foot', 'right_foot',
                    'nose', 'mouth']


# 신체 부위(내부 URN 세그먼트) — 각 x,y 좌표 디바이스를 가진다. (neck 은 그리기용, 디바이스 아님)
_SEGMENTS = [
    'nose', 'mouth',
    'eye.left', 'eye.right', 'ear.left', 'ear.right',
    'shoulder.left', 'shoulder.right', 'elbow.left', 'elbow.right',
    'wrist.left', 'wrist.right', 'hand.left', 'hand.right',
    'hip.left', 'hip.right', 'knee.left', 'knee.right',
    'ankle.left', 'ankle.right', 'foot.left', 'foot.right',
]
_COORD_KEYS = [f'{s}.{p}' for s in _SEGMENTS for p in ('x', 'y')]
_COORD_DEVICE = {k: 0xA0600202 + i for i, k in enumerate(_COORD_KEYS)}


class BodyDetection(Robot):
    """Body / pose detection AI extension."""
    ID = "kr.robomation.virtual.ai.body_detection"
    _instances = {}

    # ── Device IDs (product_id=6 → 0xA06xxxxx). eff 0x0xx / cmd 0x1xx / sensor 0x2xx / event 0x3xx
    # Effectors
    CAMERA_DEVICE      = 0xA0600000
    DISPLAY            = 0xA0600001

    # Commands
    LOAD_MODEL         = 0xA0600100
    DETECT_ONCE        = 0xA0600101
    DETECT_CONTINUOUS  = 0xA0600102

    # Sensors
    MODEL_STATE        = 0xA0600200
    DETECTED           = 0xA0600201
    # 좌표 (part.pos → device id)
    _COORD_DEVICE = _COORD_DEVICE

    # Event
    LOAD_MODEL_STATE   = 0xA0600300

    # ── Valid values ─────────────────────────────────────────────────────────
    _VALID_DISTANCE_TYPE = get_args(_DistType)
    _VALID_POS           = get_args(_Pos)

    # User-facing part name → 내부 좌표 키 세그먼트
    _PARTS = {
        'nose': 'nose', 'mouth': 'mouth',
        'left_eye': 'eye.left', 'right_eye': 'eye.right',
        'left_ear': 'ear.left', 'right_ear': 'ear.right',
        'left_shoulder': 'shoulder.left', 'right_shoulder': 'shoulder.right',
        'left_elbow': 'elbow.left', 'right_elbow': 'elbow.right',
        'left_wrist': 'wrist.left', 'right_wrist': 'wrist.right',
        'left_hand': 'hand.left', 'right_hand': 'hand.right',
        'left_hip': 'hip.left', 'right_hip': 'hip.right',
        'left_knee': 'knee.left', 'right_knee': 'knee.right',
        'left_ankle': 'ankle.left', 'right_ankle': 'ankle.right',
        'left_foot': 'foot.left', 'right_foot': 'foot.right',
    }

    # ── Robot lifecycle ──────────────────────────────────────────────────────
    def __init__(self, index=0):
        if isinstance(index, str):
            index = 0
        if index in BodyDetection._instances:
            robot = BodyDetection._instances[index]
            if robot: robot.dispose()
        BodyDetection._instances[index] = self
        super(BodyDetection, self).__init__(BodyDetection.ID, "BodyDetection", index)
        self._title = f"BodyDetection {index}"
        self._init()

    def dispose(self):
        BodyDetection._instances[self.get_index()] = None
        _display.remove_view(self._title)
        self._roboid._dispose()
        Runner.unregister_robot(self)

    def reset(self):
        self._roboid._reset()

    def _init(self):
        from robomation.ai.body_detection_roboid import BodyDetectionRoboid
        self._roboid = BodyDetectionRoboid(self.get_index())
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
        # 인식 결과(골격 연결선 + 관절점)는 DISPLAY 가 켜져 있고 검출됐을 때만 오버레이.
        if self.read(BodyDetection.DISPLAY) and engine._has_drawing():
            disp = frame.copy()
            engine._draw_overlay(disp)
            return (self._title, disp)
        return (self._title, frame)

    def _read_part(self, method, part, pos):
        if pos not in BodyDetection._VALID_POS:
            return _err(BodyDetection, method, 'pos', pos, BodyDetection._VALID_POS)
        dev_id = BodyDetection._COORD_DEVICE.get(BodyDetection._PARTS[part] + '.' + pos)
        return self.read(dev_id) if dev_id is not None else 0

    @staticmethod
    def _resolve_part(name):
        return BodyDetection._PARTS.get(name)

    # ── Public API ───────────────────────────────────────────────────────────
    def device(self, unit):
        from robomation.ai._internal._camera import Camera, resolve
        index = unit._index if isinstance(unit, Camera) else resolve(unit)
        self.write(BodyDetection.CAMERA_DEVICE, index)

        # 카메라를 붙이면 일단 카메라 화면(프레임)부터 표시한다(모델 로드 전에도).
        if index >= 0:
            _display.add_view(self._title, self._view)
        else:
            _display.remove_view(self._title)

    def load_model(self, wait: bool = True):
        self.write(BodyDetection.LOAD_MODEL, 1)
        if wait:
            timeout = time.time() + 15
            while self.model_state() != 2 and time.time() < timeout:
                time.sleep(0.01)

    def detect_once(self):
        self.write(BodyDetection.DETECT_ONCE, 1)

    def detect_continuous(self):
        self.write(BodyDetection.DETECT_CONTINUOUS, 1)

    def stop(self):
        self.write(BodyDetection.DETECT_CONTINUOUS, 0)

    def display(self, on: Union[bool, Literal[1, 0]] = True):
        if on not in (True, False, 1, 0):
            return _err(BodyDetection, 'display', 'on', on, 'bool | 1 | 0')
        self.write(BodyDetection.DISPLAY, 1 if on else 0)

    # ── Body parts ────────────────────────────────────────────────────────────
    def left_eye(self, pos: _Pos) -> int:
        return self._read_part('left_eye', 'left_eye', pos)

    def right_eye(self, pos: _Pos) -> int:
        return self._read_part('right_eye', 'right_eye', pos)

    def left_ear(self, pos: _Pos) -> int:
        return self._read_part('left_ear', 'left_ear', pos)

    def right_ear(self, pos: _Pos) -> int:
        return self._read_part('right_ear', 'right_ear', pos)

    def nose(self, pos: _Pos) -> int:
        return self._read_part('nose', 'nose', pos)

    def mouth(self, pos: _Pos) -> int:
        return self._read_part('mouth', 'mouth', pos)

    def left_shoulder(self, pos: _Pos) -> int:
        return self._read_part('left_shoulder', 'left_shoulder', pos)

    def right_shoulder(self, pos: _Pos) -> int:
        return self._read_part('right_shoulder', 'right_shoulder', pos)

    def left_elbow(self, pos: _Pos) -> int:
        return self._read_part('left_elbow', 'left_elbow', pos)

    def right_elbow(self, pos: _Pos) -> int:
        return self._read_part('right_elbow', 'right_elbow', pos)

    def left_wrist(self, pos: _Pos) -> int:
        return self._read_part('left_wrist', 'left_wrist', pos)

    def right_wrist(self, pos: _Pos) -> int:
        return self._read_part('right_wrist', 'right_wrist', pos)

    def left_hand(self, pos: _Pos) -> int:
        return self._read_part('left_hand', 'left_hand', pos)

    def right_hand(self, pos: _Pos) -> int:
        return self._read_part('right_hand', 'right_hand', pos)

    def left_hip(self, pos: _Pos) -> int:
        return self._read_part('left_hip', 'left_hip', pos)

    def right_hip(self, pos: _Pos) -> int:
        return self._read_part('right_hip', 'right_hip', pos)

    def left_knee(self, pos: _Pos) -> int:
        return self._read_part('left_knee', 'left_knee', pos)

    def right_knee(self, pos: _Pos) -> int:
        return self._read_part('right_knee', 'right_knee', pos)

    def left_ankle(self, pos: _Pos) -> int:
        return self._read_part('left_ankle', 'left_ankle', pos)

    def right_ankle(self, pos: _Pos) -> int:
        return self._read_part('right_ankle', 'right_ankle', pos)

    def left_foot(self, pos: _Pos) -> int:
        return self._read_part('left_foot', 'left_foot', pos)

    def right_foot(self, pos: _Pos) -> int:
        return self._read_part('right_foot', 'right_foot', pos)

    def get_distance(self, unit1: _PartName, unit2: _PartName, type: _DistType = None) -> float:
        if type is not None and type not in BodyDetection._VALID_DISTANCE_TYPE:
            return _err(BodyDetection, 'get_distance', 'type', type, BodyDetection._VALID_DISTANCE_TYPE)
        p1 = BodyDetection._resolve_part(unit1)
        p2 = BodyDetection._resolve_part(unit2)
        dx = self._read_seg(p2, 'x') - self._read_seg(p1, 'x')
        dy = self._read_seg(p2, 'y') - self._read_seg(p1, 'y')
        if type is None:
            return math.sqrt(dx * dx + dy * dy)
        if type == 'horizontal':
            return math.fabs(dx)
        return math.fabs(dy)

    def _read_seg(self, seg, pos):
        dev_id = BodyDetection._COORD_DEVICE.get(seg + '.' + pos)
        return self.read(dev_id) if dev_id is not None else 0

    def model_state(self) -> int:
        return self.read(BodyDetection.MODEL_STATE)

    def detected(self) -> bool:
        return self.read(BodyDetection.DETECTED) == 1
