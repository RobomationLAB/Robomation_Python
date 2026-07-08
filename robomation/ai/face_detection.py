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

# AI 확장 모듈 - 얼굴 찾기(FaceDetection)

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
_Square   = Literal['min_x', 'max_x', 'min_y', 'max_y', 'width', 'height', 'area']
_FacePos  = Literal['x', 'y', 'min_x', 'max_x', 'min_y', 'max_y', 'width', 'height', 'area']
_PartName = Literal['face', 'left_eye', 'right_eye', 'left_ear', 'right_ear', 'nose', 'mouth']


class FaceDetection(Robot):
    """Face detection AI extension."""
    ID = "kr.robomation.virtual.ai.face_detection"
    _instances = {}

    # ── Device IDs (product_id=2 → 0xA02xxxxx)
    # Effectors
    CAMERA_DEVICE      = 0xA0200000
    DISPLAY            = 0xA0200001

    # Commands
    LOAD_MODEL         = 0xA0200100
    DETECT_ONCE        = 0xA0200101
    DETECT_CONTINUOUS  = 0xA0200102

    # Sensors 
    MODEL_STATE        = 0xA0200200
    DETECTED           = 0xA0200201

    # 좌표 (part.pos → device id).
    _COORD_DEVICE = {
        'face.x':      0xA0200202,  'face.y':      0xA0200203,
        'face.min_x':  0xA0200204,  'face.max_x':  0xA0200205,
        'face.min_y':  0xA0200206,  'face.max_y':  0xA0200207,
        'face.width':  0xA0200208,  'face.height': 0xA0200209,  'face.area': 0xA020020A,
        'eye.left.x':  0xA020020B,  'eye.left.y':  0xA020020C,
        'eye.right.x': 0xA020020D,  'eye.right.y': 0xA020020E,
        'ear.left.x':  0xA020020F,  'ear.left.y':  0xA0200210,
        'ear.right.x': 0xA0200211,  'ear.right.y': 0xA0200212,
        'nose.x':      0xA0200213,  'nose.y':      0xA0200214,
        'mouth.x':     0xA0200215,  'mouth.y':     0xA0200216,
    }

    # Event
    LOAD_MODEL_STATE   = 0xA0200300

    # User-facing part name → 내부 좌표 키 세그먼트
    _PARTS = {
        'face': 'face',
        'left_eye': 'eye.left',     'right_eye': 'eye.right',
        'left_ear': 'ear.left',     'right_ear': 'ear.right',
        'nose': 'nose',             'mouth': 'mouth',
    }
    
    # ── Valid values ─────────────────────────────────────────────────────────
    _VALID_DISTANCE_TYPE = get_args(_DistType)
    _VALID_POS           = get_args(_Pos)
    _VALID_SQUARE        = get_args(_Square)
    _VALID_FACE_POS      = get_args(_FacePos)

    # ── Robot lifecycle ──────────────────────────────────────────────────────
    def __init__(self, index=0):
        if isinstance(index, str):
            index = 0
        if index in FaceDetection._instances:
            robot = FaceDetection._instances[index]
            if robot: robot.dispose()
        FaceDetection._instances[index] = self
        super(FaceDetection, self).__init__(FaceDetection.ID, "FaceDetection", index)
        self._title = f"FaceDetection {index}"
        self._init()

    def dispose(self):
        FaceDetection._instances[self.get_index()] = None
        _display.remove_view(self._title)
        self._roboid._dispose()
        Runner.unregister_robot(self)

    def reset(self):
        self._roboid._reset()

    def _init(self):
        from robomation.ai.face_detection_roboid import FaceDetectionRoboid
        self._roboid = FaceDetectionRoboid(self.get_index())
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
        # 인식 결과(박스/랜드마크)는 DISPLAY 가 켜져 있고 검출됐을 때만 오버레이.
        if self.read(FaceDetection.DISPLAY) and engine._has_drawing():
            disp = frame.copy()
            engine._draw_overlay(disp)
            return (self._title, disp)
        return (self._title, frame)

    def _read_part(self, method, part, pos, allowed):
        if pos not in allowed:
            return _err(FaceDetection, method, 'pos', pos, allowed)
        return self.read(FaceDetection._COORD_DEVICE[FaceDetection._PARTS[part] + '.' + pos])

    @staticmethod
    def _resolve_part(name):
        return FaceDetection._PARTS.get(name)

    # ── Public API ───────────────────────────────────────────────────────────
    def device(self, unit):
        from robomation.ai._internal._camera import Camera, resolve
        index = unit._index if isinstance(unit, Camera) else resolve(unit)
        self.write(FaceDetection.CAMERA_DEVICE, index)
        
        # 카메라를 붙이면 일단 카메라 화면(프레임)부터 표시한다(모델 로드 전에도).
        if index >= 0:
            _display.add_view(self._title, self._view)
        else:
            _display.remove_view(self._title)

    def load_model(self, wait: bool = True):
        self.write(FaceDetection.LOAD_MODEL, 1)
        if wait:
            timeout = time.time() + 10
            while self.model_state() != 2 and time.time() < timeout:
                time.sleep(0.01)

    def detect_once(self):
        self.write(FaceDetection.DETECT_ONCE, 1)

    def detect_continuous(self):
        self.write(FaceDetection.DETECT_CONTINUOUS, 1)

    def stop(self):
        self.write(FaceDetection.DETECT_CONTINUOUS, 0)

    def display(self, on: Union[bool, Literal[1, 0]] = True):
        if on not in (True, False, 1, 0):
            return _err(FaceDetection, 'display', 'on', on, 'bool | 1 | 0')
        self.write(FaceDetection.DISPLAY, 1 if on else 0)

    def face(self, pos: _FacePos) -> int:      
        return self._read_part('face', 'face', pos, FaceDetection._VALID_FACE_POS)
    
    def left_eye(self, pos: _Pos) -> int:      
        return self._read_part('left_eye', 'left_eye', pos, FaceDetection._VALID_POS)
    
    def right_eye(self, pos: _Pos) -> int:     
        return self._read_part('right_eye', 'right_eye', pos, FaceDetection._VALID_POS)
    
    def left_ear(self, pos: _Pos) -> int:      
        return self._read_part('left_ear', 'left_ear', pos, FaceDetection._VALID_POS)
    
    def right_ear(self, pos: _Pos) -> int:     
        return self._read_part('right_ear', 'right_ear', pos, FaceDetection._VALID_POS)
    
    def nose(self, pos: _Pos) -> int:          
        return self._read_part('nose', 'nose', pos, FaceDetection._VALID_POS)
    
    def mouth(self, pos: _Pos) -> int:         
        return self._read_part('mouth', 'mouth', pos, FaceDetection._VALID_POS)
    
    def get_distance(self, unit1: _PartName, unit2: _PartName, type: _DistType = None) -> float:
        if type is not None and type not in FaceDetection._VALID_DISTANCE_TYPE:
            return _err(FaceDetection, 'get_distance', 'type', type, FaceDetection._VALID_DISTANCE_TYPE)
        p1 = FaceDetection._resolve_part(unit1)
        p2 = FaceDetection._resolve_part(unit2)
        dx = self.read(FaceDetection._COORD_DEVICE[p2 + '.x']) - self.read(FaceDetection._COORD_DEVICE[p1 + '.x'])
        dy = self.read(FaceDetection._COORD_DEVICE[p2 + '.y']) - self.read(FaceDetection._COORD_DEVICE[p1 + '.y'])
        if type is None:
            return math.sqrt(dx * dx + dy * dy)
        if type == 'horizontal':
            return math.fabs(dx)
        return math.fabs(dy)

    def model_state(self) -> int:
        return self.read(FaceDetection.MODEL_STATE)

    def detected(self) -> bool:
        return self.read(FaceDetection.DETECTED) == 1
