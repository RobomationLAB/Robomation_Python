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

# AI 확장 모듈 - 상세하게 얼굴 찾기(DetailedFaceDetection)

import math
import time

import cv2

from robomation.core.error import _err
from robomation.core.runner import Runner
from robomation.core.model import Robot
from robomation.ai._internal import _display
from typing import Literal, Union, get_args


# ── Module-level Literal aliases (for IDE auto-complete) ─────────────────────
_DistType  = Literal['horizontal', 'vertical']
_Pos       = Literal['x', 'y']
_PosSquare = Literal['x', 'y', 'min_x', 'max_x', 'min_y', 'max_y', 'width', 'height', 'area']
_PartName  = Literal['face', 
                     'left_eye', 'right_eye', 
                     'upper_lip', 'lower_lip', 
                     'left_lip', 'right_lip',
                     'left_pupil', 'right_pupil',
                     'mouth', 'nose']


class DetailedFaceDetection(Robot):
    """Detailed face detection AI extension (face + landmarks)."""
    ID = "kr.robomation.virtual.ai.detailed_face_detection"
    _instances = {}

    # ── Device IDs (product_id=3 → 0xA03xxxxx)
    # Effectors
    CAMERA_DEVICE      = 0xA0300000
    DISPLAY            = 0xA0300001

    # Commands
    LOAD_MODEL         = 0xA0300100
    DETECT_ONCE        = 0xA0300101
    DETECT_CONTINUOUS  = 0xA0300102

    # Sensors
    MODEL_STATE        = 0xA0300200
    DETECTED           = 0xA0300201

    # 좌표 (part.pos → device id). face/eye.left/eye.right/mouth 는 9필드, 나머지는 x,y.
    _COORD_DEVICE = {
        'face.x':       0xA0300202, 'face.y':       0xA0300203,
        'face.min_x':   0xA0300204, 'face.max_x':   0xA0300205,
        'face.min_y':   0xA0300206, 'face.max_y':   0xA0300207,
        'face.width':   0xA0300208, 'face.height':  0xA0300209, 'face.area': 0xA030020A,
        'eye.left.x':     0xA030020B, 'eye.left.y':     0xA030020C,
        'eye.left.min_x': 0xA030020D, 'eye.left.max_x': 0xA030020E,
        'eye.left.min_y': 0xA030020F, 'eye.left.max_y': 0xA0300210,
        'eye.left.width': 0xA0300211, 'eye.left.height':0xA0300212, 'eye.left.area': 0xA0300213,
        'eye.right.x':     0xA0300214, 'eye.right.y':     0xA0300215,
        'eye.right.min_x': 0xA0300216, 'eye.right.max_x': 0xA0300217,
        'eye.right.min_y': 0xA0300218, 'eye.right.max_y': 0xA0300219,
        'eye.right.width': 0xA030021A, 'eye.right.height':0xA030021B, 'eye.right.area': 0xA030021C,
        'mouth.x':     0xA030021D, 'mouth.y':     0xA030021E,
        'mouth.min_x': 0xA030021F, 'mouth.max_x': 0xA0300220,
        'mouth.min_y': 0xA0300221, 'mouth.max_y': 0xA0300222,
        'mouth.width': 0xA0300223, 'mouth.height':0xA0300224, 'mouth.area': 0xA0300225,
        'nose.x':       0xA0300226, 'nose.y':       0xA0300227,
        'lip.up.x':     0xA0300228, 'lip.up.y':     0xA0300229,
        'lip.down.x':   0xA030022A, 'lip.down.y':   0xA030022B,
        'lip.left.x':   0xA030022C, 'lip.left.y':   0xA030022D,
        'lip.right.x':  0xA030022E, 'lip.right.y':  0xA030022F,
        'pupil.left.x': 0xA0300230, 'pupil.left.y': 0xA0300231,
        'pupil.right.x':0xA0300232, 'pupil.right.y':0xA0300233,
    }

    # Event
    LOAD_MODEL_STATE   = 0xA0300300

    # User-facing part name → 내부 좌표 키 세그먼트
    _PARTS = {
        # Square + x/y parts
        'face':        'face',
        'left_eye':    'eye.left',    'right_eye':   'eye.right',
        'mouth':       'mouth',
        # x/y only parts
        'nose':        'nose',
        'upper_lip':   'lip.up',      'lower_lip':   'lip.down',
        'left_lip':    'lip.left',    'right_lip':   'lip.right',
        'left_pupil':  'pupil.left',  'right_pupil': 'pupil.right',
    }

    # ── Valid values ─────────────────────────────────────────────────────────
    _VALID_DISTANCE_TYPE = get_args(_DistType)
    _VALID_POS           = get_args(_Pos)
    _VALID_POS_SQUARE    = get_args(_PosSquare)

    # ── Robot lifecycle ──────────────────────────────────────────────────────
    def __init__(self, index=0):
        if isinstance(index, str):
            index = 0
        if index in DetailedFaceDetection._instances:
            robot = DetailedFaceDetection._instances[index]
            if robot: robot.dispose()
        DetailedFaceDetection._instances[index] = self
        super(DetailedFaceDetection, self).__init__(DetailedFaceDetection.ID, "DetailedFaceDetection", index)
        self._title = f"DetailedFaceDetection {index}"
        self._init()

    def dispose(self):
        DetailedFaceDetection._instances[self.get_index()] = None
        _display.remove_view(self._title)
        self._roboid._dispose()
        Runner.unregister_robot(self)

    def reset(self):
        self._roboid._reset()

    def _init(self):
        from robomation.ai.detailed_face_detection_roboid import DetailedFaceDetectionRoboid
        self._roboid = DetailedFaceDetectionRoboid(self.get_index())
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
        # 인식 결과(메시/윤곽/눈·입술·홍채)는 DISPLAY 가 켜져 있고 검출됐을 때만 오버레이.
        if self.read(DetailedFaceDetection.DISPLAY) and engine._has_drawing():
            disp = frame.copy()
            engine._draw_overlay(disp)
            return (self._title, disp)
        return (self._title, frame)

    def _read_part(self, method, part, pos, allowed):
        if pos not in allowed:
            return _err(DetailedFaceDetection, method, 'pos', pos, allowed)
        return self.read(DetailedFaceDetection._COORD_DEVICE[DetailedFaceDetection._PARTS[part] + '.' + pos])

    @staticmethod
    def _resolve_part(name):
        return DetailedFaceDetection._PARTS.get(name)

    # ── Public API ───────────────────────────────────────────────────────────
    def device(self, unit):
        from robomation.ai._internal._camera import Camera, resolve
        index = unit._index if isinstance(unit, Camera) else resolve(unit)
        self.write(DetailedFaceDetection.CAMERA_DEVICE, index)

        # 카메라를 붙이면 일단 카메라 화면(프레임)부터 표시한다(모델 로드 전에도).
        if index >= 0:
            _display.add_view(self._title, self._view)
        else:
            _display.remove_view(self._title)

    def load_model(self, wait: bool = True):
        self.write(DetailedFaceDetection.LOAD_MODEL, 1)
        if wait:
            timeout = time.time() + 10
            while self.model_state() != 2 and time.time() < timeout:
                time.sleep(0.01)

    def detect_once(self):
        self.write(DetailedFaceDetection.DETECT_ONCE, 1)

    def detect_continuous(self):
        self.write(DetailedFaceDetection.DETECT_CONTINUOUS, 1)

    def stop(self):
        self.write(DetailedFaceDetection.DETECT_CONTINUOUS, 0)

    def display(self, on: Union[bool, Literal[1, 0]] = True):
        if on not in (True, False, 1, 0):
            return _err(DetailedFaceDetection, 'display', 'on', on, 'bool | 1 | 0')
        self.write(DetailedFaceDetection.DISPLAY, 1 if on else 0)

    # Square + x/y (face / left_eye / right_eye / mouth)
    def face(self, pos: _PosSquare) -> int:
        return self._read_part('face', 'face', pos, DetailedFaceDetection._VALID_POS_SQUARE)

    def left_eye(self, pos: _PosSquare) -> int:
        return self._read_part('left_eye', 'left_eye', pos, DetailedFaceDetection._VALID_POS_SQUARE)

    def right_eye(self, pos: _PosSquare) -> int:
        return self._read_part('right_eye', 'right_eye', pos, DetailedFaceDetection._VALID_POS_SQUARE)

    def mouth(self, pos: _PosSquare) -> int:
        return self._read_part('mouth', 'mouth', pos, DetailedFaceDetection._VALID_POS_SQUARE)

    # x/y only (nose / lips / pupils)
    def nose(self, pos: _Pos) -> int:
        return self._read_part('nose', 'nose', pos, DetailedFaceDetection._VALID_POS)

    def upper_lip(self, pos: _Pos) -> int:
        return self._read_part('upper_lip', 'upper_lip', pos, DetailedFaceDetection._VALID_POS)

    def lower_lip(self, pos: _Pos) -> int:
        return self._read_part('lower_lip', 'lower_lip', pos, DetailedFaceDetection._VALID_POS)

    def left_lip(self, pos: _Pos) -> int:
        return self._read_part('left_lip', 'left_lip', pos, DetailedFaceDetection._VALID_POS)

    def right_lip(self, pos: _Pos) -> int:
        return self._read_part('right_lip', 'right_lip', pos, DetailedFaceDetection._VALID_POS)

    def left_pupil(self, pos: _Pos) -> int:
        return self._read_part('left_pupil', 'left_pupil', pos, DetailedFaceDetection._VALID_POS)

    def right_pupil(self, pos: _Pos) -> int:
        return self._read_part('right_pupil', 'right_pupil', pos, DetailedFaceDetection._VALID_POS)

    def get_distance(self, unit1: _PartName, unit2: _PartName, type: _DistType = None) -> float:
        if type is not None and type not in DetailedFaceDetection._VALID_DISTANCE_TYPE:
            return _err(DetailedFaceDetection, 'get_distance', 'type', type, DetailedFaceDetection._VALID_DISTANCE_TYPE)
        p1 = DetailedFaceDetection._resolve_part(unit1)
        p2 = DetailedFaceDetection._resolve_part(unit2)
        dx = self.read(DetailedFaceDetection._COORD_DEVICE[p2 + '.x']) - self.read(DetailedFaceDetection._COORD_DEVICE[p1 + '.x'])
        dy = self.read(DetailedFaceDetection._COORD_DEVICE[p2 + '.y']) - self.read(DetailedFaceDetection._COORD_DEVICE[p1 + '.y'])
        if type is None:
            return math.sqrt(dx * dx + dy * dy)
        if type == 'horizontal':
            return math.fabs(dx)
        return math.fabs(dy)

    def model_state(self) -> int:
        return self.read(DetailedFaceDetection.MODEL_STATE)

    def detected(self) -> bool:
        return self.read(DetailedFaceDetection.DETECTED) == 1
