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

# AI 확장 모듈 - 사물 찾기(ObjectDetection)

import math
import time

from robomation.core.error import _err
from robomation.core.runner import Runner
from robomation.core.model import Robot
from robomation.ai._internal import _display
from typing import Literal, Union, get_args


# ── Module-level Literal aliases (for IDE auto-complete) ─────────────────────
_DistType = Literal['horizontal', 'vertical']
_Pos      = Literal['x', 'y', 'min_x', 'max_x', 'min_y', 'max_y', 'width', 'height', 'area']
# COCO 80-class labels (COCO 인덱스 순서 — 배열 인덱스로 사용)
_Label = Literal[
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck',
    'boat', 'traffic_light', 'fire_hydrant', 'stop_sign', 'parking_meter', 'bench',
    'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra',
    'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
    'skis', 'snowboard', 'sports_ball', 'kite', 'baseball_bat', 'baseball_glove',
    'skateboard', 'surfboard', 'tennis_racket', 'bottle', 'wine_glass', 'cup',
    'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
    'broccoli', 'carrot', 'hot_dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
    'potted_plant', 'bed', 'dining_table', 'toilet', 'tv', 'laptop', 'mouse',
    'remote', 'keyboard', 'cell_phone', 'microwave', 'oven', 'toaster', 'sink',
    'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy_bear', 'hair_drier',
    'toothbrush',
]

# 라벨(언더스코어) → COCO 인덱스. 배열 디바이스 슬롯 위치.
_LABELS = list(get_args(_Label))
_LABEL_INDEX = {label: i for i, label in enumerate(_LABELS)}
_NUM_CLASSES = len(_LABELS)   # 80
_POS_LIST = ('x', 'y', 'min_x', 'max_x', 'min_y', 'max_y', 'width', 'height', 'area')


class ObjectDetection(Robot):
    """Object detection AI extension (COCO 80 classes)."""
    ID = "kr.robomation.virtual.ai.object_detection"
    _instances = {}

    # ── Device IDs (product_id=7 → 0xA07xxxxx). eff 0x0xx / cmd 0x1xx / sensor 0x2xx / event 0x3xx
    # Effectors
    CAMERA_DEVICE      = 0xA0700000
    DISPLAY            = 0xA0700001
    MAX_COUNT          = 0xA0700002
    CONFIDENCE         = 0xA0700003

    # Commands
    LOAD_MODEL         = 0xA0700100
    DETECT_ONCE        = 0xA0700101
    DETECT_CONTINUOUS  = 0xA0700102

    # Sensors
    MODEL_STATE        = 0xA0700200
    DETECTED           = 0xA0700201
    # 좌표는 클래스 인덱스 배열(dimension=80) 디바이스: pos 별 1개 + confidence
    OBJECT_X           = 0xA0700202
    OBJECT_Y           = 0xA0700203
    OBJECT_MIN_X       = 0xA0700204
    OBJECT_MAX_X       = 0xA0700205
    OBJECT_MIN_Y       = 0xA0700206
    OBJECT_MAX_Y       = 0xA0700207
    OBJECT_WIDTH       = 0xA0700208
    OBJECT_HEIGHT      = 0xA0700209
    OBJECT_AREA        = 0xA070020A
    OBJECT_CONFIDENCE  = 0xA070020B

    # Event
    LOAD_MODEL_STATE   = 0xA0700300

    # pos → 배열 디바이스 id
    _POS_DEVICE = {
        'x': OBJECT_X, 'y': OBJECT_Y,
        'min_x': OBJECT_MIN_X, 'max_x': OBJECT_MAX_X,
        'min_y': OBJECT_MIN_Y, 'max_y': OBJECT_MAX_Y,
        'width': OBJECT_WIDTH, 'height': OBJECT_HEIGHT, 'area': OBJECT_AREA,
    }

    # ── Valid values ─────────────────────────────────────────────────────────
    _VALID_DISTANCE_TYPE = get_args(_DistType)
    _VALID_POS           = get_args(_Pos)
    _VALID_OBJECT        = get_args(_Label)

    # ── Robot lifecycle ──────────────────────────────────────────────────────
    def __init__(self, index=0):
        if isinstance(index, str):
            index = 0
        if index in ObjectDetection._instances:
            robot = ObjectDetection._instances[index]
            if robot: robot.dispose()
        ObjectDetection._instances[index] = self
        super(ObjectDetection, self).__init__(ObjectDetection.ID, "ObjectDetection", index)
        self._title = f"ObjectDetection {index}"
        self._init()

    def dispose(self):
        ObjectDetection._instances[self.get_index()] = None
        _display.remove_view(self._title)
        self._roboid._dispose()
        Runner.unregister_robot(self)

    def reset(self):
        self._roboid._reset()

    def _init(self):
        from robomation.ai.object_detection_roboid import ObjectDetectionRoboid
        self._roboid = ObjectDetectionRoboid(self.get_index())
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
        # 인식 결과(박스 + "이름 점수")는 DISPLAY 가 켜져 있고 검출됐을 때만 오버레이.
        if self.read(ObjectDetection.DISPLAY) and engine._has_drawing():
            disp = frame.copy()
            engine._draw_overlay(disp)
            return (self._title, disp)
        return (self._title, frame)

    # ── Public API ───────────────────────────────────────────────────────────
    def device(self, unit):
        from robomation.ai._internal._camera import Camera, resolve
        index = unit._index if isinstance(unit, Camera) else resolve(unit)
        self.write(ObjectDetection.CAMERA_DEVICE, index)

        # 카메라를 붙이면 일단 카메라 화면(프레임)부터 표시한다(모델 로드 전에도).
        if index >= 0:
            _display.add_view(self._title, self._view)
        else:
            _display.remove_view(self._title)

    def load_model(self, wait: bool = True):
        self.write(ObjectDetection.LOAD_MODEL, 1)
        if wait:
            timeout = time.time() + 15
            while self.model_state() != 2 and time.time() < timeout:
                time.sleep(0.01)

    def max_objects(self, data: int):
        self.write(ObjectDetection.MAX_COUNT, data)

    def confidence_threshold(self, data):
        self.write(ObjectDetection.CONFIDENCE, data)

    def detect_once(self):
        self.write(ObjectDetection.DETECT_ONCE, 1)

    def detect_continuous(self):
        self.write(ObjectDetection.DETECT_CONTINUOUS, 1)

    def stop(self):
        self.write(ObjectDetection.DETECT_CONTINUOUS, 0)

    def display(self, on: Union[bool, Literal[1, 0]] = True):
        if on not in (True, False, 1, 0):
            return _err(ObjectDetection, 'display', 'on', on, 'bool | 1 | 0')
        self.write(ObjectDetection.DISPLAY, 1 if on else 0)

    def object(self, unit: _Label, pos: _Pos) -> int:
        if unit not in ObjectDetection._VALID_OBJECT:
            return _err(ObjectDetection, 'object', 'unit', unit, ObjectDetection._VALID_OBJECT)
        if pos not in ObjectDetection._VALID_POS:
            return _err(ObjectDetection, 'object', 'pos', pos, ObjectDetection._VALID_POS)
        return self.read(ObjectDetection._POS_DEVICE[pos], _LABEL_INDEX[unit])

    def get_distance(self, unit1: _Label, unit2: _Label, type: _DistType = None) -> float:
        if unit1 not in ObjectDetection._VALID_OBJECT:
            return _err(ObjectDetection, 'get_distance', 'unit1', unit1, ObjectDetection._VALID_OBJECT)
        if unit2 not in ObjectDetection._VALID_OBJECT:
            return _err(ObjectDetection, 'get_distance', 'unit2', unit2, ObjectDetection._VALID_OBJECT)
        if type is not None and type not in ObjectDetection._VALID_DISTANCE_TYPE:
            return _err(ObjectDetection, 'get_distance', 'type', type, ObjectDetection._VALID_DISTANCE_TYPE)
        i1, i2 = _LABEL_INDEX[unit1], _LABEL_INDEX[unit2]
        dx = self.read(ObjectDetection.OBJECT_X, i2) - self.read(ObjectDetection.OBJECT_X, i1)
        dy = self.read(ObjectDetection.OBJECT_Y, i2) - self.read(ObjectDetection.OBJECT_Y, i1)
        if type is None:
            return math.sqrt(dx * dx + dy * dy)
        if type == 'horizontal':
            return math.fabs(dx)
        return math.fabs(dy)

    def object_detected(self, unit: _Label) -> bool:
        if unit not in ObjectDetection._VALID_OBJECT:
            return _err(ObjectDetection, 'object_detected', 'unit', unit, ObjectDetection._VALID_OBJECT)
        conf = self.read(ObjectDetection.OBJECT_CONFIDENCE, _LABEL_INDEX[unit])
        return conf >= self.read(ObjectDetection.CONFIDENCE)

    def object_confidence(self, unit: _Label) -> float:
        if unit not in ObjectDetection._VALID_OBJECT:
            return _err(ObjectDetection, 'object_confidence', 'unit', unit, ObjectDetection._VALID_OBJECT)
        return self.read(ObjectDetection.OBJECT_CONFIDENCE, _LABEL_INDEX[unit])

    def model_state(self) -> int:
        return self.read(ObjectDetection.MODEL_STATE)

    def detected(self) -> bool:
        return self.read(ObjectDetection.DETECTED) == 1
