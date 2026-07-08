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

# AI 확장 모듈 - 나이/성별/표정 찾기(FaceExpression) 

import time

from robomation.core.error import _err
from robomation.core.runner import Runner
from robomation.core.model import Robot
from robomation.ai._internal import _display
from typing import Literal, Union, get_args


# ── Module-level Literal aliases (for IDE auto-complete) ─────────────────────
_Gender = Literal['male', 'female']
_Type   = Literal['angry', 'disgusted', 'fearful', 'happy', 'neutral', 'sad', 'surprised']


class FaceExpression(Robot):
    """Face expression / age / gender AI extension."""
    ID = "kr.robomation.virtual.ai.face_expression"
    _instances = {}

    # ── Device IDs (product_id=4 → 0xA04xxxxx).
    # Effectors
    CAMERA_DEVICE       = 0xA0400000
    DISPLAY             = 0xA0400001

    # Commands
    LOAD_MODEL          = 0xA0400100
    DETECT_ONCE         = 0xA0400101
    DETECT_CONTINUOUS   = 0xA0400102

    # Sensors
    MODEL_STATE         = 0xA0400200
    AGE                 = 0xA0400201
    GENDER_DETECTED     = 0xA0400202
    GENDER_CLASS        = 0xA0400203   # STRING ('male'/'female'/'')
    EXPRESSION_DETECTED = 0xA0400204
    EXPRESSION_CLASS    = 0xA0400205   # STRING

    # 신뢰도 (label → device id). FLOAT 0..1
    _GENDER_CONF_DEVICE = {
        'male':         0xA0400206,     'female':       0xA0400207,
    }
    _EXPRESSION_CONF_DEVICE = {
        'angry':        0xA0400208,     'disgusted':    0xA0400209,    'fearful':   0xA040020A,
        'happy':        0xA040020B,     'neutral':      0xA040020C,    'sad':       0xA040020D,
        'surprised':    0xA040020E,
    }

    # Event
    LOAD_MODEL_STATE    = 0xA0400300

    # ── Valid values ─────────────────────────────────────────────────────────
    _VALID_GENDER     = get_args(_Gender)
    _VALID_EXPRESSION = get_args(_Type)

    # ── Robot lifecycle ──────────────────────────────────────────────────────
    def __init__(self, index=0):
        if isinstance(index, str):
            index = 0
        if index in FaceExpression._instances:
            robot = FaceExpression._instances[index]
            if robot: robot.dispose()
        FaceExpression._instances[index] = self
        super(FaceExpression, self).__init__(FaceExpression.ID, "FaceExpression", index)
        self._title = f"FaceExpression {index}"
        self._init()

    def dispose(self):
        FaceExpression._instances[self.get_index()] = None
        _display.remove_view(self._title)
        self._roboid._dispose()
        Runner.unregister_robot(self)

    def reset(self):
        self._roboid._reset()

    def _init(self):
        from robomation.ai.face_expression_roboid import FaceExpressionRoboid
        self._roboid = FaceExpressionRoboid(self.get_index())
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
        # 인식 결과(박스 + "나이, 성별, 표정" 텍스트)는 DISPLAY 가 켜져 있고 검출됐을 때만.
        if self.read(FaceExpression.DISPLAY) and engine._has_drawing():
            disp = frame.copy()
            engine._draw_overlay(disp)
            return (self._title, disp)
        return (self._title, frame)

    # ── Public API ───────────────────────────────────────────────────────────
    def device(self, unit):
        from robomation.ai._internal._camera import Camera, resolve
        index = unit._index if isinstance(unit, Camera) else resolve(unit)
        self.write(FaceExpression.CAMERA_DEVICE, index)

        # 카메라를 붙이면 일단 카메라 화면(프레임)부터 표시한다(모델 로드 전에도).
        if index >= 0:
            _display.add_view(self._title, self._view)
        else:
            _display.remove_view(self._title)

    def load_model(self, wait: bool = True):
        self.write(FaceExpression.LOAD_MODEL, 1)
        if wait:
            timeout = time.time() + 30   # 3개 모델 다운로드 가능성 → 여유
            while self.model_state() != 2 and time.time() < timeout:
                time.sleep(0.01)

    def detect_once(self):
        self.write(FaceExpression.DETECT_ONCE, 1)

    def detect_continuous(self):
        self.write(FaceExpression.DETECT_CONTINUOUS, 1)

    def stop(self):
        self.write(FaceExpression.DETECT_CONTINUOUS, 0)

    def display(self, on: Union[bool, Literal[1, 0]] = True):
        if on not in (True, False, 1, 0):
            return _err(FaceExpression, 'display', 'on', on, 'bool | 1 | 0')
        self.write(FaceExpression.DISPLAY, 1 if on else 0)

    def age(self) -> int:
        return self.read(FaceExpression.AGE)

    def gender(self) -> str:
        return self.read(FaceExpression.GENDER_CLASS)

    def expression(self) -> str:
        return self.read(FaceExpression.EXPRESSION_CLASS)

    def gender_detected(self) -> bool:
        return self.read(FaceExpression.GENDER_DETECTED) == 1

    def is_gender(self, unit: _Gender) -> bool:
        if unit not in FaceExpression._VALID_GENDER:
            return _err(FaceExpression, 'is_gender', 'unit', unit, FaceExpression._VALID_GENDER)
        return self.read(FaceExpression.GENDER_CLASS) == unit

    def gender_confidence(self, unit: _Gender) -> float:
        if unit not in FaceExpression._VALID_GENDER:
            return _err(FaceExpression, 'gender_confidence', 'unit', unit, FaceExpression._VALID_GENDER)
        return self.read(FaceExpression._GENDER_CONF_DEVICE[unit])

    def expression_detected(self) -> bool:
        return self.read(FaceExpression.EXPRESSION_DETECTED) == 1

    def is_expression(self, unit: _Type) -> bool:
        if unit not in FaceExpression._VALID_EXPRESSION:
            return _err(FaceExpression, 'is_expression', 'unit', unit, FaceExpression._VALID_EXPRESSION)
        return self.read(FaceExpression.EXPRESSION_CLASS) == unit

    def expression_confidence(self, unit: _Type) -> float:
        if unit not in FaceExpression._VALID_EXPRESSION:
            return _err(FaceExpression, 'expression_confidence', 'unit', unit, FaceExpression._VALID_EXPRESSION)
        return self.read(FaceExpression._EXPRESSION_CONF_DEVICE[unit])

    def model_state(self) -> int:
        return self.read(FaceExpression.MODEL_STATE)
