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

# AI 확장 모듈 - 색깔 찾기(ColorDetection)

from robomation.core.error import _err
from robomation.core.runner import Runner
from robomation.core.model import Robot
from robomation.ai._internal import _display
from typing import Literal, Union, get_args


# ── Module-level Literal aliases (for IDE auto-complete) ─────────────────────
_Pos   = Literal['x', 'y', 'min_x', 'max_x', 'min_y', 'max_y', 'width', 'height', 'area']
_Color = Literal['black', 'red', 'yellow', 'green', 'cyan', 'blue', 'magenta', 'white']


class ColorDetection(Robot):
    """Color detection AI extension (cv2 HSV, 8 colors)."""
    ID = "kr.robomation.virtual.ai.color_detection"
    _instances = {}

    # ── Device IDs (product_id=8 → 0xA08xxxxx). eff 0x0xx / cmd 0x1xx / sensor 0x2xx
    # Effectors
    CAMERA_DEVICE      = 0xA0800000
    DISPLAY            = 0xA0800001
    AREA_CONDITION     = 0xA0800002

    # Commands (register/delete 는 .d 에 색 인덱스를 담는다)
    REGISTER_COLOR     = 0xA0800100
    DELETE_COLOR       = 0xA0800101
    DETECT_ONCE        = 0xA0800102
    DETECT_CONTINUOUS  = 0xA0800103

    # Sensors
    DETECTED           = 0xA0800200
    # 좌표는 색 인덱스 배열(dimension=8) 디바이스: pos 별 1개
    COLOR_X            = 0xA0800201
    COLOR_Y            = 0xA0800202
    COLOR_MIN_X        = 0xA0800203
    COLOR_MAX_X        = 0xA0800204
    COLOR_MIN_Y        = 0xA0800205
    COLOR_MAX_Y        = 0xA0800206
    COLOR_WIDTH        = 0xA0800207
    COLOR_HEIGHT       = 0xA0800208
    COLOR_AREA         = 0xA0800209
    COLOR_DETECTED     = 0xA080020A

    # pos → 배열 디바이스 id
    _POS_DEVICE = {
        'x': COLOR_X,           'y': COLOR_Y,
        'min_x': COLOR_MIN_X,   'max_x': COLOR_MAX_X,
        'min_y': COLOR_MIN_Y,   'max_y': COLOR_MAX_Y,
        'width': COLOR_WIDTH,   'height': COLOR_HEIGHT,     'area': COLOR_AREA,
    }

    # ── Valid values ─────────────────────────────────────────────────────────
    _VALID_POS   = get_args(_Pos)
    _VALID_COLOR = {c: i for i, c in enumerate(get_args(_Color))}

    # ── Robot lifecycle ──────────────────────────────────────────────────────
    def __init__(self, index=0):
        if isinstance(index, str):
            index = 0
        if index in ColorDetection._instances:
            robot = ColorDetection._instances[index]
            if robot: robot.dispose()
        ColorDetection._instances[index] = self
        super(ColorDetection, self).__init__(ColorDetection.ID, "ColorDetection", index)
        self._title = f"ColorDetection {index}"
        self._init()

    def dispose(self):
        ColorDetection._instances[self.get_index()] = None
        _display.remove_view(self._title)
        self._roboid._dispose()
        Runner.unregister_robot(self)

    def reset(self):
        self._roboid._reset()

    def _init(self):
        from robomation.ai.color_detection_roboid import ColorDetectionRoboid
        self._roboid = ColorDetectionRoboid(self.get_index())
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
        # 인식 결과(색 박스 + 이름)는 DISPLAY 가 켜져 있고 검출됐을 때만 오버레이.
        if self.read(ColorDetection.DISPLAY) and engine._has_drawing():
            disp = frame.copy()
            engine._draw_overlay(disp)
            return (self._title, disp)
        return (self._title, frame)

    # ── Public API ───────────────────────────────────────────────────────────
    def device(self, unit):
        from robomation.ai._internal._camera import Camera, resolve
        index = unit._index if isinstance(unit, Camera) else resolve(unit)
        self.write(ColorDetection.CAMERA_DEVICE, index)

        # 카메라를 붙이면 일단 카메라 화면(프레임)부터 표시한다.
        if index >= 0:
            _display.add_view(self._title, self._view)
        else:
            _display.remove_view(self._title)

    def register_color(self, color: _Color, wait: bool = True):
        if color not in ColorDetection._VALID_COLOR:
            return _err(ColorDetection, 'register_color', 'color', color, tuple(ColorDetection._VALID_COLOR))
        idx = ColorDetection._VALID_COLOR[color]
        self.write(ColorDetection.REGISTER_COLOR, idx)
        if wait:
            # 이전 등록이 엔진에 실제 반영된 뒤 다음 호출이 진행되도록 대기.
            self._roboid._wait_color(idx, True)

    def delete_color(self, color: _Color, wait: bool = True):
        if color not in ColorDetection._VALID_COLOR:
            return _err(ColorDetection, 'delete_color', 'color', color, tuple(ColorDetection._VALID_COLOR))
        idx = ColorDetection._VALID_COLOR[color]
        self.write(ColorDetection.DELETE_COLOR, idx)
        if wait:
            # 이전 삭제가 엔진에 실제 반영된 뒤 다음 호출이 진행되도록 대기.
            self._roboid._wait_color(idx, False)

    def area_condition(self, data):
        self.write(ColorDetection.AREA_CONDITION, data)

    def detect_once(self):
        self.write(ColorDetection.DETECT_ONCE, 1)

    def detect_continuous(self):
        self.write(ColorDetection.DETECT_CONTINUOUS, 1)

    def stop(self):
        self.write(ColorDetection.DETECT_CONTINUOUS, 0)

    def display(self, on: Union[bool, Literal[1, 0]] = True):
        if on not in (True, False, 1, 0):
            return _err(ColorDetection, 'display', 'on', on, 'bool | 1 | 0')
        self.write(ColorDetection.DISPLAY, 1 if on else 0)

    def color(self, color: _Color, pos: _Pos) -> int:
        if color not in ColorDetection._VALID_COLOR:
            return _err(ColorDetection, 'color', 'color', color, tuple(ColorDetection._VALID_COLOR))
        if pos not in ColorDetection._VALID_POS:
            return _err(ColorDetection, 'color', 'pos', pos, ColorDetection._VALID_POS)
        return self.read(ColorDetection._POS_DEVICE[pos], ColorDetection._VALID_COLOR[color])

    def color_detected(self, color: _Color) -> bool:
        if color not in ColorDetection._VALID_COLOR:
            return _err(ColorDetection, 'color_detected', 'color', color, tuple(ColorDetection._VALID_COLOR))
        idx = ColorDetection._VALID_COLOR[color]
        return self.read(ColorDetection.COLOR_DETECTED, idx) == 1
        # return self.read(ColorDetection.COLOR_AREA, idx) >= self.read(ColorDetection.AREA_CONDITION)
