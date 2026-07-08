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

# AI 확장 모듈 - ArUco 마커 찾기 (ArUcoMarker)

import math
import time

from robomation.core.error import _err
from robomation.core.runner import Runner
from robomation.core.model import Robot
from robomation.ai._internal import _display
from typing import Literal, Union, get_args


# ── Module-level Literal aliases (for IDE auto-complete) ─────────────────────
_Unit     = Literal['x', 'y', 'min_x', 'max_x', 'min_y', 'max_y', 'width', 'height', 'area', 'rotation']
_DistType = Literal['horizontal', 'vertical']

# DICT_ARUCO_ORIGINAL = 1024 마커(ID 0..1023) → 배열 디바이스 크기
_NUM_MARKERS = 1024
_POS_LIST = ('x', 'y', 'min_x', 'max_x', 'min_y', 'max_y', 'width', 'height', 'area', 'rotation')


class ArucoMarker(Robot):
    """Aruco marker detection AI extension (cv2.aruco)."""
    ID = "kr.robomation.virtual.ai.aruco_marker"
    _instances = {}

    # ── Device IDs (product_id=9 → 0xA09xxxxx). eff 0x0xx / cmd 0x1xx / sensor 0x2xx
    # Effectors
    CAMERA_DEVICE      = 0xA0900000
    DISPLAY            = 0xA0900001
    MAX_COUNT          = 0xA0900002

    # Commands
    DETECT_ONCE        = 0xA0900100
    DETECT_CONTINUOUS  = 0xA0900101

    # Sensors — 마커 ID 인덱스 배열(dimension=1024). unit 별 1개 + detected
    MARKER_X           = 0xA0900200
    MARKER_Y           = 0xA0900201
    MARKER_MIN_X       = 0xA0900202
    MARKER_MAX_X       = 0xA0900203
    MARKER_MIN_Y       = 0xA0900204
    MARKER_MAX_Y       = 0xA0900205
    MARKER_WIDTH       = 0xA0900206
    MARKER_HEIGHT      = 0xA0900207
    MARKER_AREA        = 0xA0900208
    MARKER_ROTATION    = 0xA0900209
    MARKER_DETECTED    = 0xA090020A

    # unit → 배열 디바이스 id
    _POS_DEVICE = {
        'x': MARKER_X, 'y': MARKER_Y,
        'min_x': MARKER_MIN_X, 'max_x': MARKER_MAX_X,
        'min_y': MARKER_MIN_Y, 'max_y': MARKER_MAX_Y,
        'width': MARKER_WIDTH, 'height': MARKER_HEIGHT, 'area': MARKER_AREA,
        'rotation': MARKER_ROTATION,
    }

    # ── Valid values ─────────────────────────────────────────────────────────
    _VALID_UNIT          = get_args(_Unit)
    _VALID_DISTANCE_TYPE = get_args(_DistType)

    # ── Robot lifecycle ──────────────────────────────────────────────────────
    def __init__(self, index=0):
        if isinstance(index, str):
            index = 0
        if index in ArucoMarker._instances:
            robot = ArucoMarker._instances[index]
            if robot: robot.dispose()
        ArucoMarker._instances[index] = self
        super(ArucoMarker, self).__init__(ArucoMarker.ID, "ArucoMarker", index)
        self._title = f"ArucoMarker {index}"
        self._init()

    def dispose(self):
        ArucoMarker._instances[self.get_index()] = None
        _display.remove_view(self._title)
        self._roboid._dispose()
        Runner.unregister_robot(self)

    def reset(self):
        self._roboid._reset()

    def _init(self):
        from robomation.ai.aruco_marker_roboid import ArucoMarkerRoboid
        self._roboid = ArucoMarkerRoboid(self.get_index())
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
        # 인식 결과(마커 외곽선 + ID)는 DISPLAY 가 켜져 있고 검출됐을 때만 오버레이.
        if self.read(ArucoMarker.DISPLAY) and engine._has_drawing():
            disp = frame.copy()
            engine._draw_overlay(disp)
            return (self._title, disp)
        return (self._title, frame)

    # ── Public API ───────────────────────────────────────────────────────────
    def device(self, unit):
        from robomation.ai._internal._camera import Camera, resolve
        index = unit._index if isinstance(unit, Camera) else resolve(unit)
        self.write(ArucoMarker.CAMERA_DEVICE, index)

        # 카메라를 붙이면 일단 카메라 화면(프레임)부터 표시한다.
        if index >= 0:
            _display.add_view(self._title, self._view)
        else:
            _display.remove_view(self._title)

    def max_count(self, data: int):
        self.write(ArucoMarker.MAX_COUNT, data)

    def detect_once(self):
        self.write(ArucoMarker.DETECT_ONCE, 1)

    def detect_continuous(self):
        self.write(ArucoMarker.DETECT_CONTINUOUS, 1)

    def stop(self):
        self.write(ArucoMarker.DETECT_CONTINUOUS, 0)

    def display(self, on: Union[bool, Literal[1, 0]] = True):
        if on not in (True, False, 1, 0):
            return _err(ArucoMarker, 'display', 'on', on, 'bool | 1 | 0')
        self.write(ArucoMarker.DISPLAY, 1 if on else 0)

    def marker(self, data: int, unit: _Unit) -> int:
        if unit not in ArucoMarker._VALID_UNIT:
            return _err(ArucoMarker, 'marker', 'unit', unit, ArucoMarker._VALID_UNIT)
        return self.read(ArucoMarker._POS_DEVICE[unit], data)

    def get_distance(self, unit1: int, unit2: int, type: _DistType = None) -> float:
        if type is not None and type not in ArucoMarker._VALID_DISTANCE_TYPE:
            return _err(ArucoMarker, 'get_distance', 'type', type, ArucoMarker._VALID_DISTANCE_TYPE)
        dx = self.read(ArucoMarker.MARKER_X, unit1) - self.read(ArucoMarker.MARKER_X, unit2)
        dy = self.read(ArucoMarker.MARKER_Y, unit1) - self.read(ArucoMarker.MARKER_Y, unit2)
        if type is None:
            return math.sqrt(dx * dx + dy * dy)
        if type == 'horizontal':
            return math.fabs(dx)
        return math.fabs(dy)

    def marker_detected(self, data: int) -> bool:
        return self.read(ArucoMarker.MARKER_DETECTED, data) == 1
