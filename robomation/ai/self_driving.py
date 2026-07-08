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

# AI 확장 모듈 - 카메라 자율주행하기(SelfDriving)

import math

from robomation.core.error import _err
from robomation.core.runner import Runner
from robomation.core.model import Robot
from robomation.ai._internal import _display
from typing import Literal, Union, get_args


# ── Module-level Literal aliases (for IDE auto-complete) ─────────────────────
_Lane       = Literal['left', 'right']
_LaneUnit   = Literal['x', 'distance']
_Color      = Literal['red', 'green', 'blue']
_ColorUnit  = Literal['x', 'y', 'min_x', 'max_x', 'min_y', 'max_y', 'width', 'height', 'area']
_DistType   = Literal['line', 'horizontal', 'vertical']
_LaneOrAny  = Literal['left', 'right', 'any', 'both']
_ColorOrAny = Literal['red', 'green', 'blue', 'any']

_COLOR_INDEX = {'red': 0, 'green': 1, 'blue': 2}
_NUM_COLORS = len(_COLOR_INDEX)   # 3
_COLOR_UNIT_LIST = ('x', 'y', 'min_x', 'max_x', 'min_y', 'max_y', 'width', 'height', 'area')


class SelfDriving(Robot):
    """Self-driving AI extension (cv2 HSV lane color detection)."""
    ID = "kr.robomation.virtual.ai.self_driving"
    _instances = {}

    # ── Device IDs (product_id=10 → 0xA0Axxxxx). eff 0x0xx / cmd 0x1xx / sensor 0x2xx
    # Effectors
    CAMERA_DEVICE      = 0xA0A00000
    DISPLAY            = 0xA0A00001
    LANE_LEFT_COLOR    = 0xA0A00002   # STRING ('red'/'green'/'blue')
    LANE_RIGHT_COLOR   = 0xA0A00003

    # Commands
    DETECT_ONCE        = 0xA0A00100
    DETECT_CONTINUOUS  = 0xA0A00101

    # Sensors — 색(red/green/blue) 인덱스 배열(dimension=3)
    COLOR_X            = 0xA0A00200
    COLOR_Y            = 0xA0A00201
    COLOR_MIN_X        = 0xA0A00202
    COLOR_MAX_X        = 0xA0A00203
    COLOR_MIN_Y        = 0xA0A00204
    COLOR_MAX_Y        = 0xA0A00205
    COLOR_WIDTH        = 0xA0A00206
    COLOR_HEIGHT       = 0xA0A00207
    COLOR_AREA         = 0xA0A00208
    COLOR_DETECTED     = 0xA0A00209
    # 차선(좌/우) 스칼라
    LANE_LEFT_X        = 0xA0A0020A
    LANE_LEFT_DISTANCE = 0xA0A0020B
    LANE_LEFT_DETECTED = 0xA0A0020C
    LANE_RIGHT_X       = 0xA0A0020D
    LANE_RIGHT_DISTANCE= 0xA0A0020E
    LANE_RIGHT_DETECTED= 0xA0A0020F

    _COLOR_POS_DEVICE = {
        'x': COLOR_X, 'y': COLOR_Y, 'min_x': COLOR_MIN_X, 'max_x': COLOR_MAX_X,
        'min_y': COLOR_MIN_Y, 'max_y': COLOR_MAX_Y, 'width': COLOR_WIDTH,
        'height': COLOR_HEIGHT, 'area': COLOR_AREA,
    }
    _LANE_DEVICE = {
        ('left', 'x'): LANE_LEFT_X, ('left', 'distance'): LANE_LEFT_DISTANCE,
        ('right', 'x'): LANE_RIGHT_X, ('right', 'distance'): LANE_RIGHT_DISTANCE,
    }
    _LANE_DETECTED_DEVICE = {'left': LANE_LEFT_DETECTED, 'right': LANE_RIGHT_DETECTED}

    # ── Valid values ─────────────────────────────────────────────────────────
    _VALID_LANE          = get_args(_Lane)
    _VALID_LANE_UNIT     = get_args(_LaneUnit)
    _VALID_COLOR         = _COLOR_INDEX
    _VALID_COLOR_UNIT    = get_args(_ColorUnit)
    _VALID_DISTANCE_TYPE = get_args(_DistType)

    # ── Robot lifecycle ──────────────────────────────────────────────────────
    def __init__(self, index=0):
        if isinstance(index, str):
            index = 0
        if index in SelfDriving._instances:
            robot = SelfDriving._instances[index]
            if robot: robot.dispose()
        SelfDriving._instances[index] = self
        super(SelfDriving, self).__init__(SelfDriving.ID, "SelfDriving", index)
        self._title = f"SelfDriving {index}"
        self._init()

    def dispose(self):
        SelfDriving._instances[self.get_index()] = None
        _display.remove_view(self._title)
        self._roboid._dispose()
        Runner.unregister_robot(self)

    def reset(self):
        self._roboid._reset()

    def _init(self):
        from robomation.ai.self_driving_roboid import SelfDrivingRoboid
        self._roboid = SelfDrivingRoboid(self.get_index())
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
        # 인식 결과(색 박스 + 좌표)는 DISPLAY 가 켜져 있고 검출됐을 때만 오버레이.
        if self.read(SelfDriving.DISPLAY) and engine._has_drawing():
            disp = frame.copy()
            engine._draw_overlay(disp)
            return (self._title, disp)
        return (self._title, frame)

    # ── Public API ───────────────────────────────────────────────────────────
    def device(self, unit):
        from robomation.ai._internal._camera import Camera, resolve
        index = unit._index if isinstance(unit, Camera) else resolve(unit)
        self.write(SelfDriving.CAMERA_DEVICE, index)

        # 카메라를 붙이면 일단 카메라 화면(프레임)부터 표시한다.
        if index >= 0:
            _display.add_view(self._title, self._view)
        else:
            _display.remove_view(self._title)

    def set_lane(self, left: _Color, right: _Color):
        if left not in SelfDriving._VALID_COLOR:
            return _err(SelfDriving, 'set_lane', 'left', left, tuple(SelfDriving._VALID_COLOR))
        if right not in SelfDriving._VALID_COLOR:
            return _err(SelfDriving, 'set_lane', 'right', right, tuple(SelfDriving._VALID_COLOR))
        self.write(SelfDriving.LANE_LEFT_COLOR, left)
        self.write(SelfDriving.LANE_RIGHT_COLOR, right)

    def detect_once(self):
        self.write(SelfDriving.DETECT_ONCE, 1)

    def detect_continuous(self):
        self.write(SelfDriving.DETECT_CONTINUOUS, 1)

    def stop(self):
        self.write(SelfDriving.DETECT_CONTINUOUS, 0)

    def display(self, on: Union[bool, Literal[1, 0]] = True):
        if on not in (True, False, 1, 0):
            return _err(SelfDriving, 'display', 'on', on, 'bool | 1 | 0')
        self.write(SelfDriving.DISPLAY, 1 if on else 0)

    def lane(self, lane: _Lane, unit: _LaneUnit) -> int:
        if lane not in SelfDriving._VALID_LANE:
            return _err(SelfDriving, 'lane', 'lane', lane, SelfDriving._VALID_LANE)
        if unit not in SelfDriving._VALID_LANE_UNIT:
            return _err(SelfDriving, 'lane', 'unit', unit, SelfDriving._VALID_LANE_UNIT)
        return self.read(SelfDriving._LANE_DEVICE[(lane, unit)])

    def color(self, color: _Color, unit: _ColorUnit) -> int:
        if color not in SelfDriving._VALID_COLOR:
            return _err(SelfDriving, 'color', 'color', color, tuple(SelfDriving._VALID_COLOR))
        if unit not in SelfDriving._VALID_COLOR_UNIT:
            return _err(SelfDriving, 'color', 'unit', unit, SelfDriving._VALID_COLOR_UNIT)
        return self.read(SelfDriving._COLOR_POS_DEVICE[unit], SelfDriving._VALID_COLOR[color])

    def get_distance(self, unit1: _Color, unit2: _Color, type: _DistType = None) -> float:
        if unit1 not in SelfDriving._VALID_COLOR:
            return _err(SelfDriving, 'get_distance', 'unit1', unit1, tuple(SelfDriving._VALID_COLOR))
        if unit2 not in SelfDriving._VALID_COLOR:
            return _err(SelfDriving, 'get_distance', 'unit2', unit2, tuple(SelfDriving._VALID_COLOR))
        if type is not None and type not in SelfDriving._VALID_DISTANCE_TYPE:
            return _err(SelfDriving, 'get_distance', 'type', type, SelfDriving._VALID_DISTANCE_TYPE)
        i1, i2 = SelfDriving._VALID_COLOR[unit1], SelfDriving._VALID_COLOR[unit2]
        dx = self.read(SelfDriving.COLOR_X, i2) - self.read(SelfDriving.COLOR_X, i1)
        dy = self.read(SelfDriving.COLOR_Y, i2) - self.read(SelfDriving.COLOR_Y, i1)
        if type is None or type == 'line':
            return math.sqrt(dx * dx + dy * dy)
        if type == 'horizontal':
            return math.fabs(dx)
        return math.fabs(dy)

    def lane_detected(self, lane: _LaneOrAny = 'any') -> bool:
        left = self.read(SelfDriving.LANE_LEFT_DETECTED) == 1
        right = self.read(SelfDriving.LANE_RIGHT_DETECTED) == 1
        if lane == 'any':
            return left or right
        if lane == 'both':
            return left and right
        if lane not in SelfDriving._VALID_LANE:
            return _err(SelfDriving, 'lane_detected', 'lane', lane, SelfDriving._VALID_LANE)
        return left if lane == 'left' else right

    def color_detected(self, color: _ColorOrAny = 'any') -> bool:
        if color == 'any':
            return any(self.read(SelfDriving.COLOR_DETECTED, i) == 1 for i in range(_NUM_COLORS))
        if color not in SelfDriving._VALID_COLOR:
            return _err(SelfDriving, 'color_detected', 'color', color, tuple(SelfDriving._VALID_COLOR))
        return self.read(SelfDriving.COLOR_DETECTED, SelfDriving._VALID_COLOR[color]) == 1
