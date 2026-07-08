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

import sys
import time
import threading

import cv2
import numpy as np

from robomation.core.runner import Runner
from robomation.core.model import DeviceType, DataType, Roboid
from robomation.ai._internal._camera import _CaptureSource
from robomation.ai.self_driving import _COLOR_INDEX, _COLOR_UNIT_LIST, _NUM_COLORS


_IDX_TO_COLOR = {v: k for k, v in _COLOR_INDEX.items()}

# 색 HSV 규칙 (OpenCV 스케일 H 0..179; 브라우저 h*30 과 동일). s>50, v>50.
_COLOR_RULES = {
    'red':   [((0, 50, 50), (10, 255, 255)), ((170, 50, 50), (180, 255, 255))],
    'green': [((40, 50, 50), (80, 255, 255))],
    'blue':  [((100, 50, 50), (140, 255, 255))],
}
_DRAW_BGR = {'red': (0, 0, 255), 'green': (0, 255, 0), 'blue': (255, 0, 0)}
_KERNEL = np.ones((3, 3), np.uint8)
_BAND = 50          # 하단 관심영역 높이 (브라우저 thresholdY = h - 50)
_MIN_DIM = 5        # 최소 블롭 크기 (브라우저 setMinDimension(5))


def _blank_arrays():
    arrays = {unit: [0] * _NUM_COLORS for unit in _COLOR_UNIT_LIST}
    detected = [0] * _NUM_COLORS
    return arrays, detected


class _SelfDrivingEngine:
    """cv2 HSV 차선 색검출 + 바인딩된 카메라(_CaptureSource) 래핑. (ML 모델 없음)"""

    def __init__(self):
        self._source = None
        self._camera_index = -1
        self._left_color = ''
        self._right_color = ''
        self._detected = False
        self._one_detection = False
        self._continuous = False
        self._arrays, self._color_detected = _blank_arrays()
        self._lane = {'left': (0, 0, 0), 'right': (0, 0, 0)}   # (x, distance, detected)
        self._draw = []            # [(ix, iy, w, h, color_name), ...]
        self._opened = False
        self._closed = False
        self._thread = None
        self._lock = threading.Lock()
        self._detect_lock = threading.Lock()

    def _open(self):
        self._opened = True
        return True

    def _is_open(self):
        return self._opened

    # ── 카메라 바인딩 ──
    def _bind_camera(self, index):
        if index == self._camera_index:
            return
        if self._source is not None:
            self._source.release()
            self._source = None
        self._camera_index = index
        if index is not None and index >= 0:
            self._source = _CaptureSource.acquire(index)
            self._check_permission(self._source)

    def _read_frame(self):
        if self._source is None:
            return None
        return self._source.read()

    def _set_lane_colors(self, left, right):
        self._left_color = left
        self._right_color = right

    # ── 검출 모드 ──
    def _detect_once(self):
        self._one_detection = True
        frame = self._read_frame()
        if frame is not None:
            self._process_frame(frame)

    def _set_mode(self, continuous):
        if continuous:
            self._continuous = True
            if self._thread is None:
                self._thread = threading.Thread(target=self._continuous_run)
                self._thread.daemon = True
                self._thread.start()
        else:
            self._continuous = False
            thread = self._thread
            self._thread = None
            if thread:
                thread.join(timeout=1)

    def _continuous_run(self):
        while self._continuous and not self._closed:
            frame = self._read_frame()
            if frame is not None:
                self._process_frame(frame)
            time.sleep(1 / 15)

    def _process_frame(self, frame):
        h, w = frame.shape[:2]
        hw, hh = w / 2, h / 2
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        band_top = max(0, h - _BAND)   # 하단 관심영역만
        arrays, color_detected = _blank_arrays()
        draw = []
        for name in ('red', 'green', 'blue'):
            blob = self._largest_blob(hsv, name, band_top)
            if blob is None:
                continue
            ix, iy, bw, bh = blob
            idx = _COLOR_INDEX[name]
            tlx = int(ix - hw); brx = tlx + bw
            tly = int(hh - iy);  bry = tly - bh
            arrays['x'][idx] = int((tlx + brx) / 2)
            arrays['y'][idx] = int((tly + bry) / 2)
            arrays['min_x'][idx] = tlx
            arrays['max_x'][idx] = brx
            arrays['min_y'][idx] = tly
            arrays['max_y'][idx] = bry
            arrays['width'][idx] = bw
            arrays['height'][idx] = bh
            arrays['area'][idx] = bw * bh
            color_detected[idx] = 1
            draw.append((ix, iy, bw, bh, name))

        # 차선 매핑 (기본값: 미검출 시 좌 x=0/우 x=w, distance=w/2)
        lane = {'left': (0, int(hw), 0), 'right': (int(w), int(hw), 0)}
        if self._left_color and color_detected[_COLOR_INDEX.get(self._left_color, 0)]:
            lx = arrays['x'][_COLOR_INDEX[self._left_color]]
            lane['left'] = (lx, abs(lx), 1)
        if self._right_color and color_detected[_COLOR_INDEX.get(self._right_color, 0)]:
            rx = arrays['x'][_COLOR_INDEX[self._right_color]]
            lane['right'] = (rx, abs(rx), 1)

        any_det = any(color_detected)
        with self._lock:
            self._arrays = arrays
            self._color_detected = color_detected
            self._lane = lane
            self._draw = draw
            self._detected = any_det

    def _largest_blob(self, hsv, name, band_top):
        mask = None
        for lower, upper in _COLOR_RULES[name]:
            m = cv2.inRange(hsv, np.array(lower, np.uint8), np.array(upper, np.uint8))
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        mask[:band_top, :] = 0          # 하단 관심영역만 남김
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _KERNEL)
        num, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        best = None
        best_area = 0
        for i in range(1, num):
            bw = int(stats[i, cv2.CC_STAT_WIDTH])
            bh = int(stats[i, cv2.CC_STAT_HEIGHT])
            if bw < _MIN_DIM or bh < _MIN_DIM:
                continue
            area = int(stats[i, cv2.CC_STAT_AREA])
            if area > best_area:
                best_area = area
                best = (int(stats[i, cv2.CC_STAT_LEFT]), int(stats[i, cv2.CC_STAT_TOP]), bw, bh)
        return best

    # ── 조회 (lock 스냅샷) ──
    def _is_detected(self):
        with self._lock:
            return self._detected and (self._one_detection or self._continuous)

    def _get_data(self):
        return self._arrays, self._color_detected, self._lane

    def _has_drawing(self):
        with self._lock:
            return len(self._draw) > 0

    def _draw_overlay(self, frame):
        with self._lock:
            draws = list(self._draw)
        for ix, iy, w, h, name in draws:
            color = _DRAW_BGR[name]
            cv2.rectangle(frame, (ix, iy), (ix + w, iy + h), color, 2)
            cv2.putText(frame, name, (ix, iy - 6 if iy > 14 else iy + 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2, cv2.LINE_AA)

    def _check_permission(self, source):
        if sys.platform != 'darwin':
            return
        thread = threading.Thread(target=self._permission_watch, args=(source,))
        thread.daemon = True
        thread.start()

    def _permission_watch(self, source):
        deadline = time.time() + 3.0
        while time.time() < deadline:
            if self._closed or source is not self._source:
                return
            frame = source.read()
            if frame is not None and frame.max() > 0:
                return
            time.sleep(0.05)
        if self._closed or source is not self._source:
            return
        from robomation.core import permission
        print(
            "[SelfDriving] 카메라 입력이 감지되지 않습니다.\n"
            "      macOS 라면 Python 을 실행한 앱(VSCode/터미널 등)의 카메라 권한이 꺼져 있을 수 있습니다.\n"
            "      시스템 설정 → 개인정보 보호 및 보안 → 카메라 에서 해당 앱을 허용한 뒤,\n"
            "      그 앱을 완전히 종료했다가 다시 실행하세요.",
            file=sys.stderr,
        )
        permission.open_camera_settings()

    def _close(self):
        self._closed = True
        self._set_mode(False)
        if self._source is not None:
            self._source.release()
            self._source = None
        self._opened = False


class SelfDrivingRoboid(Roboid):
    def __init__(self, index):
        from robomation.ai.self_driving import SelfDriving
        super(SelfDrivingRoboid, self).__init__(SelfDriving.ID, "SelfDriving", 0xA0A00000)
        self._index = index
        self._engine = None
        self._ready = False
        self._running = False
        self._releasing = 0
        self._thread = None
        self._thread_lock = threading.Lock()

        # ── Motoring 스냅샷 ──
        self._camera_index = -1
        self._display = 0
        self._left_color = ''
        self._right_color = ''
        self._detect_once_written = False
        self._detect_continuous = 0
        self._detect_continuous_written = False

        self._create_model()

    def _create_model(self):
        from robomation.ai.self_driving import SelfDriving
        dict = self._device_dict = {}
        # Effectors
        dict[SelfDriving.CAMERA_DEVICE]    = self._camera_device     = self._add_device(SelfDriving.CAMERA_DEVICE, "CameraDevice", DeviceType.EFFECTOR, DataType.INTEGER, 1, -1, 1000, -1)
        dict[SelfDriving.DISPLAY]          = self._display_device    = self._add_device(SelfDriving.DISPLAY, "Display", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 1)
        dict[SelfDriving.LANE_LEFT_COLOR]  = self._left_color_device  = self._add_device(SelfDriving.LANE_LEFT_COLOR,  "LaneLeftColor",  DeviceType.EFFECTOR, DataType.STRING, 1, 0, 0, '')
        dict[SelfDriving.LANE_RIGHT_COLOR] = self._right_color_device = self._add_device(SelfDriving.LANE_RIGHT_COLOR, "LaneRightColor", DeviceType.EFFECTOR, DataType.STRING, 1, 0, 0, '')

        # Commands
        dict[SelfDriving.DETECT_ONCE]       = self._detect_once_device       = self._add_device(SelfDriving.DETECT_ONCE,       "DetectOnce",       DeviceType.COMMAND, DataType.INTEGER, 1, 0, 1, 0)
        dict[SelfDriving.DETECT_CONTINUOUS] = self._detect_continuous_device = self._add_device(SelfDriving.DETECT_CONTINUOUS, "DetectContinuous", DeviceType.COMMAND, DataType.INTEGER, 1, 0, 1, 0)

        # Sensors — 색(red/green/blue) 인덱스 배열(dimension = 3)
        _range = [-10000, 10000]
        self._pos_devices = []
        for unit in _COLOR_UNIT_LIST:
            dev_id = SelfDriving._COLOR_POS_DEVICE[unit]
            dev = self._add_device(dev_id, "Color_" + unit, DeviceType.SENSOR, DataType.INTEGER, _NUM_COLORS, _range[0], _range[1] * _range[1], 0)
            dict[dev_id] = dev
            self._pos_devices.append((dev, unit))
        dict[SelfDriving.COLOR_DETECTED] = self._color_detected_device = self._add_device(SelfDriving.COLOR_DETECTED, "Color_detected", DeviceType.SENSOR, DataType.INTEGER, _NUM_COLORS, 0, 1, 0)

        # 차선(좌/우) 스칼라 센서
        dict[SelfDriving.LANE_LEFT_X]         = self._left_x_device      = self._add_device(SelfDriving.LANE_LEFT_X,         "LaneLeftX",        DeviceType.SENSOR, DataType.INTEGER, 1, _range[0], _range[1], 0)
        dict[SelfDriving.LANE_LEFT_DISTANCE]  = self._left_dist_device   = self._add_device(SelfDriving.LANE_LEFT_DISTANCE,  "LaneLeftDistance", DeviceType.SENSOR, DataType.INTEGER, 1, 0, _range[1], 0)
        dict[SelfDriving.LANE_LEFT_DETECTED]  = self._left_det_device    = self._add_device(SelfDriving.LANE_LEFT_DETECTED,  "LaneLeftDetected", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[SelfDriving.LANE_RIGHT_X]        = self._right_x_device     = self._add_device(SelfDriving.LANE_RIGHT_X,        "LaneRightX",       DeviceType.SENSOR, DataType.INTEGER, 1, _range[0], _range[1], 0)
        dict[SelfDriving.LANE_RIGHT_DISTANCE] = self._right_dist_device  = self._add_device(SelfDriving.LANE_RIGHT_DISTANCE, "LaneRightDistance",DeviceType.SENSOR, DataType.INTEGER, 1, 0, _range[1], 0)
        dict[SelfDriving.LANE_RIGHT_DETECTED] = self._right_det_device   = self._add_device(SelfDriving.LANE_RIGHT_DETECTED, "LaneRightDetected",DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)

    def _find_device_by_id(self, device_id):
        return self._device_dict.get(device_id)

    def _run(self):
        try:
            while self._running or self._releasing > 0:
                if self._decode():
                    self._encode()
                    if self._releasing > 0:
                        self._releasing -= 1
                time.sleep(0.01)
        except Exception:
            import traceback
            traceback.print_exc()

    def _init(self):
        Runner.register_required()
        self._running = True
        self._releasing = 0
        thread = threading.Thread(target=self._run)
        self._thread = thread
        thread.daemon = True
        thread.start()

        self._engine = _SelfDrivingEngine()
        if self._engine._open():
            while self._ready == False and self._is_disposed() == False:
                time.sleep(0.01)
        else:
            Runner.register_checked()

    def _release(self):
        if self._ready:
            self._releasing = 5
        self._running = False
        thread = self._thread
        self._thread = None
        if thread:
            thread.join()
        engine = self._engine
        self._engine = None
        if engine:
            engine._close()

    def _dispose(self):
        if self._is_disposed() == False:
            super(SelfDrivingRoboid, self)._dispose()
            self._release()

    def _reset(self):
        super(SelfDrivingRoboid, self)._reset()
        self._camera_index = -1
        self._display = 0
        self._left_color = ''
        self._right_color = ''
        self._detect_once_written = False
        self._detect_continuous = 0
        self._detect_continuous_written = False

    def _request_motoring_data(self):
        with self._thread_lock:
            # EFFECTOR
            self._camera_index = self._camera_device.read()
            self._display = self._display_device.read()
            self._left_color = self._left_color_device.read()
            self._right_color = self._right_color_device.read()
            # COMMAND latch
            if self._detect_once_device._is_written():
                self._detect_once_written = True
            if self._detect_continuous_device._is_written():
                self._detect_continuous = self._detect_continuous_device.read()
                self._detect_continuous_written = True
        self._clear_written()

    def _decode(self):
        # engine → sensor
        if self._engine is None or self._engine._is_open() == False:
            return False

        arrays, color_detected, lane = self._engine._get_data()   # lock 스냅샷
        for dev, unit in self._pos_devices:
            dev._put_array(arrays[unit])
        self._color_detected_device._put_array(color_detected)
        lx, ld, ldet = lane['left']
        rx, rd, rdet = lane['right']
        self._left_x_device._put(lx);    self._left_dist_device._put(ld);   self._left_det_device._put(ldet)
        self._right_x_device._put(rx);   self._right_dist_device._put(rd);  self._right_det_device._put(rdet)

        if self._ready == False:
            self._ready = True
            Runner.register_checked()
        self._notify_sensory_device_data_changed()
        return True

    def _encode(self):
        # motor(command/effector) → engine
        with self._thread_lock:
            camera_index = self._camera_index
            left_color = self._left_color
            right_color = self._right_color
            detect_once = self._detect_once_written
            detect_continuous = self._detect_continuous
            detect_continuous_written = self._detect_continuous_written
            self._detect_once_written = False
            self._detect_continuous_written = False

        if camera_index != self._engine._camera_index:
            self._engine._bind_camera(camera_index)
        self._engine._set_lane_colors(left_color, right_color)
        if detect_once:
            self._engine._detect_once()
        if detect_continuous_written:
            self._engine._set_mode(detect_continuous == 1)
