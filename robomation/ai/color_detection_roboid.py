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
from robomation.ai.color_detection import ColorDetection

# 색 HSV 규칙 (OpenCV 스케일: H 0..179, S/V 0..255). 브라우저 H(0..360)을 ÷2 변환.
# 각 색은 [(lower, upper), ...] (여러 구간은 OR).
_COLOR_RULES = {
    'black':   [((0, 0, 0),     (180, 255, 60))],
    'red':     [((0, 128, 128), (5, 255, 255)), ((175, 128, 128), (180, 255, 255))],
    'yellow':  [((25, 100, 150), (35, 255, 255))],
    'green':   [((40, 80, 60),  (80, 255, 255))],
    'cyan':    [((85, 80, 60),  (95, 255, 255))],
    'blue':    [((100, 100, 40), (140, 255, 255))],
    'magenta': [((145, 80, 80), (155, 255, 255))],
    'white':   [((0, 0, 190),   (180, 45, 255))],
}

# 그리기용 BGR 색
_DRAW_BGR = {
    'black': (0, 0, 0), 'red': (0, 0, 255), 'yellow': (0, 255, 255), 'green': (0, 255, 0),
    'cyan': (255, 255, 0), 'blue': (255, 0, 0), 'magenta': (255, 0, 255), 'white': (255, 255, 255),
}

_IDX_TO_COLOR = {v: k for k, v in ColorDetection._VALID_COLOR.items()}

_KERNEL = np.ones((3, 3), np.uint8)


def _blank_arrays():
    arrays = {pos: [0] * 8 for pos in ColorDetection._VALID_POS}
    arrays['detected'] = [0] * 8
    return arrays


class _ColorEngine:
    """cv2 HSV 색검출 + 바인딩된 카메라(_CaptureSource) 래핑. (ML 모델 없음)"""

    def __init__(self):
        self._source = None
        self._camera_index = -1
        self._area_condition = 100
        self._registered = []      # 등록된 색 이름 리스트(순서 유지)
        self._detected = False
        self._one_detection = False
        self._continuous = False
        self._arrays = _blank_arrays()
        self._draw = []            # [(ix, iy, w, h, color_name), ...] 픽셀
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

    # ── 카메라 바인딩 (camera.py _CaptureSource 공유) ──
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

    # ── 색 등록/삭제 ──
    def _register_color(self, idx):
        name = _IDX_TO_COLOR.get(idx)
        if name and name not in self._registered:
            with self._detect_lock:
                self._registered.append(name)

    def _delete_color(self, idx):
        name = _IDX_TO_COLOR.get(idx)
        if name and name in self._registered:
            with self._detect_lock:
                self._registered.remove(name)

    def _is_registered(self, idx):
        name = _IDX_TO_COLOR.get(idx)
        with self._detect_lock:
            return name in self._registered

    def _set_area_condition(self, area):
        self._area_condition = area

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
        # ~15fps 백그라운드 검출
        while self._continuous and not self._closed:
            frame = self._read_frame()
            if frame is not None:
                self._process_frame(frame)
            time.sleep(1 / 15)

    def _process_frame(self, frame):
        with self._detect_lock:
            registered = list(self._registered)
        if not registered:
            with self._lock:
                self._arrays = _blank_arrays()
                self._draw = []
                self._detected = False
            return
        h, w = frame.shape[:2]
        hw, hh = w / 2, h / 2
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        arrays = _blank_arrays()
        draw = []
        cond = self._area_condition
        any_detected = False
        for name in registered:
            blob = self._largest_blob(hsv, name, cond)
            if blob is None:
                continue
            ix, iy, bw, bh = blob
            tlx = int(ix - hw); brx = tlx + bw
            tly = int(hh - iy);  bry = tly + bh
            area = bw * bh
            if area <= cond:
                continue
            idx = ColorDetection._VALID_COLOR[name]
            arrays['x'][idx] = int((tlx + brx) / 2)
            arrays['y'][idx] = int((tly + bry) / 2)
            arrays['min_x'][idx] = tlx
            arrays['max_x'][idx] = brx
            arrays['min_y'][idx] = tly
            arrays['max_y'][idx] = bry
            arrays['width'][idx] = bw
            arrays['height'][idx] = bh
            arrays['area'][idx] = area
            arrays['detected'][idx] = 1 if self._detected and area >= cond else 0
            draw.append((ix, iy, bw, bh, name))
            any_detected = True
        with self._lock:
            self._arrays = arrays
            self._draw = draw
            self._detected = any_detected

    def _largest_blob(self, hsv, name, cond):
        # 해당 색 마스크 → 형태학 open → 최대 연결요소 bbox. 최소 픽셀 미만이면 None.
        mask = None
        for lower, upper in _COLOR_RULES[name]:
            m = cv2.inRange(hsv, np.array(lower, np.uint8), np.array(upper, np.uint8))
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _KERNEL)
        num, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        min_pixels = max(5, int(cond))
        best = None
        best_pixels = 0
        for i in range(1, num):   # 0 = background
            pixels = stats[i, cv2.CC_STAT_AREA]
            if pixels < min_pixels:
                continue
            if pixels > best_pixels:
                best_pixels = pixels
                best = (int(stats[i, cv2.CC_STAT_LEFT]), int(stats[i, cv2.CC_STAT_TOP]),
                        int(stats[i, cv2.CC_STAT_WIDTH]), int(stats[i, cv2.CC_STAT_HEIGHT]))
        return best

    # ── 조회 (lock 스냅샷) ──
    def _is_detected(self):
        with self._lock:
            return self._detected and (self._one_detection or self._continuous)

    def _get_data(self):
        return self._arrays

    def _has_drawing(self):
        with self._lock:
            return len(self._draw) > 0

    def _draw_overlay(self, frame):
        with self._lock:
            draws = list(self._draw)
        for ix, iy, w, h, name in draws:
            color = _DRAW_BGR.get(name, (0, 255, 0))
            cv2.rectangle(frame, (ix, iy), (ix + w, iy + h), color, 2)
            ty = iy - 8 if iy > 14 else iy + 18
            cv2.putText(frame, name, (ix, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2, cv2.LINE_AA)

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
            "[ColorDetection] 카메라 입력이 감지되지 않습니다.\n"
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


class ColorDetectionRoboid(Roboid):
    def __init__(self, index):
        from robomation.ai.color_detection import ColorDetection
        super(ColorDetectionRoboid, self).__init__(ColorDetection.ID, "ColorDetection", 0xA0800000)
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
        self._area_condition = 100
        self._register_value = -1
        self._register_written = False
        self._delete_value = -1
        self._delete_written = False
        self._detect_once_written = False
        self._detect_continuous = 0
        self._detect_continuous_written = False

        self._create_model()

    def _create_model(self):
        from robomation.ai.color_detection import ColorDetection
        dict = self._device_dict = {}
        # Effectors
        dict[ColorDetection.CAMERA_DEVICE]  = self._camera_device  = self._add_device(ColorDetection.CAMERA_DEVICE, "CameraDevice", DeviceType.EFFECTOR, DataType.INTEGER, 1, -1, 1000, -1)
        dict[ColorDetection.DISPLAY]        = self._display_device = self._add_device(ColorDetection.DISPLAY, "Display", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 1)
        dict[ColorDetection.AREA_CONDITION] = self._area_device    = self._add_device(ColorDetection.AREA_CONDITION, "AreaCondition", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 10000000, 100)

        # Commands
        dict[ColorDetection.REGISTER_COLOR]    = self._register_device         = self._add_device(ColorDetection.REGISTER_COLOR,    "RegisterColor",    DeviceType.COMMAND, DataType.INTEGER, 1, 0, 7, 0)
        dict[ColorDetection.DELETE_COLOR]      = self._delete_device           = self._add_device(ColorDetection.DELETE_COLOR,      "DeleteColor",      DeviceType.COMMAND, DataType.INTEGER, 1, 0, 7, 0)
        dict[ColorDetection.DETECT_ONCE]       = self._detect_once_device       = self._add_device(ColorDetection.DETECT_ONCE,       "DetectOnce",       DeviceType.COMMAND, DataType.INTEGER, 1, 0, 1, 0)
        dict[ColorDetection.DETECT_CONTINUOUS] = self._detect_continuous_device = self._add_device(ColorDetection.DETECT_CONTINUOUS, "DetectContinuous", DeviceType.COMMAND, DataType.INTEGER, 1, 0, 1, 0)

        # Sensors
        dict[ColorDetection.DETECTED] = self._detected_device = self._add_device(ColorDetection.DETECTED, "Detected", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)

        # 좌표 배열 디바이스 (dimension = 8 색)
        _range = [-10000, 10000]
        self._pos_devices = []
        for pos in ColorDetection._VALID_POS:
            dev_id = ColorDetection._POS_DEVICE[pos]
            dev = self._add_device(dev_id, "Color_" + pos, DeviceType.SENSOR, DataType.INTEGER, 8, _range[0], _range[1] * _range[1], 0)
            dict[dev_id] = dev
            self._pos_devices.append((dev, pos))
        dict[ColorDetection.COLOR_DETECTED]    = self._color_detected_device = self._add_device(ColorDetection.COLOR_DETECTED, "ColorDetected", DeviceType.SENSOR, DataType.INTEGER, 8, 0, 1, 0)

    def _find_device_by_id(self, device_id):
        return self._device_dict.get(device_id)

    def _wait_color(self, idx, present, timeout=2.0):
        # register/delete 명령이 엔진에 실제 반영될 때까지 폴링 대기.
        deadline = time.time() + timeout
        while time.time() < deadline:
            engine = self._engine
            if engine is None:
                return
            if engine._is_registered(idx) == present:
                return
            time.sleep(0.005)

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

        self._engine = _ColorEngine()
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
            super(ColorDetectionRoboid, self)._dispose()
            self._release()

    def _reset(self):
        super(ColorDetectionRoboid, self)._reset()
        self._camera_index = -1
        self._display = 0
        self._area_condition = 100
        self._register_value = -1
        self._register_written = False
        self._delete_value = -1
        self._delete_written = False
        self._detect_once_written = False
        self._detect_continuous = 0
        self._detect_continuous_written = False

    def _request_motoring_data(self):
        with self._thread_lock:
            # EFFECTOR
            self._camera_index = self._camera_device.read()
            self._display = self._display_device.read()
            self._area_condition = self._area_device.read()
            # COMMAND latch
            if self._register_device._is_written():
                self._register_value = self._register_device.read()
                self._register_written = True
            if self._delete_device._is_written():
                self._delete_value = self._delete_device.read()
                self._delete_written = True
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

        # self._detected_device._put(1 if self._engine._is_detected() else 0)

        arrays = self._engine._get_data()                 # lock 스냅샷
        for dev, pos in self._pos_devices:
            dev._put_array(arrays[pos])
        self._color_detected_device._put_array(arrays['detected'])

        if self._ready == False:
            self._ready = True
            Runner.register_checked()
        self._notify_sensory_device_data_changed()
        return True

    def _encode(self):
        # motor(command/effector) → engine
        with self._thread_lock:
            camera_index = self._camera_index
            area_condition = self._area_condition
            register = self._register_written
            register_value = self._register_value
            delete = self._delete_written
            delete_value = self._delete_value
            detect_once = self._detect_once_written
            detect_continuous = self._detect_continuous
            detect_continuous_written = self._detect_continuous_written
            self._register_written = False
            self._delete_written = False
            self._detect_once_written = False
            self._detect_continuous_written = False

        if camera_index != self._engine._camera_index:
            self._engine._bind_camera(camera_index)
        self._engine._set_area_condition(area_condition)
        if register:
            self._engine._register_color(register_value)
        if delete:
            self._engine._delete_color(delete_value)
        if detect_once:
            self._engine._detect_once()
        if detect_continuous_written:
            self._engine._set_mode(detect_continuous == 1)
