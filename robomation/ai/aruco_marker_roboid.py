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
import math
import threading

import cv2
import numpy as np

from robomation.core.runner import Runner
from robomation.core.model import DeviceType, DataType, Roboid
from robomation.ai._internal._camera import _CaptureSource
from robomation.ai.aruco_marker import _POS_LIST, _NUM_MARKERS


_DRAW_BGR = (114, 224, 255)   # #FFE072


def _blank_arrays():
    arrays = {unit: [0] * _NUM_MARKERS for unit in _POS_LIST}
    detected = [0] * _NUM_MARKERS
    return arrays, detected


def _dist(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _polygon_area(pts):
    area = 0.0
    n = len(pts)
    for i in range(n):
        j = (i + 1) % n
        area += pts[i][0] * pts[j][1]
        area -= pts[j][0] * pts[i][1]
    return abs(area) / 2


class _ArucoEngine:
    """cv2.aruco 마커 검출 + 바인딩된 카메라(_CaptureSource) 래핑. (ML 모델 없음)"""

    def __init__(self):
        self._detector = None
        self._source = None
        self._camera_index = -1
        self._max_count = 10
        self._detected = False
        self._one_detection = False
        self._continuous = False
        self._arrays, self._marker_detected = _blank_arrays()
        self._draw = []            # [(corners_px[4], id), ...]
        self._opened = False
        self._closed = False
        self._thread = None
        self._lock = threading.Lock()
        self._detect_lock = threading.Lock()

    def _open(self):
        # cv2.aruco 검출기 생성 (DICT_ARUCO_ORIGINAL = js-aruco 호환, 1024 마커)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self._detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
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

    def _set_max_count(self, n):
        self._max_count = n

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
            if self._detector is None:
                return
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self._detector.detectMarkers(gray)
        if ids is not None and len(ids) > 0:
            h, w = frame.shape[:2]
            arrays, marker_detected, draw = self._compute(corners, ids, w, h)
            with self._lock:
                self._arrays = arrays
                self._marker_detected = marker_detected
                self._draw = draw
                self._detected = True
        else:
            with self._lock:
                self._arrays, self._marker_detected = _blank_arrays()
                self._draw = []
                self._detected = False

    def _compute(self, corners, ids, w, h):
        hw, hh = w / 2, h / 2
        arrays, marker_detected = _blank_arrays()
        draw = []
        count = max(1, int(self._max_count))
        for i in range(min(len(ids), count)):
            mid = int(ids[i][0])
            if mid < 0 or mid >= _NUM_MARKERS:
                continue
            pts_px = [(float(p[0]), float(p[1])) for p in corners[i][0]]   # 4×(x,y) 픽셀
            # 센터좌표(원점 중앙). y 는 위로 +.
            cx = [p[0] - hw for p in pts_px]
            cy = [p[1] - hh for p in pts_px]
            tlx = int(min(cx)); brx = int(max(cx))
            tly = int(-max(cy)); bry = int(-min(cy))
            arrays['x'][mid] = int((tlx + brx) / 2)
            arrays['y'][mid] = int((tly + bry) / 2)
            arrays['min_x'][mid] = tlx
            arrays['max_x'][mid] = brx
            arrays['min_y'][mid] = tly
            arrays['max_y'][mid] = bry
            arrays['width'][mid] = int((_dist(pts_px[0], pts_px[1]) + _dist(pts_px[3], pts_px[2])) / 2)
            arrays['height'][mid] = int((_dist(pts_px[0], pts_px[3]) + _dist(pts_px[1], pts_px[2])) / 2)
            arrays['area'][mid] = int(_polygon_area(pts_px))
            arrays['rotation'][mid] = int(math.atan2(pts_px[1][0] - pts_px[0][0],
                                                     pts_px[1][1] - pts_px[0][1]) * 180 / math.pi)
            marker_detected[mid] = 1
            draw.append(([(int(p[0]), int(p[1])) for p in pts_px], mid))
        return arrays, marker_detected, draw

    # ── 조회 (lock 스냅샷) ──
    def _is_detected(self):
        with self._lock:
            return self._detected and (self._one_detection or self._continuous)

    def _get_data(self):
        return self._arrays, self._marker_detected

    def _has_drawing(self):
        with self._lock:
            return len(self._draw) > 0

    def _draw_overlay(self, frame):
        with self._lock:
            draws = list(self._draw)
        for pts, mid in draws:
            cv2.polylines(frame, [np.array(pts, np.int32)], True, _DRAW_BGR, 3, cv2.LINE_AA)
            cv2.putText(frame, str(mid), (pts[0][0], pts[0][1] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, _DRAW_BGR, 2, cv2.LINE_AA)

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
            "[ArucoMarker] 카메라 입력이 감지되지 않습니다.\n"
            "      macOS 라면 Python 을 실행한 앱(VSCode/터미널 등)의 카메라 권한이 꺼져 있을 수 있습니다.\n"
            "      시스템 설정 → 개인정보 보호 및 보안 → 카메라 에서 해당 앱을 허용한 뒤,\n"
            "      그 앱을 완전히 종료했다가 다시 실행하세요.",
            file=sys.stderr,
        )
        permission.open_camera_settings()

    def _close(self):
        self._closed = True
        self._set_mode(False)
        self._detector = None
        if self._source is not None:
            self._source.release()
            self._source = None
        self._opened = False


class ArucoMarkerRoboid(Roboid):
    def __init__(self, index):
        from robomation.ai.aruco_marker import ArucoMarker
        super(ArucoMarkerRoboid, self).__init__(ArucoMarker.ID, "ArucoMarker", 0xA0900000)
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
        self._max_count = 10
        self._detect_once_written = False
        self._detect_continuous = 0
        self._detect_continuous_written = False

        self._create_model()

    def _create_model(self):
        from robomation.ai.aruco_marker import ArucoMarker
        dict = self._device_dict = {}
        # Effectors
        dict[ArucoMarker.CAMERA_DEVICE] = self._camera_device  = self._add_device(ArucoMarker.CAMERA_DEVICE, "CameraDevice", DeviceType.EFFECTOR, DataType.INTEGER, 1, -1, 1000, -1)
        dict[ArucoMarker.DISPLAY]   = self._display_device   = self._add_device(ArucoMarker.DISPLAY, "Display", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 1)
        dict[ArucoMarker.MAX_COUNT] = self._max_count_device = self._add_device(ArucoMarker.MAX_COUNT, "MaxCount", DeviceType.EFFECTOR, DataType.INTEGER, 1, 1, _NUM_MARKERS, 10)

        # Commands
        dict[ArucoMarker.DETECT_ONCE]       = self._detect_once_device       = self._add_device(ArucoMarker.DETECT_ONCE,       "DetectOnce",       DeviceType.COMMAND, DataType.INTEGER, 1, 0, 1, 0)
        dict[ArucoMarker.DETECT_CONTINUOUS] = self._detect_continuous_device = self._add_device(ArucoMarker.DETECT_CONTINUOUS, "DetectContinuous", DeviceType.COMMAND, DataType.INTEGER, 1, 0, 1, 0)

        # Sensors — 마커 ID 인덱스 배열(dimension = 1024)
        _range = [-10000, 10000]
        self._pos_devices = []
        for unit in _POS_LIST:
            dev_id = ArucoMarker._POS_DEVICE[unit]
            if unit == 'rotation':
                lo, hi = -180, 180
            elif unit == 'area':
                lo, hi = 0, _range[1] * _range[1]
            elif unit in ('width', 'height'):
                lo, hi = 0, 2 * _range[1]
            else:
                lo, hi = _range[0], _range[1]
            dev = self._add_device(dev_id, "Marker_" + unit, DeviceType.SENSOR, DataType.INTEGER, _NUM_MARKERS, lo, hi, 0)
            dict[dev_id] = dev
            self._pos_devices.append((dev, unit))
        dict[ArucoMarker.MARKER_DETECTED] = self._marker_detected_device = self._add_device(ArucoMarker.MARKER_DETECTED, "Marker_detected", DeviceType.SENSOR, DataType.INTEGER, _NUM_MARKERS, 0, 1, 0)

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

        self._engine = _ArucoEngine()
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
            super(ArucoMarkerRoboid, self)._dispose()
            self._release()

    def _reset(self):
        super(ArucoMarkerRoboid, self)._reset()
        self._camera_index = -1
        self._display = 0
        self._max_count = 10
        self._detect_once_written = False
        self._detect_continuous = 0
        self._detect_continuous_written = False

    def _request_motoring_data(self):
        with self._thread_lock:
            # EFFECTOR
            self._camera_index = self._camera_device.read()
            self._display = self._display_device.read()
            self._max_count = self._max_count_device.read()
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

        arrays, marker_detected = self._engine._get_data()   # lock 스냅샷
        for dev, unit in self._pos_devices:
            dev._put_array(arrays[unit])
        self._marker_detected_device._put_array(marker_detected)

        if self._ready == False:
            self._ready = True
            Runner.register_checked()
        self._notify_sensory_device_data_changed()
        return True

    def _encode(self):
        # motor(command/effector) → engine
        with self._thread_lock:
            camera_index = self._camera_index
            max_count = self._max_count
            detect_once = self._detect_once_written
            detect_continuous = self._detect_continuous
            detect_continuous_written = self._detect_continuous_written
            self._detect_once_written = False
            self._detect_continuous_written = False

        if camera_index != self._engine._camera_index:
            self._engine._bind_camera(camera_index)
        self._engine._set_max_count(max_count)
        if detect_once:
            self._engine._detect_once()
        if detect_continuous_written:
            self._engine._set_mode(detect_continuous == 1)
