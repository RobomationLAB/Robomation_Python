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
import mediapipe as mp
from mediapipe.tasks.python import BaseOptions
from mediapipe.tasks.python.vision import PoseLandmarker, PoseLandmarkerOptions, RunningMode

from robomation.core.runner import Runner
from robomation.core.model import DeviceType, DataType, Roboid
from robomation.ai._internal._camera import _CaptureSource
from robomation.ai._internal._model import ensure_model, suppress_native_stderr
from robomation.ai.body_detection import _SEGMENTS, _COORD_KEYS


# 자세 모델 (MediaPipe PoseLandmarker, Tasks API; BlazePose 33). 번들 우선, 없으면 캐시/다운로드.
_MODEL_FILE = 'pose_landmarker_full.task'
_MODEL_URL = 'https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_full/float16/1/pose_landmarker_full.task'

# 그리기용 연결선 (브라우저 _POSE_CONNECTIONS, BlazePose 원본 인덱스 기준)
_POSE_CONNECTIONS = [[11, 12], [11, 13], [13, 15], [12, 14], [14, 16],
                     [11, 23], [12, 24], [23, 24], [23, 25], [24, 26],
                     [25, 27], [26, 28]]


def _blank_coords():
    return {k: 0 for k in _COORD_KEYS}


class _BodyEngine:
    """MediaPipe PoseLandmarker + 바인딩된 카메라(_CaptureSource) 래핑."""

    def __init__(self):
        self._detector = None
        self._source = None
        self._camera_index = -1
        self._state = 0            # 0 none / 1 loading / 2 loaded
        self._detected = False
        self._one_detection = False
        self._continuous = False
        self._coords = _blank_coords()
        self._raw = None           # 연결선용 원본 33 키포인트 픽셀
        self._dots = None          # 관절점용 23 파생점 픽셀
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

    # ── 모델 로드 ──
    def _load_model(self):
        if self._state == 2:
            return
        self._state = 1   # loading
        path = ensure_model(_MODEL_FILE, _MODEL_URL, 'BodyDetection')
        if path is None:
            self._state = 0
            return
        try:
            opts = PoseLandmarkerOptions(
                base_options=BaseOptions(model_asset_path=path),
                running_mode=RunningMode.IMAGE,
                num_poses=1,
                min_pose_detection_confidence=0.5)
            with suppress_native_stderr():   # 네이티브 초기화 로그(absl/TFLite) 억제
                self._detector = PoseLandmarker.create_from_options(opts)
            self._state = 2   # loaded
        except Exception as e:
            print(f"[BodyDetection] 모델 로드 실패: {e}", file=sys.stderr)
            self._state = 0

    def _is_loaded(self):
        return self._state == 2

    def _model_state(self):
        return self._state

    # ── 검출 모드 ──
    def _detect_once(self):
        if not self._is_loaded():
            return
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
        # ~15fps 백그라운드 인식
        while self._continuous and not self._closed:
            if self._is_loaded():
                frame = self._read_frame()
                if frame is not None:
                    self._process_frame(frame)
            time.sleep(1 / 15)

    def _process_frame(self, frame):
        with self._detect_lock:
            if self._detector is None:
                return
            h, w = frame.shape[:2]
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            with suppress_native_stderr():   # detect 시 네이티브 경고 억제
                result = self._detector.detect(image)
            poses = result.pose_landmarks if result else None
        if poses and len(poses) > 0 and len(poses[0]) >= 33:
            coords, raw, dots = self._compute(poses[0], w, h)
            with self._lock:
                self._coords = coords
                self._raw = raw
                self._dots = dots
                self._detected = True
        else:
            with self._lock:
                self._coords = _blank_coords()
                self._raw = None
                self._dots = None
                self._detected = False

    def _compute(self, lm, w, h):
        # 브라우저와 동일한 센터좌표(원점=중앙, y-up). 좌/우는 브라우저 미러 매핑 그대로.
        hw, hh = w / 2, h / 2

        def P(i):
            return (lm[i].x * w, lm[i].y * h)

        def avg(*idxs):
            xs = [lm[i].x * w for i in idxs]
            ys = [lm[i].y * h for i in idxs]
            return (sum(xs) / len(xs), sum(ys) / len(ys))

        pts = {}   # 세그먼트 → (px, py)
        pts['eye.left'] = P(5);  pts['eye.right'] = P(2)
        pts['ear.left'] = P(8);  pts['ear.right'] = P(7)
        pts['mouth'] = avg(9, 10)
        pts['nose'] = P(0)
        pts['shoulder.left'] = P(12); pts['shoulder.right'] = P(11)
        lsx, lsy = pts['shoulder.left']; rsx, rsy = pts['shoulder.right']; nx, ny = pts['nose']
        pts['neck'] = (((lsx + rsx) / 2 + nx) / 2, ((lsy + rsy) / 2) * 0.75 + ny * 0.25)
        pts['elbow.left'] = P(14); pts['elbow.right'] = P(13)
        pts['wrist.left'] = P(16); pts['wrist.right'] = P(15)
        pts['hand.left'] = avg(16, 18, 20, 22); pts['hand.right'] = avg(15, 17, 19, 21)
        pts['hip.left'] = P(24); pts['hip.right'] = P(23)
        pts['knee.left'] = P(26); pts['knee.right'] = P(25)
        pts['ankle.left'] = P(28); pts['ankle.right'] = P(27)
        pts['foot.left'] = avg(28, 30, 32); pts['foot.right'] = avg(27, 29, 31)

        coords = _blank_coords()
        for seg in _SEGMENTS:
            px, py = pts[seg]
            coords[seg + '.x'] = int(px - hw)
            coords[seg + '.y'] = int(hh - py)

        raw = [(int(lm[i].x * w), int(lm[i].y * h)) for i in range(len(lm))]   # 연결선용 원본
        dots = [(int(px), int(py)) for px, py in pts.values()]                 # 관절점(23, neck 포함)
        return coords, raw, dots

    # ── 조회 (lock 스냅샷) ──
    def _is_detected(self):
        with self._lock:
            return self._detected and (self._one_detection or self._continuous)

    def _get_coords(self):
        return self._coords

    def _has_drawing(self):
        with self._lock:
            return self._raw is not None

    def _draw_overlay(self, frame):
        with self._lock:
            raw = self._raw
            dots = self._dots
        if raw is None:
            return
        for a, b in _POSE_CONNECTIONS:
            if a < len(raw) and b < len(raw):
                cv2.line(frame, raw[a], raw[b], (0, 255, 0), 3, cv2.LINE_AA)   # 골격 초록
        if dots:
            for p in dots:
                cv2.circle(frame, p, 4, (0, 0, 255), -1)                       # 관절점 빨강

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
            "[BodyDetection] 카메라 입력이 감지되지 않습니다.\n"
            "      macOS 라면 Python 을 실행한 앱(VSCode/터미널 등)의 카메라 권한이 꺼져 있을 수 있습니다.\n"
            "      시스템 설정 → 개인정보 보호 및 보안 → 카메라 에서 해당 앱을 허용한 뒤,\n"
            "      그 앱을 완전히 종료했다가 다시 실행하세요.",
            file=sys.stderr,
        )
        permission.open_camera_settings()

    def _close(self):
        self._closed = True
        self._set_mode(False)
        if self._detector is not None:
            try:
                with suppress_native_stderr():   # 종료 시 clearcut 텔레메트리 로그 억제
                    self._detector.close()
            except Exception:
                pass
            self._detector = None
        if self._source is not None:
            self._source.release()
            self._source = None
        self._opened = False
        self._state = 0


class BodyDetectionRoboid(Roboid):
    def __init__(self, index):
        from robomation.ai.body_detection import BodyDetection
        super(BodyDetectionRoboid, self).__init__(BodyDetection.ID, "BodyDetection", 0xA0600000)
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
        self._load_model_written = False
        self._detect_once_written = False
        self._detect_continuous = 0
        self._detect_continuous_written = False
        self._prev_state = 0

        self._create_model()

    def _create_model(self):
        from robomation.ai.body_detection import BodyDetection
        dict = self._device_dict = {}
        # Effectors
        dict[BodyDetection.CAMERA_DEVICE] = self._camera_device  = self._add_device(BodyDetection.CAMERA_DEVICE, "CameraDevice", DeviceType.EFFECTOR, DataType.INTEGER, 1, -1, 1000, -1)
        dict[BodyDetection.DISPLAY] = self._display_device = self._add_device(BodyDetection.DISPLAY, "Display", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 1)

        # Commands
        dict[BodyDetection.LOAD_MODEL]        = self._load_model_device        = self._add_device(BodyDetection.LOAD_MODEL,        "LoadModel",        DeviceType.COMMAND, DataType.INTEGER, 1, 0, 1, 0)
        dict[BodyDetection.DETECT_ONCE]       = self._detect_once_device       = self._add_device(BodyDetection.DETECT_ONCE,       "DetectOnce",       DeviceType.COMMAND, DataType.INTEGER, 1, 0, 1, 0)
        dict[BodyDetection.DETECT_CONTINUOUS] = self._detect_continuous_device = self._add_device(BodyDetection.DETECT_CONTINUOUS, "DetectContinuous", DeviceType.COMMAND, DataType.INTEGER, 1, 0, 1, 0)

        # Sensors
        dict[BodyDetection.MODEL_STATE] = self._model_state_device = self._add_device(BodyDetection.MODEL_STATE, "ModelState", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 2, 0)
        dict[BodyDetection.DETECTED]    = self._detected_device    = self._add_device(BodyDetection.DETECTED,    "Detected",    DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)

        # 좌표 (BodyDetection._COORD_DEVICE 맵 기반으로 개별 SENSOR 디바이스 생성)
        _range = [-10000, 10000]
        self._coord_devices = []
        for key, dev_id in BodyDetection._COORD_DEVICE.items():
            dev = self._add_device(dev_id, key, DeviceType.SENSOR, DataType.INTEGER, 1, _range[0], _range[1], 0)
            dict[dev_id] = dev
            self._coord_devices.append((dev, key))

        # Event
        dict[BodyDetection.LOAD_MODEL_STATE] = self._load_model_state_device = self._add_device(BodyDetection.LOAD_MODEL_STATE, "LoadModelState", DeviceType.EVENT, DataType.INTEGER, 1, 0, 0, 0)

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

        self._engine = _BodyEngine()
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
            super(BodyDetectionRoboid, self)._dispose()
            self._release()

    def _reset(self):
        super(BodyDetectionRoboid, self)._reset()
        self._camera_index = -1
        self._display = 0
        self._load_model_written = False
        self._detect_once_written = False
        self._detect_continuous = 0
        self._detect_continuous_written = False
        self._prev_state = 0

    def _request_motoring_data(self):
        with self._thread_lock:
            # EFFECTOR
            self._camera_index = self._camera_device.read()
            self._display = self._display_device.read()
            # COMMAND latch
            if self._load_model_device._is_written():
                self._load_model_written = True
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

        state = self._engine._model_state()
        self._model_state_device._put(state)
        if self._prev_state == 1 and state == 2:          # loading → loaded
            self._load_model_state_device._put_empty()
        self._prev_state = state

        self._detected_device._put(1 if self._engine._is_detected() else 0)

        c = self._engine._get_coords()                    # lock 스냅샷
        for dev, key in self._coord_devices:
            dev._put(c[key])

        if self._ready == False:
            self._ready = True
            Runner.register_checked()
        self._notify_sensory_device_data_changed()
        return True

    def _encode(self):
        # motor(command/effector) → engine
        with self._thread_lock:
            camera_index = self._camera_index
            load_model = self._load_model_written
            detect_once = self._detect_once_written
            detect_continuous = self._detect_continuous
            detect_continuous_written = self._detect_continuous_written
            self._load_model_written = False
            self._detect_once_written = False
            self._detect_continuous_written = False

        if camera_index != self._engine._camera_index:
            self._engine._bind_camera(camera_index)
        if load_model:
            self._engine._load_model()
        if detect_once:
            self._engine._detect_once()
        if detect_continuous_written:
            self._engine._set_mode(detect_continuous == 1)
