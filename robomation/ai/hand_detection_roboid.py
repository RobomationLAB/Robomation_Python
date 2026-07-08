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
from mediapipe.tasks.python.vision import HandLandmarker, HandLandmarkerOptions, RunningMode
from mediapipe.tasks.python.vision.hand_landmarker import HandLandmarksConnections as _HLC

from robomation.core.runner import Runner
from robomation.core.model import DeviceType, DataType, Roboid
from robomation.ai._internal._camera import _CaptureSource
from robomation.ai._internal._model import ensure_model, suppress_native_stderr
from robomation.ai.hand_detection import _COORD_KEYS


# 손 모델 (MediaPipe HandLandmarker, Tasks API; 21 랜드마크). 번들 우선, 없으면 캐시/다운로드.
_MODEL_FILE = 'hand_landmarker.task'
_MODEL_URL = 'https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task'

_PALMS = [0, 1, 2, 5, 9, 13, 17]
# 손가락 → 21 랜드마크 시작 인덱스 (4 관절 연속). joint 0..3 = first/second/third/last
_FINGER_BASE = (('thumb', 1), ('index', 5), ('middle', 9), ('ring', 13), ('pinky', 17))
_JOINT_NAMES = ('first', 'second', 'third', 'last')
# 그리기용 연결선 (HandLandmarker 토폴로지)
_HAND_CONNECTIONS = [(c.start, c.end) for c in _HLC.HAND_CONNECTIONS]


def _blank_coords():
    return {k: 0 for k in _COORD_KEYS}


class _HandEngine:
    """MediaPipe HandLandmarker + 바인딩된 카메라(_CaptureSource) 래핑."""

    def __init__(self):
        self._detector = None
        self._source = None
        self._camera_index = -1
        self._max_hands = 1
        self._state = 0            # 0 none / 1 loading / 2 loaded
        self._detected = False
        self._one_detection = False
        self._continuous = False
        self._coords = _blank_coords()
        self._draw = []            # [(pts_px[21], color_bgr), ...]
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
    def _create_detector(self):
        path = ensure_model(_MODEL_FILE, _MODEL_URL, 'HandDetection')
        if path is None:
            return None
        opts = HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=path),
            running_mode=RunningMode.IMAGE,
            num_hands=self._max_hands,
            min_hand_detection_confidence=0.5)
        with suppress_native_stderr():   # 네이티브 초기화 로그(absl/TFLite) 억제
            return HandLandmarker.create_from_options(opts)

    def _load_model(self):
        if self._state == 2:
            return
        self._state = 1   # loading
        try:
            detector = self._create_detector()
            if detector is None:
                self._state = 0
                return
            with self._detect_lock:
                self._detector = detector
            self._state = 2   # loaded
        except Exception as e:
            print(f"[HandDetection] 모델 로드 실패: {e}", file=sys.stderr)
            self._state = 0

    def _set_max_hands(self, n):
        if n == self._max_hands:
            return
        self._max_hands = n
        if self._state == 2:   # 로드된 상태면 detector 재생성
            try:
                detector = self._create_detector()
                if detector is not None:
                    with self._detect_lock:
                        old = self._detector
                        self._detector = detector
                    if old is not None:
                        try:
                            with suppress_native_stderr():
                                old.close()
                        except Exception:
                            pass
            except Exception as e:
                print(f"[HandDetection] detector 재생성 실패: {e}", file=sys.stderr)

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
            with suppress_native_stderr():   # detect 시 landmark_projection NORM_RECT 경고 억제
                result = self._detector.detect(image)
            hands = result.hand_landmarks if result else None
            handed = result.handedness if result else None
        coords = _blank_coords()
        draw = []
        if hands and len(hands) > 0:
            if self._max_hands == 1:
                self._fill(coords, 'left', hands[0], w, h)
                self._fill(coords, 'right', hands[0], w, h)
                draw.append((self._pts(hands[0], w, h), (255, 255, 0)))   # cyan
            else:
                for i, lm in enumerate(hands):
                    is_right = (handed and handed[i] and handed[i][0].category_name == 'Right')
                    side = 'right' if is_right else 'left'
                    self._fill(coords, side, lm, w, h)
                    color = (255, 0, 0) if is_right else (0, 255, 0)      # 우=파랑/좌=초록
                    draw.append((self._pts(lm, w, h), color))
            with self._lock:
                self._coords = coords
                self._draw = draw
                self._detected = True
        else:
            with self._lock:
                self._coords = coords
                self._draw = []
                self._detected = False

    def _pts(self, lm, w, h):
        return [(int(p.x * w), int(p.y * h)) for p in lm]   # 원본 픽셀(y-down) 그리기용

    def _fill(self, coords, side, lm, w, h):
        hw, hh = w / 2, h / 2
        cx = [int(p.x * w - hw) for p in lm]
        cy = [int(hh - p.y * h) for p in lm]
        coords[f'{side}.wrist.x'], coords[f'{side}.wrist.y'] = cx[0], cy[0]
        for fname, base in _FINGER_BASE:
            for j in range(4):
                idx = base + j
                jn = _JOINT_NAMES[j]
                coords[f'{side}.{fname}.{jn}.x'] = cx[idx]
                coords[f'{side}.{fname}.{jn}.y'] = cy[idx]
        # 손바닥 중심(센터좌표 평균)
        n = len(_PALMS)
        coords[f'{side}.palm.x'] = int(sum(cx[i] for i in _PALMS) / n)
        coords[f'{side}.palm.y'] = int(sum(cy[i] for i in _PALMS) / n)
        self._bbox(coords, f'{side}.palm', cx, cy, _PALMS)
        self._bbox(coords, f'{side}.hand', cx, cy, range(len(lm)))

    def _bbox(self, coords, prefix, cx, cy, idxs):
        xs = [cx[i] for i in idxs]
        ys = [cy[i] for i in idxs]
        tlx, brx = min(xs), max(xs)
        tly, bry = min(ys), max(ys)
        coords[prefix + '.min_x'], coords[prefix + '.max_x'] = tlx, brx
        coords[prefix + '.min_y'], coords[prefix + '.max_y'] = tly, bry
        coords[prefix + '.width'] = abs(brx - tlx)
        coords[prefix + '.height'] = abs(bry - tly)
        coords[prefix + '.area'] = abs(brx - tlx) * abs(bry - tly)

    # ── 조회 (lock 스냅샷) ──
    def _is_detected(self):
        with self._lock:
            return self._detected and (self._one_detection or self._continuous)

    def _get_coords(self):
        return self._coords

    def _has_drawing(self):
        with self._lock:
            return len(self._draw) > 0

    def _draw_overlay(self, frame):
        with self._lock:
            draws = list(self._draw)
        for pts, color in draws:
            for a, b in _HAND_CONNECTIONS:
                if a < len(pts) and b < len(pts):
                    cv2.line(frame, pts[a], pts[b], color, 3, cv2.LINE_AA)
            for p in pts:
                cv2.circle(frame, p, 4, (0, 0, 255), -1)   # 관절점 빨강

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
            "[HandDetection] 카메라 입력이 감지되지 않습니다.\n"
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


class HandDetectionRoboid(Roboid):
    def __init__(self, index):
        from robomation.ai.hand_detection import HandDetection
        super(HandDetectionRoboid, self).__init__(HandDetection.ID, "HandDetection", 0xA0500000)
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
        self._max_count = 1
        self._load_model_written = False
        self._detect_once_written = False
        self._detect_continuous = 0
        self._detect_continuous_written = False
        self._prev_state = 0

        self._create_model()

    def _create_model(self):
        from robomation.ai.hand_detection import HandDetection
        dict = self._device_dict = {}
        # Effectors
        dict[HandDetection.CAMERA_DEVICE] = self._camera_device = self._add_device(HandDetection.CAMERA_DEVICE, "CameraDevice", DeviceType.EFFECTOR, DataType.INTEGER, 1, -1, 1000, -1)
        dict[HandDetection.DISPLAY]       = self._display_device = self._add_device(HandDetection.DISPLAY, "Display", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 1)
        dict[HandDetection.MAX_COUNT]     = self._max_count_device = self._add_device(HandDetection.MAX_COUNT, "MaxCount", DeviceType.EFFECTOR, DataType.INTEGER, 1, 1, 2, 1)

        # Commands
        dict[HandDetection.LOAD_MODEL]        = self._load_model_device        = self._add_device(HandDetection.LOAD_MODEL,        "LoadModel",        DeviceType.COMMAND, DataType.INTEGER, 1, 0, 1, 0)
        dict[HandDetection.DETECT_ONCE]       = self._detect_once_device       = self._add_device(HandDetection.DETECT_ONCE,       "DetectOnce",       DeviceType.COMMAND, DataType.INTEGER, 1, 0, 1, 0)
        dict[HandDetection.DETECT_CONTINUOUS] = self._detect_continuous_device = self._add_device(HandDetection.DETECT_CONTINUOUS, "DetectContinuous", DeviceType.COMMAND, DataType.INTEGER, 1, 0, 1, 0)

        # Sensors
        dict[HandDetection.MODEL_STATE] = self._model_state_device = self._add_device(HandDetection.MODEL_STATE, "ModelState", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 2, 0)
        dict[HandDetection.DETECTED]    = self._detected_device    = self._add_device(HandDetection.DETECTED,    "Detected",    DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)

        # 좌표 (HandDetection._COORD_DEVICE 맵 기반으로 개별 SENSOR 디바이스 생성)
        _range = [-10000, 10000]
        self._coord_devices = []
        for key, dev_id in HandDetection._COORD_DEVICE.items():
            if key.endswith('width') or key.endswith('height'):
                lo, hi = 0, 2 * _range[1]
            elif key.endswith('area'):
                lo, hi = 0, _range[1] * _range[1]
            else:
                lo, hi = _range[0], _range[1]
            dev = self._add_device(dev_id, key, DeviceType.SENSOR, DataType.INTEGER, 1, lo, hi, 0)
            dict[dev_id] = dev
            self._coord_devices.append((dev, key))

        # Event
        dict[HandDetection.LOAD_MODEL_STATE] = self._load_model_state_device = self._add_device(HandDetection.LOAD_MODEL_STATE, "LoadModelState", DeviceType.EVENT, DataType.INTEGER, 1, 0, 0, 0)

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

        self._engine = _HandEngine()
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
            super(HandDetectionRoboid, self)._dispose()
            self._release()

    def _reset(self):
        super(HandDetectionRoboid, self)._reset()
        self._camera_index = -1
        self._display = 0
        self._max_count = 1
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
            self._max_count = self._max_count_device.read()
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
            max_count = self._max_count
            load_model = self._load_model_written
            detect_once = self._detect_once_written
            detect_continuous = self._detect_continuous
            detect_continuous_written = self._detect_continuous_written
            self._load_model_written = False
            self._detect_once_written = False
            self._detect_continuous_written = False

        if camera_index != self._engine._camera_index:
            self._engine._bind_camera(camera_index)
        if max_count != self._engine._max_hands:
            self._engine._set_max_hands(max_count)
        if load_model:
            self._engine._load_model()
        if detect_once:
            self._engine._detect_once()
        if detect_continuous_written:
            self._engine._set_mode(detect_continuous == 1)
