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
from mediapipe.tasks.python.vision import FaceDetector, FaceDetectorOptions, RunningMode

from robomation.core.runner import Runner
from robomation.core.model import DeviceType, DataType, Roboid
from robomation.ai._internal._camera import resolve, _CaptureSource
from robomation.ai._internal._model import ensure_model, suppress_native_stderr


# 미리 학습된 얼굴 검출 모델 (MediaPipe BlazeFace short-range, Tasks API)
# 패키지 번들(robomation/ai/models) 우선, 없으면 캐시/다운로드. (오프라인 지원)
_MODEL_FILE = 'blaze_face_short_range.tflite'
_MODEL_URL = 'https://storage.googleapis.com/mediapipe-models/face_detector/blaze_face_short_range/float16/1/blaze_face_short_range.tflite'

def _blank_data():
    return {
        'face': {'x': 0, 'y': 0, 'min_x': 0, 'min_y': 0, 'max_x': 0, 'max_y': 0,
                 'width': 0, 'height': 0, 'area': 0},
        'eye':  {'left': {'x': 0, 'y': 0}, 'right': {'x': 0, 'y': 0}},
        'ear':  {'left': {'x': 0, 'y': 0}, 'right': {'x': 0, 'y': 0}},
        'nose': {'x': 0, 'y': 0},
        'mouth': {'x': 0, 'y': 0},
    }


class _FaceEngine:
    """MediaPipe FaceDetection + 바인딩된 카메라(_CaptureSource) 래핑. ASR _ASREngine 대응."""

    def __init__(self):
        self._detector = None
        self._source = None
        self._camera_index = -1
        self._state = 0            # 0 none / 1 loading / 2 loaded
        self._detected = False
        self._one_detection = False
        self._continuous = False
        self._data = _blank_data()
        self._raw = None           # 오버레이용 원본픽셀 (box_pt1, box_pt2, [6 points])
        self._opened = False
        self._closed = False
        self._thread = None        # 연속 인식 스레드
        self._lock = threading.Lock()         # _data/_raw/_detected/_state 보호
        self._detect_lock = threading.Lock()  # detector.process 직렬화

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
        path = ensure_model(_MODEL_FILE, _MODEL_URL, 'FaceDetection')
        if path is None:
            self._state = 0
            return
        try:
            opts = FaceDetectorOptions(
                base_options=BaseOptions(model_asset_path=path),
                running_mode=RunningMode.IMAGE,
                min_detection_confidence=0.5)
            with suppress_native_stderr():   # 네이티브 초기화 로그(absl/TFLite) 억제
                self._detector = FaceDetector.create_from_options(opts)
            self._state = 2   # loaded
        except Exception as e:
            print(f"[FaceDetection] 모델 로드 실패: {e}", file=sys.stderr)
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
            result = self._detector.detect(image)
            dets = result.detections if result else None
            if dets and len(dets[0].keypoints) >= 6:
                data, raw = self._compute(dets[0].bounding_box, dets[0].keypoints, w, h)
                with self._lock:
                    self._data = data
                    self._raw = raw
                    self._detected = True
            else:
                with self._lock:
                    self._data = _blank_data()
                    self._raw = None
                    self._detected = False

    def _compute(self, box, kps, w, h):
        # 브라우저와 동일한 센터좌표(원점=중앙, y-up). Tasks API box 는 픽셀(top-left) 기준.
        hw, hh = w / 2, h / 2
        ox, oy, bw, bh = box.origin_x, box.origin_y, box.width, box.height   # pixels
        data = _blank_data()
        f = data['face']
        f['x'] = int(ox + bw / 2 - hw)
        f['y'] = int(hh - (oy + bh / 2))
        f['min_x'] = int(ox - hw)
        f['min_y'] = int(hh - (oy + bh))
        f['max_x'] = int(ox + bw - hw)
        f['max_y'] = int(hh - oy)
        f['width'] = abs(f['max_x'] - f['min_x'])
        f['height'] = abs(f['max_y'] - f['min_y'])
        f['area'] = f['width'] * f['height']

        def _cx(p): return int(p.x * w - hw)
        def _cy(p): return int(hh - p.y * h)
        # keypoints(정규화): 0 left eye, 1 right eye, 2 nose, 3 mouth, 4 left ear, 5 right ear
        data['eye']['left'] = {'x': _cx(kps[0]), 'y': _cy(kps[0])}
        data['eye']['right'] = {'x': _cx(kps[1]), 'y': _cy(kps[1])}
        data['nose'] = {'x': _cx(kps[2]), 'y': _cy(kps[2])}
        data['mouth'] = {'x': _cx(kps[3]), 'y': _cy(kps[3])}
        data['ear']['left'] = {'x': _cx(kps[4]), 'y': _cy(kps[4])}
        data['ear']['right'] = {'x': _cx(kps[5]), 'y': _cy(kps[5])}

        # 오버레이용 원본픽셀 (top-left, y-down)
        pt1 = (int(ox), int(oy))
        pt2 = (int(ox + bw), int(oy + bh))
        pts = [(int(p.x * w), int(p.y * h)) for p in kps]
        return data, (pt1, pt2, pts)

    # ── 조회 (lock 스냅샷) ──
    def _is_detected(self):
        with self._lock:
            return self._detected and (self._one_detection or self._continuous)

    def _get_coords(self):
        dict = self._data
        face = dict['face']
        return {
            'face.x': face['x'], 'face.y': face['y'],
            'face.min_x': face['min_x'], 'face.max_x': face['max_x'],
            'face.min_y': face['min_y'], 'face.max_y': face['max_y'],
            'face.width': face['width'], 'face.height': face['height'], 'face.area': face['area'],
            'eye.left.x': dict['eye']['left']['x'], 'eye.left.y': dict['eye']['left']['y'],
            'eye.right.x': dict['eye']['right']['x'], 'eye.right.y': dict['eye']['right']['y'],
            'ear.left.x': dict['ear']['left']['x'], 'ear.left.y': dict['ear']['left']['y'],
            'ear.right.x': dict['ear']['right']['x'], 'ear.right.y': dict['ear']['right']['y'],
            'nose.x': dict['nose']['x'], 'nose.y': dict['nose']['y'],
            'mouth.x': dict['mouth']['x'], 'mouth.y': dict['mouth']['y'],
        }

    def _has_drawing(self):
        with self._lock:
            return self._raw is not None

    def _draw_overlay(self, frame):
        with self._lock:
            raw = self._raw
        if raw is None:
            return
        pt1, pt2, pts = raw
        cv2.rectangle(frame, pt1, pt2, (13, 173, 255), 2)   # 주황 (#FFAD0D)
        for p in pts:
            cv2.circle(frame, p, 4, (0, 0, 255), -1)        # 랜드마크 빨강

    def _check_permission(self, source):
        # macOS: 카메라 권한 미허용 시 검은 프레임만 옴
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
            "[FaceDetection] 카메라 입력이 감지되지 않습니다.\n"
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


class FaceDetectionRoboid(Roboid):
    def __init__(self, index):
        from robomation.ai.face_detection import FaceDetection
        super(FaceDetectionRoboid, self).__init__(FaceDetection.ID, "FaceDetection", 0xA0200000)
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
        self._prev_state = 0       # model_state loading→loaded 엣지 추적

        self._create_model()

    def _create_model(self):
        from robomation.ai.face_detection import FaceDetection
        dict = self._device_dict = {}
        # Effectors
        dict[FaceDetection.CAMERA_DEVICE] = self._camera_device  = self._add_device(FaceDetection.CAMERA_DEVICE, "CameraDevice", DeviceType.EFFECTOR, DataType.INTEGER, 1, -1, 1000, -1)
        dict[FaceDetection.DISPLAY] = self._display_device = self._add_device(FaceDetection.DISPLAY, "Display", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 1)

        # Commands
        dict[FaceDetection.LOAD_MODEL]         = self._load_model_device         = self._add_device(FaceDetection.LOAD_MODEL,         "LoadModel",         DeviceType.COMMAND, DataType.INTEGER, 1, 0, 1, 0)
        dict[FaceDetection.DETECT_ONCE]        = self._detect_once_device        = self._add_device(FaceDetection.DETECT_ONCE,        "DetectOnce",        DeviceType.COMMAND, DataType.INTEGER, 1, 0, 1, 0)
        dict[FaceDetection.DETECT_CONTINUOUS]  = self._detect_continuous_device  = self._add_device(FaceDetection.DETECT_CONTINUOUS,  "DetectContinuous",  DeviceType.COMMAND, DataType.INTEGER, 1, 0, 1, 0)

        # Sensors
        dict[FaceDetection.MODEL_STATE] = self._model_state_device = self._add_device(FaceDetection.MODEL_STATE, "ModelState", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 2, 0)
        dict[FaceDetection.DETECTED]    = self._detected_device    = self._add_device(FaceDetection.DETECTED,    "Detected",    DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)

        # 좌표 (FaceDetection._COORD_DEVICE 맵 기반으로 개별 SENSOR 디바이스 생성)
        # 기본 범위 _range[0]~_range[1], width/height 는 0~2*_range[1], area 는 0~_range[1]*_range[1]
        _range = [-10000, 10000]
        self._coord_devices = []
        for key, dev_id in FaceDetection._COORD_DEVICE.items():
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
        dict[FaceDetection.LOAD_MODEL_STATE] = self._load_model_state_device = self._add_device(FaceDetection.LOAD_MODEL_STATE, "LoadModelState", DeviceType.EVENT, DataType.INTEGER, 1, 0, 0, 0)

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

        self._engine = _FaceEngine()
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
            super(FaceDetectionRoboid, self)._dispose()
            self._release()

    def _reset(self):
        super(FaceDetectionRoboid, self)._reset()
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
