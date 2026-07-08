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
from mediapipe.tasks.python.vision import FaceLandmarker, FaceLandmarkerOptions, RunningMode
from mediapipe.tasks.python.vision.face_landmarker import FaceLandmarksConnections as _FLC

from robomation.core.runner import Runner
from robomation.core.model import DeviceType, DataType, Roboid
from robomation.ai._internal._camera import _CaptureSource
from robomation.ai._internal._model import ensure_model, suppress_native_stderr


# 미리 학습된 얼굴 메시 모델 (MediaPipe FaceLandmarker, Tasks API; 468 메시 + 홍채)
# 패키지 번들(robomation/ai/models) 우선, 없으면 캐시/다운로드. (오프라인 지원)
_MODEL_FILE = 'face_landmarker.task'
_MODEL_URL = 'https://storage.googleapis.com/mediapipe-models/face_landmarker/face_landmarker/float16/1/face_landmarker.task'

# 랜드마크 개수 (FaceMesh 468 + 홍채 좌/우 각 5 = 478)
_NUM_KEYPOINTS = 468
_NUM_IRIS_KEYPOINTS = 5

# _calcValue 용 부위별 랜드마크 인덱스
_LEFT_EYES  = [33, 7, 163, 144, 145, 153, 154, 155, 133, 173, 157, 158, 159, 160, 161, 246]
_RIGHT_EYES = [263, 249, 390, 373, 374, 380, 381, 382, 362, 398, 384, 385, 386, 387, 388, 466]
_OUTER_LIPS = [61, 185, 40, 39, 37, 0, 267, 269, 270, 409, 291, 375, 321, 405, 314, 17, 84, 181, 91, 146]

# 좌표 부위 정의 (DetailedFaceDetection._COORD_DEVICE 키와 일치해야 함)
_SQUARE_FIELDS = ('x', 'y', 'min_x', 'max_x', 'min_y', 'max_y', 'width', 'height', 'area')
_PARTS_SQUARE  = ('face', 'eye.left', 'eye.right', 'mouth')
_PARTS_XY      = ('nose', 'lip.up', 'lip.down', 'lip.left', 'lip.right', 'pupil.left', 'pupil.right')

# 메시 토폴로지 (FaceLandmarksConnections → (start, end) 엣지 리스트). 거대 배열 직접 박지 않음.
def _edges(conn):
    return [(c.start, c.end) for c in conn]

_TESS  = _edges(_FLC.FACE_LANDMARKS_TESSELATION)
_OVAL  = _edges(_FLC.FACE_LANDMARKS_FACE_OVAL)
_LEYE  = _edges(_FLC.FACE_LANDMARKS_LEFT_EYE)  + _edges(_FLC.FACE_LANDMARKS_LEFT_EYEBROW)
_REYE  = _edges(_FLC.FACE_LANDMARKS_RIGHT_EYE) + _edges(_FLC.FACE_LANDMARKS_RIGHT_EYEBROW)
_LIPS  = _edges(_FLC.FACE_LANDMARKS_LIPS)
_IRIS  = _edges(_FLC.FACE_LANDMARKS_LEFT_IRIS) + _edges(_FLC.FACE_LANDMARKS_RIGHT_IRIS)


def _blank_data():
    d = {}
    for p in _PARTS_SQUARE:
        for f in _SQUARE_FIELDS:
            d[p + '.' + f] = 0
    for p in _PARTS_XY:
        d[p + '.x'] = 0
        d[p + '.y'] = 0
    return d


class _FaceMeshEngine:
    """MediaPipe FaceLandmarker + 바인딩된 카메라(_CaptureSource) 래핑. _FaceEngine 대응."""

    def __init__(self):
        self._detector = None
        self._source = None
        self._camera_index = -1
        self._state = 0            # 0 none / 1 loading / 2 loaded
        self._detected = False
        self._one_detection = False
        self._continuous = False
        self._data = _blank_data()
        self._raw = None           # 오버레이용 원본픽셀 랜드마크 [(x,y), ...]
        self._opened = False
        self._closed = False
        self._thread = None        # 연속 인식 스레드
        self._lock = threading.Lock()         # _data/_raw/_detected/_state 보호
        self._detect_lock = threading.Lock()  # detector.detect 직렬화

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
        path = ensure_model(_MODEL_FILE, _MODEL_URL, 'DetailedFaceDetection')
        if path is None:
            self._state = 0
            return
        try:
            opts = FaceLandmarkerOptions(
                base_options=BaseOptions(model_asset_path=path),
                running_mode=RunningMode.IMAGE,
                num_faces=1,
                min_face_detection_confidence=0.5)
            with suppress_native_stderr():   # 네이티브 초기화 로그(absl/TFLite) 억제
                self._detector = FaceLandmarker.create_from_options(opts)
            self._state = 2   # loaded
        except Exception as e:
            print(f"[DetailedFaceDetection] 모델 로드 실패: {e}", file=sys.stderr)
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
            faces = result.face_landmarks if result else None
            if faces and len(faces) > 0 and len(faces[0]) > 0:
                data, raw = self._compute(faces[0], w, h)
                with self._lock:
                    self._data = data
                    self._raw = raw
                    self._detected = True
            else:
                with self._lock:
                    self._data = _blank_data()
                    self._raw = None
                    self._detected = False

    def _compute(self, lm, w, h):
        # 브라우저와 동일한 센터좌표(원점=중앙, y-up). Tasks API 랜드마크는 정규화(0..1).
        hw, hh = w / 2, h / 2
        n = len(lm)
        px = [lm[i].x * w for i in range(n)]
        py = [lm[i].y * h for i in range(n)]
        data = _blank_data()

        self._calc_square(data, 'face', px, py, range(n), hw, hh)   # 전체 랜드마크
        self._calc_square(data, 'eye.left', px, py, _LEFT_EYES, hw, hh)
        self._calc_square(data, 'eye.right', px, py, _RIGHT_EYES, hw, hh)
        self._calc_square(data, 'mouth', px, py, _OUTER_LIPS, hw, hh)

        def _cx(i): return int(px[i] - hw)
        def _cy(i): return int(hh - py[i])
        data['nose.x'], data['nose.y'] = _cx(1), _cy(1)            # 코끝
        data['lip.up.x'], data['lip.up.y'] = _cx(0), _cy(0)        # 윗입술 중앙
        data['lip.down.x'], data['lip.down.y'] = _cx(17), _cy(17)  # 아랫입술 중앙
        data['lip.left.x'], data['lip.left.y'] = _cx(61), _cy(61)
        data['lip.right.x'], data['lip.right.y'] = _cx(291), _cy(291)
        if n > _NUM_KEYPOINTS:                                     # 홍채(refine) 있으면
            data['pupil.right.x'], data['pupil.right.y'] = _cx(_NUM_KEYPOINTS), _cy(_NUM_KEYPOINTS)
            j = _NUM_KEYPOINTS + _NUM_IRIS_KEYPOINTS
            if n > j:
                data['pupil.left.x'], data['pupil.left.y'] = _cx(j), _cy(j)

        pts = [(int(px[i]), int(py[i])) for i in range(n)]         # 오버레이용 원본픽셀(y-down)
        return data, pts

    def _calc_square(self, data, part, px, py, indices, hw, hh):
        minX = minY = float('inf')
        maxX = maxY = float('-inf')
        sumX = sumY = 0.0
        N = 0
        for i in indices:
            x, y = px[i], py[i]
            if x < minX: minX = x
            if x > maxX: maxX = x
            if y < minY: minY = y
            if y > maxY: maxY = y
            sumX += x
            sumY += y
            N += 1
        tlx = int(minX - hw); tly = int(hh - maxY)
        brx = int(maxX - hw); bry = int(hh - minY)
        if N > 0:
            cx = int(sumX / N - hw); cy = int(hh - sumY / N)
        else:
            cx = int((tlx + brx) / 2); cy = int((tly + bry) / 2)
        data[part + '.x'], data[part + '.y'] = cx, cy
        data[part + '.min_x'], data[part + '.max_x'] = tlx, brx
        data[part + '.min_y'], data[part + '.max_y'] = tly, bry
        data[part + '.width'] = abs(brx - tlx)
        data[part + '.height'] = abs(bry - tly)
        data[part + '.area'] = abs(brx - tlx) * abs(bry - tly)

    # ── 조회 (lock 스냅샷) ──
    def _is_detected(self):
        with self._lock:
            return self._detected and (self._one_detection or self._continuous)

    def _get_coords(self):
        return self._data

    def _has_drawing(self):
        with self._lock:
            return self._raw is not None

    def _draw_overlay(self, frame):
        # 브라우저 drawFrame 대응: 메시(은색) → 윤곽(살색) → 좌눈(초록)/우눈(파랑) → 입술(빨강) → 홍채(빨강)
        with self._lock:
            pts = self._raw
        if pts is None:
            return
        _draw_edges(frame, pts, _TESS, (192, 192, 192), 1)   # 테셀레이션 (은색)
        _draw_edges(frame, pts, _OVAL, (177, 206, 251), 2)   # 얼굴 윤곽 (#fbceb1 살색)
        _draw_edges(frame, pts, _LEYE, (0, 255, 0), 1)       # 좌안+눈썹 (초록)
        _draw_edges(frame, pts, _REYE, (255, 0, 0), 1)       # 우안+눈썹 (파랑)
        _draw_edges(frame, pts, _LIPS, (0, 0, 255), 2)       # 입술 (빨강)
        if len(pts) > _NUM_KEYPOINTS:
            _draw_edges(frame, pts, _IRIS, (0, 0, 255), 1)   # 홍채 (빨강)

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
            "[DetailedFaceDetection] 카메라 입력이 감지되지 않습니다.\n"
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


def _draw_edges(frame, pts, edges, color, thick):
    n = len(pts)
    for a, b in edges:
        if a < n and b < n:
            cv2.line(frame, pts[a], pts[b], color, thick, cv2.LINE_AA)


class DetailedFaceDetectionRoboid(Roboid):
    def __init__(self, index):
        from robomation.ai.detailed_face_detection import DetailedFaceDetection
        super(DetailedFaceDetectionRoboid, self).__init__(DetailedFaceDetection.ID, "DetailedFaceDetection", 0xA0300000)
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
        from robomation.ai.detailed_face_detection import DetailedFaceDetection
        dict = self._device_dict = {}
        # Effectors
        dict[DetailedFaceDetection.CAMERA_DEVICE] = self._camera_device  = self._add_device(DetailedFaceDetection.CAMERA_DEVICE, "CameraDevice", DeviceType.EFFECTOR, DataType.INTEGER, 1, -1, 1000, -1)
        dict[DetailedFaceDetection.DISPLAY] = self._display_device = self._add_device(DetailedFaceDetection.DISPLAY, "Display", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 1)

        # Commands
        dict[DetailedFaceDetection.LOAD_MODEL]         = self._load_model_device         = self._add_device(DetailedFaceDetection.LOAD_MODEL,         "LoadModel",         DeviceType.COMMAND, DataType.INTEGER, 1, 0, 1, 0)
        dict[DetailedFaceDetection.DETECT_ONCE]        = self._detect_once_device        = self._add_device(DetailedFaceDetection.DETECT_ONCE,        "DetectOnce",        DeviceType.COMMAND, DataType.INTEGER, 1, 0, 1, 0)
        dict[DetailedFaceDetection.DETECT_CONTINUOUS]  = self._detect_continuous_device  = self._add_device(DetailedFaceDetection.DETECT_CONTINUOUS,  "DetectContinuous",  DeviceType.COMMAND, DataType.INTEGER, 1, 0, 1, 0)

        # Sensors
        dict[DetailedFaceDetection.MODEL_STATE] = self._model_state_device = self._add_device(DetailedFaceDetection.MODEL_STATE, "ModelState", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 2, 0)
        dict[DetailedFaceDetection.DETECTED]    = self._detected_device    = self._add_device(DetailedFaceDetection.DETECTED,    "Detected",    DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)

        # 좌표 (DetailedFaceDetection._COORD_DEVICE 맵 기반으로 개별 SENSOR 디바이스 생성)
        # 기본 범위 _range[0]~_range[1], width/height 는 0~2*_range[1], area 는 0~_range[1]*_range[1]
        _range = [-10000, 10000]
        self._coord_devices = []
        for key, dev_id in DetailedFaceDetection._COORD_DEVICE.items():
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
        dict[DetailedFaceDetection.LOAD_MODEL_STATE] = self._load_model_state_device = self._add_device(DetailedFaceDetection.LOAD_MODEL_STATE, "LoadModelState", DeviceType.EVENT, DataType.INTEGER, 1, 0, 0, 0)

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

        self._engine = _FaceMeshEngine()
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
            super(DetailedFaceDetectionRoboid, self)._dispose()
            self._release()

    def _reset(self):
        super(DetailedFaceDetectionRoboid, self)._reset()
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
