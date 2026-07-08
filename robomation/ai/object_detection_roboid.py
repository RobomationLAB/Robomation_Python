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
from mediapipe.tasks.python.vision import ObjectDetector, ObjectDetectorOptions, RunningMode

from robomation.core.runner import Runner
from robomation.core.model import DeviceType, DataType, Roboid
from robomation.ai._internal._camera import _CaptureSource
from robomation.ai._internal._model import ensure_model, suppress_native_stderr
from robomation.ai.object_detection import _LABEL_INDEX, _POS_LIST, _NUM_CLASSES


# COCO 사물 인식 모델 (MediaPipe ObjectDetector, Tasks API; EfficientDet-Lite0).
_MODEL_FILE = 'efficientdet_lite0.tflite'
_MODEL_URL = 'https://storage.googleapis.com/mediapipe-models/object_detector/efficientdet_lite0/float16/1/efficientdet_lite0.tflite'


def _blank_arrays():
    arrays = {pos: [0] * _NUM_CLASSES for pos in _POS_LIST}
    conf = [0.0] * _NUM_CLASSES
    return arrays, conf


class _ObjectEngine:
    """MediaPipe ObjectDetector + 바인딩된 카메라(_CaptureSource) 래핑."""

    def __init__(self):
        self._detector = None
        self._source = None
        self._camera_index = -1
        self._max_count = 5
        self._confidence = 0.5
        self._state = 0            # 0 none / 1 loading / 2 loaded
        self._detected = False
        self._one_detection = False
        self._continuous = False
        self._arrays, self._conf = _blank_arrays()
        self._draw = []            # [(x, y, w, h, label, score), ...] 픽셀
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
        path = ensure_model(_MODEL_FILE, _MODEL_URL, 'ObjectDetection')
        if path is None:
            return None
        opts = ObjectDetectorOptions(
            base_options=BaseOptions(model_asset_path=path),
            running_mode=RunningMode.IMAGE,
            max_results=max(1, int(self._max_count)),
            score_threshold=float(self._confidence))
        with suppress_native_stderr():   # 네이티브 초기화 로그(absl/TFLite) 억제
            return ObjectDetector.create_from_options(opts)

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
            print(f"[ObjectDetection] 모델 로드 실패: {e}", file=sys.stderr)
            self._state = 0

    def _set_params(self, max_count, confidence):
        if max_count == self._max_count and confidence == self._confidence:
            return
        self._max_count = max_count
        self._confidence = confidence
        if self._state == 2:   # 로드됐으면 옵션 반영 위해 detector 재생성
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
                print(f"[ObjectDetection] detector 재생성 실패: {e}", file=sys.stderr)

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
            dets = result.detections if result else None
        if dets and len(dets) > 0:
            arrays, conf, draw = self._compute(dets, w, h)
            with self._lock:
                self._arrays = arrays
                self._conf = conf
                self._draw = draw
                self._detected = True
        else:
            with self._lock:
                self._arrays, self._conf = _blank_arrays()
                self._draw = []
                self._detected = False

    def _compute(self, dets, w, h):
        hw, hh = w / 2, h / 2
        arrays, conf = _blank_arrays()
        best_area = {}   # idx → area (클래스별 최대 면적 1개 유지)
        draw = []
        for det in dets:
            cat = det.categories[0] if det.categories else None
            if cat is None:
                continue
            idx = _LABEL_INDEX.get(cat.category_name.replace(' ', '_'))
            if idx is None:
                continue
            box = det.bounding_box
            bw, bh = int(box.width), int(box.height)
            tlx = int(box.origin_x - hw)
            tly = int(hh - (box.origin_y + bh))
            brx = tlx + bw
            bry = int(hh - box.origin_y)
            area = bw * bh
            if idx in best_area and area <= best_area[idx]:
                continue
            best_area[idx] = area
            arrays['x'][idx] = int((tlx + brx) / 2)
            arrays['y'][idx] = int((tly + bry) / 2)
            arrays['min_x'][idx] = tlx
            arrays['max_x'][idx] = brx
            arrays['min_y'][idx] = tly
            arrays['max_y'][idx] = bry
            arrays['width'][idx] = bw
            arrays['height'][idx] = bh
            arrays['area'][idx] = area
            conf[idx] = round(float(cat.score), 3)
            draw.append((int(box.origin_x), int(box.origin_y), bw, bh, cat.category_name, round(float(cat.score), 3)))
        return arrays, conf, draw

    # ── 조회 (lock 스냅샷) ──
    def _is_detected(self):
        with self._lock:
            return self._detected and (self._one_detection or self._continuous)

    def _get_data(self):
        return self._arrays, self._conf

    def _has_drawing(self):
        with self._lock:
            return len(self._draw) > 0

    def _draw_overlay(self, frame):
        with self._lock:
            draws = list(self._draw)
        for x, y, w, h, label, score in draws:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (143, 76, 153), 2)   # 보라 (#994C8F)
            text = "{} {:.2f}".format(label, score)
            ty = y - 10 if y > 14 else y + 18
            cv2.putText(frame, text, (x, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (143, 76, 153), 2, cv2.LINE_AA)

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
            "[ObjectDetection] 카메라 입력이 감지되지 않습니다.\n"
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


class ObjectDetectionRoboid(Roboid):
    def __init__(self, index):
        from robomation.ai.object_detection import ObjectDetection
        super(ObjectDetectionRoboid, self).__init__(ObjectDetection.ID, "ObjectDetection", 0xA0700000)
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
        self._max_count = 5
        self._confidence = 0.5
        self._load_model_written = False
        self._detect_once_written = False
        self._detect_continuous = 0
        self._detect_continuous_written = False
        self._prev_state = 0

        self._create_model()

    def _create_model(self):
        from robomation.ai.object_detection import ObjectDetection
        dict = self._device_dict = {}
        # Effectors
        dict[ObjectDetection.CAMERA_DEVICE] = self._camera_device  = self._add_device(ObjectDetection.CAMERA_DEVICE, "CameraDevice", DeviceType.EFFECTOR, DataType.INTEGER, 1, -1, 1000, -1)
        dict[ObjectDetection.DISPLAY]    = self._display_device    = self._add_device(ObjectDetection.DISPLAY, "Display", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 1)
        dict[ObjectDetection.MAX_COUNT]  = self._max_count_device  = self._add_device(ObjectDetection.MAX_COUNT, "MaxCount", DeviceType.EFFECTOR, DataType.INTEGER, 1, 1, _NUM_CLASSES, 5)
        dict[ObjectDetection.CONFIDENCE] = self._confidence_device = self._add_device(ObjectDetection.CONFIDENCE, "Confidence", DeviceType.EFFECTOR, DataType.FLOAT, 1, 0, 1, 0.5)

        # Commands
        dict[ObjectDetection.LOAD_MODEL]        = self._load_model_device        = self._add_device(ObjectDetection.LOAD_MODEL,        "LoadModel",        DeviceType.COMMAND, DataType.INTEGER, 1, 0, 1, 0)
        dict[ObjectDetection.DETECT_ONCE]       = self._detect_once_device       = self._add_device(ObjectDetection.DETECT_ONCE,       "DetectOnce",       DeviceType.COMMAND, DataType.INTEGER, 1, 0, 1, 0)
        dict[ObjectDetection.DETECT_CONTINUOUS] = self._detect_continuous_device = self._add_device(ObjectDetection.DETECT_CONTINUOUS, "DetectContinuous", DeviceType.COMMAND, DataType.INTEGER, 1, 0, 1, 0)

        # Sensors
        dict[ObjectDetection.MODEL_STATE] = self._model_state_device = self._add_device(ObjectDetection.MODEL_STATE, "ModelState", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 2, 0)
        dict[ObjectDetection.DETECTED]    = self._detected_device    = self._add_device(ObjectDetection.DETECTED,    "Detected",    DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)

        # 좌표 배열 디바이스 (dimension = 80 클래스). pos 별 INT + confidence FLOAT
        _range = [-10000, 10000]
        self._pos_devices = []
        for pos in _POS_LIST:
            dev_id = ObjectDetection._POS_DEVICE[pos]
            dev = self._add_device(dev_id, "Object_" + pos, DeviceType.SENSOR, DataType.INTEGER, _NUM_CLASSES, _range[0], _range[1] * _range[1], 0)
            dict[dev_id] = dev
            self._pos_devices.append((dev, pos))
        dict[ObjectDetection.OBJECT_CONFIDENCE] = self._confidence_array_device = self._add_device(ObjectDetection.OBJECT_CONFIDENCE, "Object_confidence", DeviceType.SENSOR, DataType.FLOAT, _NUM_CLASSES, 0, 1, 0)

        # Event
        dict[ObjectDetection.LOAD_MODEL_STATE] = self._load_model_state_device = self._add_device(ObjectDetection.LOAD_MODEL_STATE, "LoadModelState", DeviceType.EVENT, DataType.INTEGER, 1, 0, 0, 0)

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

        self._engine = _ObjectEngine()
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
            super(ObjectDetectionRoboid, self)._dispose()
            self._release()

    def _reset(self):
        super(ObjectDetectionRoboid, self)._reset()
        self._camera_index = -1
        self._display = 0
        self._max_count = 5
        self._confidence = 0.5
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
            self._confidence = self._confidence_device.read()
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

        arrays, conf = self._engine._get_data()           # lock 스냅샷
        for dev, pos in self._pos_devices:
            dev._put_array(arrays[pos])
        self._confidence_array_device._put_array(conf)

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
            confidence = self._confidence
            load_model = self._load_model_written
            detect_once = self._detect_once_written
            detect_continuous = self._detect_continuous
            detect_continuous_written = self._detect_continuous_written
            self._load_model_written = False
            self._detect_once_written = False
            self._detect_continuous_written = False

        if camera_index != self._engine._camera_index:
            self._engine._bind_camera(camera_index)
        if max_count != self._engine._max_count or confidence != self._engine._confidence:
            self._engine._set_params(max_count, confidence)
        if load_model:
            self._engine._load_model()
        if detect_once:
            self._engine._detect_once()
        if detect_continuous_written:
            self._engine._set_mode(detect_continuous == 1)
