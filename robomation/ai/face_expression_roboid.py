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
import mediapipe as mp
from mediapipe.tasks.python import BaseOptions
from mediapipe.tasks.python.vision import FaceDetector, FaceDetectorOptions, RunningMode

from robomation.core.runner import Runner
from robomation.core.model import DeviceType, DataType, Roboid
from robomation.ai._internal._camera import _CaptureSource
from robomation.ai._internal._model import ensure_model, suppress_native_stderr


# ── 모델 파일/URL (얼굴=번들 우선, 인식모델=다운로드+캐시) ───────────────────
_ONNX_BASE = 'https://github.com/onnx/models/raw/main/validated/vision/body_analysis'
_FD_FILE  = 'blaze_face_short_range.tflite'
_FD_URL   = 'https://storage.googleapis.com/mediapipe-models/face_detector/blaze_face_short_range/float16/1/blaze_face_short_range.tflite'
_AGE_FILE = 'age_googlenet.onnx'
_AGE_URL  = _ONNX_BASE + '/age_gender/models/age_googlenet.onnx'
_GEN_FILE = 'gender_googlenet.onnx'
_GEN_URL  = _ONNX_BASE + '/age_gender/models/gender_googlenet.onnx'
_EMO_FILE = 'emotion-ferplus-8.onnx'
_EMO_URL  = _ONNX_BASE + '/emotion_ferplus/model/emotion-ferplus-8.onnx'

# 라벨/매핑
_AGE_MID    = [1, 5, 10, 17, 28, 40, 50, 80]            # 8 버킷 중앙값(int) — age()가 숫자 반환
_GENDER     = ['male', 'female']                        # gender_googlenet 출력 순서
# FER+ 출력 순서 → 우리 7 라벨 인덱스 매핑 (contempt 제외)
#   FER+: [neutral, happiness, surprise, sadness, anger, disgust, fear, contempt]
_EMO_MAP = {
    'angry': 4, 'disgusted': 5, 'fearful': 6, 'happy': 1,
    'neutral': 0, 'sad': 3, 'surprised': 2,
}
_MODEL_MEAN = (104.0, 117.0, 123.0)   # age/gender googlenet (Caffe식, BGR)


def _softmax(x):
    x = x - np.max(x)
    e = np.exp(x)
    s = np.sum(e)
    return e / s if s > 0 else e


def _blank_data():
    return {
        'age': -1,
        'gender_class': '', 'gender_detected': 0,
        'gender_conf': {'male': 0.0, 'female': 0.0},
        'expression_class': '', 'expression_detected': 0,
        'expression_conf': {k: 0.0 for k in _EMO_MAP},
    }


class _FaceExpressionEngine:
    """MediaPipe FaceDetector(박스) + cv2.dnn 나이/성별/표정 + 카메라(_CaptureSource) 래핑."""

    def __init__(self):
        self._face_detector = None
        self._age_net = None
        self._gender_net = None
        self._emotion_net = None
        self._source = None
        self._camera_index = -1
        self._state = 0            # 0 none / 1 loading / 2 loaded
        self._detected = False
        self._one_detection = False
        self._continuous = False
        self._data = _blank_data()
        self._box = None           # 오버레이용 얼굴 박스 (x, y, w, h) 픽셀
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
        fd  = ensure_model(_FD_FILE, _FD_URL, 'FaceExpression')
        age = ensure_model(_AGE_FILE, _AGE_URL, 'FaceExpression')
        gen = ensure_model(_GEN_FILE, _GEN_URL, 'FaceExpression')
        emo = ensure_model(_EMO_FILE, _EMO_URL, 'FaceExpression')
        if not (fd and age and gen and emo):
            self._state = 0
            return
        try:
            opts = FaceDetectorOptions(
                base_options=BaseOptions(model_asset_path=fd),
                running_mode=RunningMode.IMAGE,
                min_detection_confidence=0.5)
            with suppress_native_stderr():   # 네이티브 초기화 로그(absl/TFLite) 억제
                self._face_detector = FaceDetector.create_from_options(opts)
            self._age_net = cv2.dnn.readNetFromONNX(age)
            self._gender_net = cv2.dnn.readNetFromONNX(gen)
            self._emotion_net = cv2.dnn.readNetFromONNX(emo)
            self._state = 2   # loaded
        except Exception as e:
            print(f"[FaceExpression] 모델 로드 실패: {e}", file=sys.stderr)
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
        # ~10fps 백그라운드 인식 (3개 네트 추론이라 약간 여유)
        while self._continuous and not self._closed:
            if self._is_loaded():
                frame = self._read_frame()
                if frame is not None:
                    self._process_frame(frame)
            time.sleep(1 / 10)

    def _process_frame(self, frame):
        with self._detect_lock:
            if self._face_detector is None:
                return
            h, w = frame.shape[:2]
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            result = self._face_detector.detect(image)
            dets = result.detections if result else None
            if dets and len(dets) > 0:
                box = dets[0].bounding_box
                pad = 20
                x1 = max(0, int(box.origin_x) - pad)
                y1 = max(0, int(box.origin_y) - pad)
                x2 = min(w, int(box.origin_x + box.width) + pad)
                y2 = min(h, int(box.origin_y + box.height) + pad)
                crop = frame[y1:y2, x1:x2]
                if crop.shape[0] >= 10 and crop.shape[1] >= 10:
                    data = self._compute(crop)
                    with self._lock:
                        self._data = data
                        self._box = (x1, y1, x2 - x1, y2 - y1)
                        self._detected = True
                    return
            with self._lock:
                self._data = _blank_data()
                self._box = None
                self._detected = False

    def _compute(self, crop):
        data = _blank_data()
        # 나이/성별 (동일 blob: 224x224, Caffe식 mean, BGR)
        blob = cv2.dnn.blobFromImage(crop, 1.0, (224, 224), _MODEL_MEAN, swapRB=False)
        self._age_net.setInput(blob)
        age_out = _softmax(self._age_net.forward()[0])
        data['age'] = _AGE_MID[int(np.argmax(age_out))]

        self._gender_net.setInput(blob)
        gen_out = _softmax(self._gender_net.forward()[0])
        gi = int(np.argmax(gen_out))
        data['gender_class'] = _GENDER[gi]
        data['gender_conf'] = {
            'male': round(float(gen_out[0]), 3), 
            'female': round(float(gen_out[1]), 3),
        }

        # 표정 (64x64 grayscale, FER+)
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        eblob = cv2.dnn.blobFromImage(gray, 1.0, (64, 64))
        self._emotion_net.setInput(eblob)
        emo_out = _softmax(self._emotion_net.forward()[0])
        conf = {label: float(emo_out[idx]) for label, idx in _EMO_MAP.items()}
        data['expression_conf'] = conf
        data['expression_class'] = max(conf, key=conf.get)
        return data

    # ── 조회 (lock 스냅샷) ──
    def _get_data(self):
        with self._lock:
            d = self._data
            det = 1 if (self._detected and (self._one_detection or self._continuous)) else 0
            return {
                'age': d['age'],
                'gender_class': d['gender_class'] if det else '',
                'gender_detected': det,
                'gender_conf': d['gender_conf'],
                'expression_class': d['expression_class'] if det else '',
                'expression_detected': det,
                'expression_conf': d['expression_conf'],
            }

    def _has_drawing(self):
        with self._lock:
            return self._box is not None

    def _draw_overlay(self, frame):
        with self._lock:
            box = self._box
            d = self._data
        if box is None:
            return
        x, y, w, h = box
        cv2.rectangle(frame, (x, y), (x + w, y + h), (97, 128, 243), 2)   # 주황 (#F38061)
        label = "{}, {}, {}".format(d['age'], d['gender_class'], d['expression_class'])
        ty = y - 10 if y > 14 else y + h + 18
        cv2.putText(frame, label, (x, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (97, 128, 243), 2, cv2.LINE_AA)

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
            "[FaceExpression] 카메라 입력이 감지되지 않습니다.\n"
            "      macOS 라면 Python 을 실행한 앱(VSCode/터미널 등)의 카메라 권한이 꺼져 있을 수 있습니다.\n"
            "      시스템 설정 → 개인정보 보호 및 보안 → 카메라 에서 해당 앱을 허용한 뒤,\n"
            "      그 앱을 완전히 종료했다가 다시 실행하세요.",
            file=sys.stderr,
        )
        permission.open_camera_settings()

    def _close(self):
        self._closed = True
        self._set_mode(False)
        if self._face_detector is not None:
            try:
                with suppress_native_stderr():   # 종료 시 clearcut 텔레메트리 로그 억제
                    self._face_detector.close()
            except Exception:
                pass
            self._face_detector = None
        self._age_net = None
        self._gender_net = None
        self._emotion_net = None
        if self._source is not None:
            self._source.release()
            self._source = None
        self._opened = False
        self._state = 0


class FaceExpressionRoboid(Roboid):
    def __init__(self, index):
        from robomation.ai.face_expression import FaceExpression
        super(FaceExpressionRoboid, self).__init__(FaceExpression.ID, "FaceExpression", 0xA0400000)
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
        from robomation.ai.face_expression import FaceExpression
        dict = self._device_dict = {}
        # Effectors
        dict[FaceExpression.CAMERA_DEVICE] = self._camera_device  = self._add_device(FaceExpression.CAMERA_DEVICE, "CameraDevice", DeviceType.EFFECTOR, DataType.INTEGER, 1, -1, 1000, -1)
        dict[FaceExpression.DISPLAY] = self._display_device = self._add_device(FaceExpression.DISPLAY, "Display", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 1)

        # Commands
        dict[FaceExpression.LOAD_MODEL]        = self._load_model_device        = self._add_device(FaceExpression.LOAD_MODEL,        "LoadModel",        DeviceType.COMMAND, DataType.INTEGER, 1, 0, 1, 0)
        dict[FaceExpression.DETECT_ONCE]       = self._detect_once_device       = self._add_device(FaceExpression.DETECT_ONCE,       "DetectOnce",       DeviceType.COMMAND, DataType.INTEGER, 1, 0, 1, 0)
        dict[FaceExpression.DETECT_CONTINUOUS] = self._detect_continuous_device = self._add_device(FaceExpression.DETECT_CONTINUOUS, "DetectContinuous", DeviceType.COMMAND, DataType.INTEGER, 1, 0, 1, 0)

        # Sensors
        dict[FaceExpression.MODEL_STATE]         = self._model_state_device         = self._add_device(FaceExpression.MODEL_STATE,         "ModelState",         DeviceType.SENSOR, DataType.INTEGER, 1, 0, 2, 0)
        dict[FaceExpression.AGE]                 = self._age_device                 = self._add_device(FaceExpression.AGE,                 "Age",                DeviceType.SENSOR, DataType.INTEGER, 1, -1, 200, -1)
        dict[FaceExpression.GENDER_DETECTED]     = self._gender_detected_device     = self._add_device(FaceExpression.GENDER_DETECTED,     "GenderDetected",     DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[FaceExpression.GENDER_CLASS]        = self._gender_class_device        = self._add_device(FaceExpression.GENDER_CLASS,        "GenderClass",        DeviceType.SENSOR, DataType.STRING,  1, 0, 0, '')
        dict[FaceExpression.EXPRESSION_DETECTED] = self._expression_detected_device = self._add_device(FaceExpression.EXPRESSION_DETECTED, "ExpressionDetected", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[FaceExpression.EXPRESSION_CLASS]    = self._expression_class_device    = self._add_device(FaceExpression.EXPRESSION_CLASS,    "ExpressionClass",    DeviceType.SENSOR, DataType.STRING,  1, 0, 0, '')

        # 신뢰도 (label → FLOAT SENSOR 디바이스)
        self._gender_conf_devices = []
        for label, dev_id in FaceExpression._GENDER_CONF_DEVICE.items():
            dev = self._add_device(dev_id, "GenderConf_" + label, DeviceType.SENSOR, DataType.FLOAT, 1, 0, 1, 0)
            dict[dev_id] = dev
            self._gender_conf_devices.append((dev, label))
        self._expression_conf_devices = []
        for label, dev_id in FaceExpression._EXPRESSION_CONF_DEVICE.items():
            dev = self._add_device(dev_id, "ExpressionConf_" + label, DeviceType.SENSOR, DataType.FLOAT, 1, 0, 1, 0)
            dict[dev_id] = dev
            self._expression_conf_devices.append((dev, label))

        # Event
        dict[FaceExpression.LOAD_MODEL_STATE] = self._load_model_state_device = self._add_device(FaceExpression.LOAD_MODEL_STATE, "LoadModelState", DeviceType.EVENT, DataType.INTEGER, 1, 0, 0, 0)

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

        self._engine = _FaceExpressionEngine()
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
            super(FaceExpressionRoboid, self)._dispose()
            self._release()

    def _reset(self):
        super(FaceExpressionRoboid, self)._reset()
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

        d = self._engine._get_data()                      # lock 스냅샷
        self._age_device._put(d['age'])
        self._gender_detected_device._put(d['gender_detected'])
        self._gender_class_device._put(d['gender_class'])
        self._expression_detected_device._put(d['expression_detected'])
        self._expression_class_device._put(d['expression_class'])
        for dev, label in self._gender_conf_devices:
            dev._put(d['gender_conf'][label])
        for dev, label in self._expression_conf_devices:
            dev._put(d['expression_conf'][label])

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
