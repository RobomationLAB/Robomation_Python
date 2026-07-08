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

from robomation.core.runner import Runner
from robomation.core.model import DeviceType, DataType, Roboid
from robomation.ai.asr import ASR 

import speech_recognition as sr
import sounddevice as sd

# sounddevice 기반 SpeechRecognition AudioSource.
class _SoundDeviceSource(sr.AudioSource):
    class _Stream:
        def __init__(self, raw):
            self._raw = raw

        def read(self, size):
            data, _overflowed = self._raw.read(size)  # RawInputStream.read → (buffer, bool)
            return bytes(data)

        def close(self):
            self._raw.stop()
            self._raw.close()

    def __init__(self, sample_rate=16000, chunk=1024, device=None):
        self.SAMPLE_RATE = sample_rate
        self.SAMPLE_WIDTH = 2   # int16 → 2 bytes
        self.CHUNK = chunk
        self.device = device
        self.stream = None

    def __enter__(self):
        raw = sd.RawInputStream(samplerate=self.SAMPLE_RATE, blocksize=self.CHUNK,
                                device=self.device, dtype='int16', channels=1)
        raw.start()
        self.stream = _SoundDeviceSource._Stream(raw)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.stream is not None:
            self.stream.close()
            self.stream = None

# SpeechRecognition(Google Web Speech) + sounddevice 기반 음성인식 엔진
class _ASREngine:
    def __init__(self):
        self._lang = 'ko-KR'
        self._state = 0           # onstart→1 / onend·onresult→0
        self._result = ''
        self._new_result = False  # onresult 에서 set, roboid 가 _decode 에서 소비
        self._opened = False      # open() 성공 여부 (로봇 connector.is_connected 대응)
        self._recognizer = None
        self._source = None
        self._stop_fn = None      # listen_in_background 정지 함수
        self._lock = threading.Lock()

    def open(self):
        try:
            self._recognizer = sr.Recognizer()
            self._source = _SoundDeviceSource()
            with self._source as source:  # 주변 소음 보정 (1회)
                self._recognizer.adjust_for_ambient_noise(source, duration=0.3)
                # 마이크가 실제로 오디오를 주는지 확인 — macOS 에서 권한 미허용 시 전부 0
                if self._is_silent(source):
                    self._warn_mic_permission()
            self._opened = True
            return True
        except Exception as e:
            print(f"[ASR] 마이크 초기화 실패: {e}", file=sys.stderr)
            self._recognizer = None
            self._source = None
            self._opened = False
            return False

    def _is_silent(self, source, duration=0.2):
        # duration 동안 캡처해 샘플이 전부 0(정확히 무음)이면 True.
        # 실제 마이크는 잡음 바닥 때문에 절대 정확히 0 이 아니므로, 0 = 입력 없음(권한/장치).
        silence = b"\x00" * (source.CHUNK * source.SAMPLE_WIDTH)
        chunks = max(1, int(duration * source.SAMPLE_RATE / source.CHUNK))
        for _ in range(chunks):
            buf = source.stream.read(source.CHUNK)
            if buf != silence[:len(buf)]:
                return False   # 신호 있음
        return True            # 전부 0 → 입력 없음

    def _warn_mic_permission(self):
        from robomation.core import permission
        print(
            "[ASR] 마이크 입력이 감지되지 않습니다.\n"
            "      macOS 라면 Python 을 실행한 앱(VSCode/터미널 등)의 마이크 권한이 꺼져 있을 수 있습니다.\n"
            "      시스템 설정 → 개인정보 보호 및 보안 → 마이크 에서 해당 앱을 허용한 뒤,\n"
            "      그 앱을 완전히 종료했다가 다시 실행하세요.",
            file=sys.stderr,
        )
        permission.open_microphone_settings()   # macOS: 마이크 설정 창 자동 열기

    def is_open(self):
        return self._opened

    def set_lang(self, lang):
        self._lang = lang

    def start(self):
        if self._recognizer is None or self._source is None:
            return
        with self._lock:
            if self._stop_fn is not None:
                return  # 이미 듣는 중이면 return
            self._on_start()
            
            # phrase 단위로 _callback 호출 (에너지 기반 자동 끝점 검출 = Web Speech 와 유사)
            self._stop_fn = self._recognizer.listen_in_background(
                self._source, 
                self._callback, 
                phrase_time_limit=10
            )

    def _callback(self, recognizer, audio):
        # listen_in_background 의 백그라운드 스레드에서 호출됨
        try:
            text = recognizer.recognize_google(audio, language=self._lang)
        except sr.UnknownValueError:
            text = None            # 음성을 알아듣지 못함 → 결과 없이 종료
        except sr.RequestError as e:
            print(f"[ASR] 인식 요청 실패(네트워크?): {e}", file=sys.stderr)
            text = None

        self._stop_background(wait=False)  # 단발성: 첫 phrase 후 정지 (onresult→abort 대응)
        if text:
            self._on_result(text)
        else:
            self._on_end()

    def abort(self):
        self._stop_background(wait=True)
        self._on_end()

    def _stop_background(self, wait=False):
        with self._lock:
            fn = self._stop_fn
            self._stop_fn = None
        if fn is not None:
            # wait=False: 콜백(리스너) 스레드 내에서 호출돼도 안전(self-join 방지)
            # wait=True : 캡처 스레드/스트림이 완전히 닫힐 때까지 대기(종료 시 PortAudio 충돌 방지)
            fn(wait_for_stop=wait)

    def is_active(self):
        return self._state == 1

    def has_result(self):
        return self._new_result

    def read_result(self):
        self._new_result = False
        return self._result

    def close(self):
        self._stop_background(wait=True)
        self._opened = False
        self._state = 0

    # ── 엔진 콜백 (브라우저 onstart/onend/onresult 대응) ──────────────────────
    def _on_start(self):
        self._state = 1

    def _on_end(self):
        self._state = 0

    def _on_result(self, text):
        self._result = text or ''
        self._new_result = True
        self._state = 0


class ASRRoboid(Roboid):
    def __init__(self, index):
        from robomation.ai.asr import ASR
        super(ASRRoboid, self).__init__(ASR.ID, "ASR", 0xA0000000)  # uid = 0xA0000000 + (product_id=0 << 20)
        self._index = index
        self._engine = None          # = _ASREngine (로봇의 SerialConnector 자리)
        self._ready = False
        self._running = False
        self._releasing = 0
        self._thread = None
        self._thread_lock = threading.Lock()

        # ── Motoring 스냅샷 — _request_motoring_data 가 채우고 _encode 가 소비 ──
        self._lang = 'ko-KR'           
        self._listen = 0               
        self._listen_written = False   

        self._create_model()

    def _create_model(self):
        from robomation.ai.asr import ASR
        dict = self._device_dict = {}

        dict[ASR.LANG]         = self._lang_device         = self._add_device(ASR.LANG,         "Lang",        DeviceType.EFFECTOR, DataType.STRING,  1, 0, 0, 'ko-KR')
        dict[ASR.LISTEN]       = self._listen_device       = self._add_device(ASR.LISTEN,       "Listen",      DeviceType.COMMAND,  DataType.INTEGER, 1, 0, 1, 0)
        dict[ASR.RESULT]       = self._result_device       = self._add_device(ASR.RESULT,       "Result",      DeviceType.SENSOR,   DataType.STRING,  1, 0, 0, '')
        dict[ASR.STATE]        = self._state_device        = self._add_device(ASR.STATE,        "State",       DeviceType.SENSOR,   DataType.INTEGER, 1, 0, 1, 0)
        dict[ASR.LISTEN_STATE] = self._listen_state_device = self._add_device(ASR.LISTEN_STATE, "ListenState", DeviceType.EVENT,    DataType.INTEGER, 1, 0, 0, 0)

    def find_device_by_id(self, device_id):
        return self._device_dict.get(device_id)
    
    def _run(self):
        try:
            while self._running or self._releasing > 0:
                if self._decode():            # 로봇의 _receive 대응 (engine → sensor)
                    self._encode()            # 로봇의 _send 대응 (motor → engine)
                    if self._releasing > 0:
                        self._releasing -= 1
                time.sleep(0.01)              # 10ms
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

        # SerialConnector 생성/open 자리 — _ASREngine
        self._engine = _ASREngine()
        if self._engine.open():              
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
            engine.close()

    def _dispose(self):
        if self._is_disposed() == False:
            super(ASRRoboid, self)._dispose()
            self._release()

    def _reset(self):
        super(ASRRoboid, self)._reset()

        # ── Motoring 스냅샷 초기화 ──
        self._lang = 'ko-KR'
        self._listen = 0
        self._listen_written = False

    def _request_motoring_data(self):
        with self._thread_lock:
            # EFFECTOR — 매 사이클 현재 값 반영
            self._lang = self._lang_device.read()

            # COMMAND — _is_written latch 
            if self._listen_device._is_written():
                self._listen = self._listen_device.read()
                self._listen_written = True
        self._clear_written()

    def _decode(self):
        # engine → sensor 
        if self._engine is None or self._engine.is_open() == False:
            return False

        self._state_device._put(1 if self._engine.is_active() else 0)
        if self._engine.has_result():                      
            self._result_device._put(self._engine.read_result())
            self._state_device._put(0)
            self._listen_state_device._put_empty()         

        if self._ready == False:
            self._ready = True
            Runner.register_checked()
        self._notify_sensory_device_data_changed()
        return True

    def _encode(self):
        # motor(command) → engine
        with self._thread_lock:
            if self._listen_written:
                if self._listen == 1:
                    self._engine._lang = self._lang
                    if not self._engine.is_active():       
                        self._engine.start()
                else:
                    self._engine.abort()
                self._listen_written = False
