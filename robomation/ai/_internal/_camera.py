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

# AI 확장 모듈 - 카메라(Camera)

import sys
import time
import threading
import subprocess

import cv2

from robomation.core.runner import Runner
from robomation.ai._internal import _display


# ── 카메라 이름 열거 + 식별자 해석 (cross-OS) ────────────────────

def list_cameras():
    """OS별 사용 가능한 카메라 목록 [(index, name), ...].
    같은 label 이 둘 이상이면 deviceID(index) 순으로 ' #1', ' #2' 접미사를 붙여 고유화한다.
    (예: 동일 모델 카메라 2대 → 'HAMSTER_AI_UVC (303a:8003) #1' / '... #2')
    """
    try:
        if sys.platform == 'darwin':
            cams = _list_macos()
        elif sys.platform.startswith('win'):
            cams = _list_windows()
        else:
            cams = _list_linux()
    except Exception:
        return []
    return _labeling(cams)


def _labeling(cams):
    # 같은 이름이 여러 개면 deviceID(index) 오름차순으로 #1, #2 ... 를 붙여 고유 label 생성.
    # 이름이 하나뿐이면 접미사 없이 그대로 둔다.
    counts = {}
    for _, name in cams:
        counts[name] = counts.get(name, 0) + 1
    seen = {}
    result = []
    for idx, name in sorted(cams, key=lambda c: c[0]):
        if counts[name] > 1:
            seen[name] = seen.get(name, 0) + 1
            name = f"{name} #{seen[name]}"
        result.append((idx, name))
    return result


def resolve(x):
    """device() 인자 → 카메라 인덱스(int).
    - int(또는 숫자문자열) : 카메라 리스트에서 index 가 그 값인 카메라
    - str(이름)            : 그 이름의 카메라 (부분/양방향 매칭)
    """
    if isinstance(x, bool):
        return 0
    cams = list_cameras()
    # 숫자 → 인덱스: 리스트에서 index 가 그 값인 카메라
    if isinstance(x, int) or (isinstance(x, str) and x.strip().isdigit()):
        idx = int(x)
        for i, _ in cams:
            if i == idx:
                return idx
        if not cams:                     # 카메라 열거가 안 되는 환경 → 인덱스 그대로 시도
            return idx
        print(f"[Camera] 인덱스 {idx} 카메라가 목록에 없습니다. "
              f"인덱스(0, 1, ...) 또는 카메라 이름으로 지정하세요.", file=sys.stderr)
        return idx                       # cv2 인덱스로는 유효할 수 있으니 그대로 시도
    # 이름 → 카메라 찾기
    if isinstance(x, str):
        s = x.strip()
        for i, name in cams:             # 정확히 일치 우선
            if s == name:
                return i
        # 부분 일치(양방향): 브라우저 'FaceTime HD 카메라 (3A71:F4B5)' 처럼 접미사가 붙어도 매칭
        sl = s.lower()
        for i, name in cams:
            nl = name.lower()
            if nl in sl or sl in nl:
                return i
        # 이름으로 지정했는데 못 찾으면 다른 카메라로 폴백하지 않고 연결하지 않음(-1).
        print(f"[Camera] 선택한 카메라 '{s}' 를 사용할 수 없습니다. (연결 안 함)\n"
              f"      인덱스(0, 1, ...) 또는 정확한 카메라 이름으로 지정하세요.", file=sys.stderr)
        return -1
    return -1


def _list_macos():
    # system_profiler SPCameraDataType 출력 파싱.
    #   카메라 이름 라인: 들여쓰기 후 ':' 로 끝남 ('Camera:' 헤더 제외)
    #   Unique ID 라인  : 'Unique ID: 47B4...-AE273A71F4B5'
    # 브라우저(Chrome)는 라벨에 Unique ID 끝 8자리를 '(3A71:F4B5)' 로 붙인다 → 동일하게 맞춤.
    out = subprocess.run(
        ['system_profiler', 'SPCameraDataType'],
        capture_output=True, text=True, timeout=5,
    ).stdout
    cams = []  # [[name, uid], ...]
    for line in out.splitlines():
        s = line.strip()
        if not s or s == 'Camera:':
            continue
        if s.startswith('Unique ID:'):
            if cams:
                cams[-1][1] = s.split(':', 1)[1].strip()
        elif s.endswith(':'):
            cams.append([s[:-1].strip(), None])
    result = []
    for idx, (name, uid) in enumerate(cams):
        if uid:
            clean = uid.replace('-', '')
            if len(clean) >= 8:
                name = f"{name} ({clean[-8:-4]}:{clean[-4:]})"   # 브라우저 포맷
        result.append((idx, name))
    return result


def _list_windows():
    # DirectShow 장치 이름 (pygrabber 필요). 미설치 시 빈 목록.
    try:
        from pygrabber.dshow_graph import FilterGraph
        return list(enumerate(FilterGraph().get_input_devices()))
    except Exception:
        return []


def _list_linux():
    # /sys/class/video4linux/video*/name 에서 이름 추출.
    import glob, os, re
    result = []
    for path in sorted(glob.glob('/sys/class/video4linux/video*')):
        m = re.search(r'video(\d+)$', path)
        if not m:
            continue
        idx = int(m.group(1))
        try:
            with open(os.path.join(path, 'name')) as f:
                name = f.read().strip()
        except Exception:
            name = f'video{idx}'
        result.append((idx, name))
    return result


# 모든 카메라 프레임을 동일한 기준 폭으로 정규화(가로세로비 유지)한다.
# → 카메라 해상도와 무관하게 표시 크기·오버레이 크기·좌표(face.area 등)가 일관됨.
_FRAME_WIDTH = 800

class _CaptureSource:
    """실제 카메라 캡처(cv2.VideoCapture + 백그라운드 스레드). 프레임은 기준 폭으로 정규화."""
    _instances = {}
    _reg_lock = threading.Lock()

    @staticmethod
    def acquire(index):
        with _CaptureSource._reg_lock:
            src = _CaptureSource._instances.get(index)
            if src is None:
                src = _CaptureSource(index)
                _CaptureSource._instances[index] = src
            src._refcount += 1
            return src

    def __init__(self, index):
        self._index = index
        self._refcount = 0
        self._frame = None
        self.__frame_width = _FRAME_WIDTH   # 정규화 기준 폭 (생성 시 고정)
        self._running = True
        self._cap = cv2.VideoCapture(index)
        thread = threading.Thread(target=self._run)
        thread.daemon = True
        thread.start()

    @property
    def _frame_width(self):
        # 읽기 전용 — 런타임에 프레임 크기를 변경할 수 없다(setter 없음).
        # 크기를 바꾸려면 모듈 상수 _FRAME_WIDTH 만 수정한다. _normalize 가 폭만 받고
        # 높이를 비례 계산하므로 어떤 값으로 바꿔도 가로세로비는 항상 유지된다.
        return self.__frame_width

    def _run(self):
        while self._running:
            cap = self._cap
            if cap is not None and cap.isOpened():
                ok, frame = cap.read()
                if ok:
                    self._frame = self._normalize(frame)
            time.sleep(0.01)

    def _normalize(self, frame):
        # 기준 폭(self._frame_width)으로 리사이즈(가로세로비 유지). 폭이 같으면 그대로.
        fw = self._frame_width
        h, w = frame.shape[:2]
        if w > 0 and w != fw:
            nh = max(1, round(h * fw / w))
            frame = cv2.resize(frame, (fw, nh))
        return frame

    def read(self):
        return self._frame

    def is_opened(self):
        return self._cap is not None and self._cap.isOpened()

    def wait_first_frame(self, timeout=0.5):
        deadline = time.time() + timeout
        while self._frame is None and time.time() < deadline:
            time.sleep(0.01)
        return self._frame

    def release(self):
        with _CaptureSource._reg_lock:
            self._refcount -= 1
            if self._refcount > 0:
                return
            self._running = False
            _CaptureSource._instances.pop(self._index, None)
            cap = self._cap
            self._cap = None
        if cap is not None:
            cap.release()


class Camera:
    """Camera ai extension"""
    ID = "kr.robomation.virtual.ai.camera"

    def __init__(self, device=None):
        self._index = 0 if device is None else resolve(device)
        self._source = _CaptureSource.acquire(self._index) if self._index >= 0 else None
        self._name = f"Camera {self._index}"
        self._showing = False
        self._closed = False
        Runner.register_component(self)
        self._check_permission()

    def device(self, x):
        """카메라 재지정. x = 이름(str) / 인덱스(int) / Camera 객체."""
        if isinstance(x, Camera):
            index = x._index
        else:
            index = resolve(x)
        if index == self._index:
            return
        was_showing = self._showing
        if was_showing:
            self.display(False)
        if self._source is not None:
            self._source.release()
        self._index = index
        self._source = _CaptureSource.acquire(index) if index >= 0 else None
        self._name = f"Camera {index}"
        self._check_permission()
        if was_showing:
            self.display(True)

    def read(self):
        """최신 프레임(numpy BGR) 또는 None."""
        return self._source.read() if self._source is not None else None

    def _view(self):
        if self._source is None:
            return None
        frame = self._source.read()
        if frame is None:
            return None
        # 정규화 프레임 그대로 반환. 창 크기에 맞춘 늘이기(왜곡 포함)는
        # WINDOW_NORMAL 창의 imshow 가 처리한다(드래그하면 영상이 창 비율대로 늘어남).
        return (self._name, frame)

    def display(self, on=True):
        """카메라 화면 창 표시/숨김. 실제 표시는 메인 스레드 Runner.wait() 펌프에서."""
        if on:
            self._showing = True
            _display.add_view(self._name, self._view)
        else:
            self._showing = False
            _display.remove_view(self._name)

    def _check_permission(self):
        if self._source is None:        # 연결된 카메라 없음(-1) → 권한 검사 불필요
            return
        # macOS: 권한 미허용 시 캡처가 안 열리거나 검은 프레임만 온다.
        # 단 정상 허용돼도 카메라 워밍업에 시간이 걸려 첫 프레임이 잠시 None 일 수 있으므로,
        # 구성을 막지 않고 백그라운드에서 충분히 기다린 뒤 판단한다.
        if sys.platform != 'darwin':
            return
        thread = threading.Thread(target=self._permission_watch, args=(self._source,))
        thread.daemon = True
        thread.start()

    def _permission_watch(self, source):
        deadline = time.time() + 3.0
        while time.time() < deadline:
            if self._closed or source is not self._source:
                return                      # dispose/재지정됨 → 무시
            frame = source.read()
            if frame is not None and frame.max() > 0:
                return                      # 정상 (비검은 프레임 도착)
            time.sleep(0.05)
        if self._closed or source is not self._source:
            return
        from robomation.core import permission
        print(
            "[Camera] 카메라 입력이 감지되지 않습니다.\n"
            "      macOS 라면 Python 을 실행한 앱(VSCode/터미널 등)의 카메라 권한이 꺼져 있을 수 있습니다.\n"
            "      시스템 설정 → 개인정보 보호 및 보안 → 카메라 에서 해당 앱을 허용한 뒤,\n"
            "      그 앱을 완전히 종료했다가 다시 실행하세요.",
            file=sys.stderr,
        )
        permission.open_camera_settings()

    def dispose(self):
        self._closed = True
        self.display(False)
        if self._source is not None:
            self._source.release()
            self._source = None
        Runner.unregister_component(self)
