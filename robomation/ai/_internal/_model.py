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

import os
import sys
import threading
import urllib.request
from contextlib import contextmanager
from pathlib import Path


# fd 2(stderr) 억제 상태 — 여러 스레드/엔진(연속 추론)이 겹쳐 들어와도 fd 가 깨지지
# 않도록 lock + 깊이(refcount)로 관리. 깊이>0 동안만 fd 2 가 devnull 로 향한다.
_supp_lock = threading.Lock()
_supp_depth = 0
_supp_saved = None
_supp_devnull = None


@contextmanager
def suppress_native_stderr():
    """MediaPipe/TFLite 네이티브 로그(absl/glog) 억제용.

    이 로그들은 C++ 에서 stderr(fd 2)에 직접 쓰므로 파이썬 logging/환경변수
    (GLOG_minloglevel 등)로는 막히지 않는다. 잠깐 fd 2 를 devnull 로 돌려 숨긴다.
    모델 생성/종료뿐 아니라 일부 모델은 detect 때도 경고를 내므로 그 호출도 감쌀 수 있다.
    refcount 방식이라 동시/중첩 호출에도 안전하다(마지막 컨텍스트가 빠질 때 복구).
    """
    global _supp_depth, _supp_saved, _supp_devnull
    with _supp_lock:
        if _supp_depth == 0:
            sys.stderr.flush()
            _supp_saved = os.dup(2)
            _supp_devnull = os.open(os.devnull, os.O_WRONLY)
            os.dup2(_supp_devnull, 2)
        _supp_depth += 1
    try:
        yield
    finally:
        with _supp_lock:
            _supp_depth -= 1
            if _supp_depth == 0:
                sys.stderr.flush()
                os.dup2(_supp_saved, 2)
                os.close(_supp_devnull)
                os.close(_supp_saved)
                _supp_saved = None
                _supp_devnull = None

_BUNDLED_DIR = Path(__file__).resolve().parent.parent / '_model'
_CACHE_DIR = Path.home() / '.robomation' / 'models'


def ensure_model(filename, url, label):
    """모델 파일의 로컬 경로를 반환. 캐시→다운로드 순. 모두 실패 시 None."""
    # 1) 사용자 캐시
    cached = _CACHE_DIR / filename
    if cached.exists():
        return str(cached)
    # 2) 다운로드 (온라인일 때만)
    try:
        _CACHE_DIR.mkdir(parents=True, exist_ok=True)
        print(f"[{label}] 모델 다운로드 중...",
              file=sys.stderr)
        urllib.request.urlretrieve(url, cached)
        return str(cached)
    except Exception as e:
        print(f"[{label}] 모델을 찾을 수 없습니다(번들/캐시 없음, 다운로드 실패): {e}",
              file=sys.stderr)
        return None
