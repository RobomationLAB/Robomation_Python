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

import threading
import time

import cv2

from robomation.core.runner import Runner

_views = {}            # key -> provider() → (title, frame) | None
_lock = threading.Lock()
_registered = False
_last_show = 0.0
_FPS_INTERVAL = 1.0 / 30
_created = set()       # namedWindow 로 이미 생성한 창 제목 (중복 생성 방지)


def add_view(key, provider):
    """표시할 뷰 등록. provider() 는 (title, frame) 또는 None 을 반환."""
    global _registered
    with _lock:
        _views[key] = provider
        if not _registered:
            Runner.register_wait_callback(_pump)
            _registered = True


def remove_view(key):
    with _lock:
        existed = _views.pop(key, None) is not None
    _created.discard(str(key))
    if existed and threading.current_thread() is threading.main_thread():
        try:
            cv2.destroyWindow(str(key))
            cv2.waitKey(1)
        except Exception:
            pass


def _pump():
    # 메인 스레드에서만 GUI 호출 (다른 스레드에서 wait 가 불려도 표시는 건너뜀)
    if threading.current_thread() is not threading.main_thread():
        return
    global _last_show
    now = time.time()
    if now - _last_show < _FPS_INTERVAL:
        return
    _last_show = now
    with _lock:
        items = list(_views.items())
    if not items:
        return
    for key, provider in items:
        try:
            res = provider()
            if res:
                title, frame = res
                if frame is not None and frame.shape[0] > 0 and frame.shape[1] > 0:
                    if title not in _created:
                        # WINDOW_AUTOSIZE: 창이 항상 영상 크기에 고정
                        cv2.namedWindow(title, cv2.WINDOW_AUTOSIZE)
                        try:
                            cv2.setWindowProperty(title, cv2.WND_PROP_AUTOSIZE, 1)
                        except Exception:
                            pass
                        _created.add(title)
                    cv2.imshow(title, frame)
        except Exception:
            pass
    cv2.waitKey(1)   # GUI 이벤트 처리 (필수)
