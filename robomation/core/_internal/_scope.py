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

"""Native (CPython) oscilloscope for Utils.scope.

Scope 표시 규칙:
- 스코프 창 개수 = 서로 다른 name(label) 개수. 같은 name → 같은 창(세로 스택).
- 창 안의 선(line) = (min, max, color) 튜플 단위. 같은 튜플이면 같은 선에 누적.
- 창의 Y축 범위 = [min(모든 line.min), max(모든 line.max)] 로 재조정.
- X축 = 최근 100 포인트(5초 / 50ms) 슬라이딩 윈도.

스레딩 모델 (중요):
- Utils.scope() 는 보통 사용자 loop() = Utils.parallel 워커 스레드에서 호출된다.
  하지만 matplotlib(및 모든 GUI 툴킷) 의 창 생성/갱신은 메인 스레드에서만 안전하다
  (특히 macOS 의 Cocoa 백엔드는 워커 스레드에서 figure 를 만들면 예외).
    · scope()  → 워커 스레드에서 '데이터 기록만' 한다(GUI 호출 없음).
    · _pump()  → Runner.register_wait_callback 으로 등록되어, 메인 스레드의
                 Runner.wait()/wait_forever() 루프가 매 반복 호출한다. 
                 실제 figure 생성/redraw 는 전부 여기(메인 스레드)에서 일어난다.
- matplotlib 가 없거나 GUI 백엔드를 못 열면 콘솔 출력('name: signal')으로 폴백한다.
"""

import threading
import time

from robomation.core.runner import Runner

# X축에 한 번에 보여줄 포인트 수(= 창 폭). scope() 호출마다 점이 1개 쌓이므로,
# 값이 클수록 파형이 화면을 가로지르는 데 오래 걸린다(= 더 느리게 흐름).
# 예: loop 가 10ms 주기면 300 포인트 ≈ 3초 창. 더 느리게 하려면 키우고, 빠르게 하려면 줄인다.
_MAX_POINTS = 300
_DRAW_INTERVAL = 0.05  # 50ms (화면 갱신 주기. 작을수록 부드럽지만 CPU 더 씀)

# name -> {'order': [key...], 'lines': {key: {'min','max','color','signals':[...]}}}
# key = (min, max, color)
_scopes = {}
_name_order = []

# scope()(워커 스레드) 의 기록과 _pump()(메인 스레드) 의 읽기를 보호.
_lock = threading.Lock()

_registered = False
_last_draw = 0.0

# matplotlib lazy 초기화 상태 (전부 메인 스레드에서만 건드린다)
_mpl_ready = None      # None=미시도, True=사용가능, False=폴백
_plt = None
_fig = None
_axes = {}             # name -> Axes
_line_artists = {}     # (name, key) -> Line2D
_built_names = []      # 현재 figure 에 그려진 name 순서

# 콘솔 폴백 throttle
_fallback_last = {}


def _mpl_color(color):
    """Utils.color()/color_rgb() 는 0~255 RGB 리스트를 돌려주는데,
    matplotlib 의 color 인자는 0~1 실수(또는 색 이름)를 요구한다.
    0~255 리스트/튜플이면 0~1 로 정규화하고, 그 외(색 이름 문자열 등)는 그대로 둔다."""
    if isinstance(color, (list, tuple)) and len(color) in (3, 4):
        vals = list(color)
        if any(isinstance(v, (int, float)) and v > 1 for v in vals):
            return [max(0.0, min(v / 255.0, 1.0)) for v in vals]
    return color


def scope(name, min_val, max_val, color, signal):
    """신호 한 점을 기록한다(워커 스레드 안전). 실제 그리기는 메인 스레드 _pump() 에서."""
    _record(name, min_val, max_val, color, signal)
    _ensure_registered()


def _ensure_registered():
    """메인 스레드 wait 루프에 _pump 를 한 번만 건다."""
    global _registered
    if _registered:
        return
    with _lock:
        if _registered:
            return
        Runner.register_wait_callback(_pump)
        _registered = True


def _record(name, min_val, max_val, color, signal):
    with _lock:
        scope = _scopes.get(name)
        if scope is None:
            scope = {'order': [], 'lines': {}}
            _scopes[name] = scope
            _name_order.append(name)

        # color 가 list 면 dict 키로 못 쓰므로(unhashable) 튜플로 정규화해 키를 만든다.
        color_key = tuple(color) if isinstance(color, list) else color
        key = (min_val, max_val, color_key)
        line = scope['lines'].get(key)
        if line is None:
            line = {'min': min_val, 'max': max_val, 'color': color, 'signals': []}
            scope['lines'][key] = line
            scope['order'].append(key)

        sig = line['signals']
        sig.append(signal)
        # 메모리 보호: 윈도보다 충분히 큰 선에서 오래된 값 잘라냄.
        if len(sig) > _MAX_POINTS * 4:
            del sig[:-_MAX_POINTS * 2]


def _pump():
    """메인 스레드(Runner.wait 루프)에서만 GUI 를 그린다. 워커 스레드에서 불리면 건너뜀."""
    if threading.current_thread() is not threading.main_thread():
        return

    global _last_draw
    now = time.monotonic()
    if (now - _last_draw) < _DRAW_INTERVAL:
        return
    _last_draw = now

    if _ensure_mpl():
        try:
            _redraw()
        except Exception as e:
            print(f"[scope] draw error: {e}")
    else:
        _fallback_print()


def _ensure_mpl():
    """matplotlib 를 메인 스레드에서 lazy 초기화. 실패하면 콘솔 폴백."""
    global _mpl_ready, _plt, _fig
    if _mpl_ready is not None:
        return _mpl_ready
    try:
        import matplotlib
        import matplotlib.pyplot as plt
        _configure_korean_font(matplotlib)
        plt.ion()
        _plt = plt
        _fig = plt.figure(num="Digital Scope")
        _mpl_ready = True
    except Exception as e:
        print(f"[scope] matplotlib unavailable, falling back to console: {e}")
        _mpl_ready = False
    return _mpl_ready


def _configure_korean_font(matplotlib):
    """스코프 라벨(name)에 한글이 들어가도 깨지지 않도록 한글 지원 폰트를 지정한다.
    기본 폰트(DejaVu Sans)는 한글 글리프가 없어 'Glyph ... missing' 경고가 난다.
    설치된 폰트 중 플랫폼별 후보를 찾아 첫 번째 것을 쓰고, 없으면 조용히 기본값 유지."""
    import platform
    from matplotlib import font_manager

    system = platform.system()
    if system == 'Darwin':
        candidates = ['Apple SD Gothic Neo', 'AppleGothic', 'Noto Sans CJK KR', 'NanumGothic']
    elif system == 'Windows':
        candidates = ['Malgun Gothic', 'Gulim', 'Noto Sans CJK KR', 'NanumGothic']
    else:
        candidates = ['NanumGothic', 'Noto Sans CJK KR', 'Noto Sans KR', 'UnDotum']

    available = {f.name for f in font_manager.fontManager.ttflist}
    chosen = next((c for c in candidates if c in available), None)
    if chosen:
        matplotlib.rcParams['font.family'] = chosen
    # 한글 폰트 사용 시 마이너스 기호(−)가 네모로 깨지는 것 방지.
    matplotlib.rcParams['axes.unicode_minus'] = False


def _rebuild_axes(names):
    """name 집합이 바뀌면 subplot 격자를 다시 만든다."""
    global _built_names, _axes, _line_artists
    _fig.clf()
    _axes = {}
    _line_artists = {}
    n = len(names)
    for i, name in enumerate(names):
        ax = _fig.add_subplot(n, 1, i + 1)
        ax.set_title(f"Digital Scope[{i + 1}] : {name}", fontsize=9, loc='left')
        ax.grid(True, color='gray', linewidth=0.3, alpha=0.5)
        ax.set_xlim(0, _MAX_POINTS)
        _axes[name] = ax
    _built_names = list(names)
    _fig.tight_layout()


def _snapshot():
    """lock 안에서 그리기에 필요한 데이터를 얕게 복사한다(워커 스레드 기록과의 경합 방지)."""
    with _lock:
        names = list(_name_order)
        data = {}
        for name in names:
            scope = _scopes[name]
            lines = []
            for key in scope['order']:
                line = scope['lines'][key]
                lines.append((
                    key, line['min'], line['max'], line['color'],
                    line['signals'][-_MAX_POINTS:],
                ))
            data[name] = lines
        return names, data


def _redraw():
    names, data = _snapshot()
    if not names:
        return

    # 새 name 이 생겼으면 격자 재구성.
    if _built_names != names:
        _rebuild_axes(names)

    for name in names:
        ax = _axes[name]
        lines = data[name]
        if not lines:
            continue

        # Y축 범위 재조정: 모든 선의 min/max 중 최소/최대.
        y_min = min(l[1] for l in lines)
        y_max = max(l[2] for l in lines)
        if y_min == y_max:
            y_max = y_min + 1
        ax.set_ylim(y_min, y_max)

        for key, _mn, _mx, color, window in lines:
            xs = range(len(window))
            artist = _line_artists.get((name, key))
            if artist is None:
                (artist,) = ax.plot(
                    list(xs), list(window),
                    color=_mpl_color(color), linewidth=2.0, alpha=0.8,
                )
                _line_artists[(name, key)] = artist
            else:
                artist.set_data(list(xs), list(window))

    _fig.canvas.draw_idle()
    _fig.canvas.flush_events()


def _fallback_print():
    # matplotlib 가 없을 때: 각 name 의 최신 신호를 throttle 해서 콘솔에 출력.
    names, data = _snapshot()
    now = time.monotonic()
    for name in names:
        lines = data[name]
        if not lines:
            continue
        last = _fallback_last.get(name, 0.0)
        if now - last >= _DRAW_INTERVAL:
            _fallback_last[name] = now
            signal = lines[-1][4][-1] if lines[-1][4] else None
            print(f"[scope:{name}] {signal}")
