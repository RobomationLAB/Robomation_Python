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

"""Native (CPython) log output for Utils.log.

VSCode(일반 CPython) 환경에선 stdout 에 찍지만, 
터미널(TTY)이면 캐리지 리턴(\r)으로 직전 줄을 덮어써서 동일하게 '(개수) 내용' 으로 합친다.

- TTY: 직전과 같은 값이면 그 줄을 '(2) 내용', '(3) 내용' … 으로 갱신. 다른 값이면 새 줄.
- 비-TTY(파일 리다이렉트 등): \r 이 깨지므로 매번 새 줄로 폴백.
- 중간에 다른 출력(scope 폴백, speak 경고 등)이 끼면 카운트는 리셋된다(열린 줄이 끊김).
"""

import sys

_last_line = None   # 직전에 출력한 줄(내용)
_count = 0          # 직전 줄이 연속으로 몇 번 나왔는지


def log(data, tag=None, unit=None):
    line = _format(data, tag, unit)
    _emit(line)


def _format(data, tag, unit):
    prefix = f"[{tag}] " if tag else ""
    if isinstance(data, (list, tuple)):
        if unit:
            items = [f"{d}{unit}" for d in data]
        else:
            items = [str(d) for d in data]
        body = "[" + ",".join(items) + "]"
    else:
        body = f"{data}{unit}" if unit else f"{data}"
    return prefix + body


def _emit(line):
    global _last_line, _count

    if not sys.stdout.isatty():
        # 비대화형: \r 덮어쓰기가 불가능하므로 매번 새 줄.
        print(line)
        _last_line, _count = line, 1
        return

    if line == _last_line:
        # 직전과 같은 값 → 그 줄을 '(개수) 내용' 으로 덮어쓴다.
        _count += 1
        sys.stdout.write(f"\r({_count}) {line}\033[K")  # \033[K = 줄 끝까지 지우기
    else:
        # 다른 값 → 열려 있던 줄을 마감(\n)하고 새 줄 시작.
        if _last_line is not None:
            sys.stdout.write("\n")
        sys.stdout.write(line)
        _last_line, _count = line, 1
    sys.stdout.flush()
