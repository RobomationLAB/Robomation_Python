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
import subprocess

_opened_panes = set()   # 프로세스당 같은 창 중복 오픈 방지


def _open_settings_pane(pane, once=True):
    """macOS 시스템 설정의 개인정보 보호 창을 연다. (darwin 외에는 무동작)"""
    if sys.platform != 'darwin':
        return False
    if once and pane in _opened_panes:
        return True
    _opened_panes.add(pane)
    try:
        subprocess.run(
            ["open", f"x-apple.systempreferences:com.apple.preference.security?{pane}"],
            check=False,
        )
        return True
    except Exception:
        return False


def open_microphone_settings(once=True):
    """시스템 설정 → 개인정보 보호 및 보안 → 마이크 창 열기."""
    return _open_settings_pane("Privacy_Microphone", once)


def open_camera_settings(once=True):
    """시스템 설정 → 개인정보 보호 및 보안 → 카메라 창 열기. (카메라 모듈용)"""
    return _open_settings_pane("Privacy_Camera", once)
