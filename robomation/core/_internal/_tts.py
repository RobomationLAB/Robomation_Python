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

"""Native (CPython) TTS backend for Utils.set_tts / Utils.speak.

VSCode(일반 CPython) 환경에는 Web Speech API 가 없으므로 OS 네이티브 TTS 로 기능을 구현한다.

- macOS  : `say` 명령
- Windows: PowerShell `System.Speech.Synthesis.SpeechSynthesizer`
- Linux  : `spd-say` 또는 `espeak`

웹의 speechSynthesis.speak 와 동일하게 비차단(non-blocking) 으로 동작한다.
음성 이름/언어가 OS 보이스와 매칭되지 않으면 기본 보이스로 폴백한다.
"""

import platform
import subprocess

_TTS_OPTION = {'lang': None, 'name': None}

def set_tts(lang, name):
    """선택된 TTS 언어/음성을 기억한다."""
    _TTS_OPTION['lang'] = lang
    _TTS_OPTION['name'] = name


def speak(text):
    """선택된 음성으로 텍스트를 읽는다. 비차단."""
    if text is None:
        return
    text = str(text)
    system = platform.system()
    try:
        if system == 'Darwin':
            _speak_macos(text)
        elif system == 'Windows':
            _speak_windows(text)
        else:
            _speak_linux(text)
    except Exception as e:
        # TTS 미설치/실패는 치명적이지 않다. 경고만 남기고 계속 진행.
        print(f"[speak] TTS unavailable: {e}")


def _spawn(args):
    """자식 프로세스를 백그라운드로 띄우고 즉시 리턴(비차단)."""
    subprocess.Popen(
        args,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def _speak_macos(text):
    name = _TTS_OPTION.get('name')
    args = ['say']
    if name:
        args += ['-v', name]   # 매칭 안 되는 voice 이름이면 say 가 에러 → 폴백 처리
    args.append(text)
    try:
        _spawn(args)
    except Exception:
        _spawn(['say', text])   # voice 폴백


def _speak_linux(text):
    name = _TTS_OPTION.get('name')
    lang = _TTS_OPTION.get('lang')
    # spd-say 우선, 없으면 espeak.
    if _which('spd-say'):
        args = ['spd-say']
        if lang:
            args += ['-l', lang.split('-')[0]]
        args.append(text)
        _spawn(args)
    elif _which('espeak'):
        args = ['espeak']
        if lang:
            args += ['-v', lang.split('-')[0]]
        args.append(text)
        _spawn(args)
    else:
        print("[speak] no TTS engine found (install spd-say or espeak)")


def _speak_windows(text):
    name = _TTS_OPTION.get('name')
    safe = text.replace("'", "''")
    select = ""
    if name:
        safe_name = name.replace("'", "''")
        select = f"try {{ $s.SelectVoice('{safe_name}') }} catch {{}};"
    script = (
        "Add-Type -AssemblyName System.Speech;"
        "$s = New-Object System.Speech.Synthesis.SpeechSynthesizer;"
        f"{select}"
        f"$s.Speak('{safe}')"
    )
    _spawn(['powershell', '-NoProfile', '-Command', script])


def _which(cmd):
    import shutil
    return shutil.which(cmd) is not None
