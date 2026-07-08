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

"""Native (CPython) sound playback for Utils.play_sound.

재생은 OS 네이티브 플레이어로 비차단(non-blocking) 처리.
- macOS: afplay 
- Windows: winsound|PowerShell MediaPlayer 
- Linux: paplay|aplay

volume / repeat 는 플랫폼별 best-effort.
"""

import os
import platform
import subprocess
import threading
import hashlib
import tempfile
import urllib.request
import urllib.parse

S3_BASE = "https://robomation-lab-test.s3.ap-northeast-2.amazonaws.com"

_CACHE_DIR = os.path.join(tempfile.gettempdir(), "robomation_audio_cache")


def play_sound(file, volume=100, repeat=False):
    """사운드 클립을 S3 에서 받아 재생한다."""
    if not file:
        return
    head = str(file).split('/')[0]
    if head in ('record', 'local'):
        print(f"[play_sound] '{head}/...' clips are not supported in the package "
              f"(browser-only IndexedDB). skipped: {file}")
        return
    try:
        path = _ensure_cached(file)
    except Exception as e:
        print(f"[play_sound] download failed for {file!r}: {e}")
        return

    # 재생 자체를 백그라운드 스레드로 → 비차단.
    threading.Thread(
        target=_play_file, args=(path, volume, repeat), daemon=True
    ).start()


def _ensure_cached(file):
    """S3 의 {file}.wav 를 받아 로컬 캐시 경로를 반환. 이미 있으면 재다운로드 생략."""
    os.makedirs(_CACHE_DIR, exist_ok=True)
    key = hashlib.md5(file.encode('utf-8')).hexdigest()
    path = os.path.join(_CACHE_DIR, key + ".wav")
    if os.path.exists(path) and os.path.getsize(path) > 0:
        return path

    url = f"{S3_BASE}/Audio/{urllib.parse.quote(file)}.wav"
    tmp = path + ".part"
    urllib.request.urlretrieve(url, tmp)
    os.replace(tmp, path)
    return path


def _play_file(path, volume, repeat):
    system = platform.system()
    try:
        if system == 'Darwin':
            _play_macos(path, volume, repeat)
        elif system == 'Windows':
            _play_windows(path, volume, repeat)
        else:
            _play_linux(path, volume, repeat)
    except Exception as e:
        print(f"[play_sound] playback failed: {e}")


def _play_macos(path, volume, repeat):
    # afplay -v 는 0.0~1.0+ 범위의 볼륨.
    vol = max(0.0, volume / 100.0)
    args = ['afplay', '-v', str(vol), path]
    if repeat:
        # afplay 는 loop 옵션이 없으므로 종료 시 재실행 반복.
        while True:
            subprocess.run(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    else:
        subprocess.run(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def _play_linux(path, volume, repeat):
    import shutil
    # paplay 가 볼륨 지원(0~65536). 없으면 aplay(볼륨 미지원).
    if shutil.which('paplay'):
        vol = int(max(0.0, min(volume / 100.0, 1.0)) * 65536)
        args = ['paplay', '--volume', str(vol), path]
    elif shutil.which('aplay'):
        args = ['aplay', '-q', path]
    else:
        print("[play_sound] no audio player found (install pulseaudio or alsa-utils)")
        return
    if repeat:
        while True:
            subprocess.run(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    else:
        subprocess.run(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def _play_windows(path, volume, repeat):
    # PowerShell MediaPlayer 가 볼륨/루프를 지원. (winsound 는 볼륨 미지원)
    vol = max(0.0, min(volume / 100.0, 1.0))
    safe = path.replace("'", "''")
    loop = "$p.add_MediaEnded({ $p.Position = [TimeSpan]::Zero; $p.Play() });" if repeat else ""
    # 비동기 재생이 끝나기 전에 프로세스가 죽지 않도록 대기.
    wait = ("while($true){ Start-Sleep -Seconds 3600 }" if repeat else
            "Start-Sleep -Seconds ([int]([math]::Ceiling("
            "$p.NaturalDuration.TimeSpan.TotalSeconds)) + 1)")
    script = (
        "Add-Type -AssemblyName presentationCore;"
        "$p = New-Object System.Windows.Media.MediaPlayer;"
        f"$p.Open([uri]'{safe}');"
        f"$p.Volume = {vol};"
        f"{loop}"
        "$p.Play();"
        f"{wait}"
    )
    subprocess.run(
        ['powershell', '-NoProfile', '-Command', script],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
