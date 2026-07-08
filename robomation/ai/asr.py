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

# AI 확장 모듈 - 음성 인식(ASR)

import time
from typing import Literal, get_args

from robomation.core.error import _err
from robomation.core.runner import Runner
from robomation.core.model import Robot


# ── Module-level Literal aliases (for IDE auto-complete) ─────────────────────
_Lang = Literal['ko-KR', 'en-US']


class ASR(Robot):
    """Automatic Speech Recognition ai extension."""
    ID = "kr.robomation.virtual.ai.asr"
    _instances = {}

    # ── Device IDs (AI 확장모듈) ─────────────────────────────────────────
    # 형식: 0xA00xxxxx  (0xA=가상 AI 표식, product_id = 0)
    # Effectors 
    LANG            = 0xA0000000
    # Commands
    LISTEN          = 0xA0000100
    # Sensors 
    RESULT          = 0xA0000200
    STATE           = 0xA0000201
    # Events
    LISTEN_STATE    = 0xA0000300

    # ── Valid values for enum parameters ─────────────────────────────────────
    _VALID_LANG = get_args(_Lang)

    # ── Robot lifecycle ──────────────────────────────────────────────────────
    def __init__(self, index=0):
        if isinstance(index, str):
            index = 0
        if index in ASR._instances:
            robot = ASR._instances[index]
            if robot: robot.dispose()
        ASR._instances[index] = self
        super(ASR, self).__init__(ASR.ID, "ASR", index)
        self._init()

    def dispose(self):
        ASR._instances[self.get_index()] = None
        self._roboid._dispose()
        Runner.unregister_robot(self)

    def reset(self):
        self._roboid._reset()

    def _init(self):
        from robomation.ai.asr_roboid import ASRRoboid
        self._roboid = ASRRoboid(self.get_index())
        self._add_roboid(self._roboid)
        Runner.register_robot(self)
        Runner.start()
        self._roboid._init()

    def find_device_by_id(self, device_id):
        return self._roboid.find_device_by_id(device_id)

    def _request_motoring_data(self):
        self._roboid._request_motoring_data()

    def _update_sensory_device_state(self):
        self._roboid._update_sensory_device_state()

    def _update_motoring_device_state(self):
        self._roboid._update_motoring_device_state()

    def _notify_sensory_device_data_changed(self):
        self._roboid._notify_sensory_device_data_changed()

    def _notify_motoring_device_data_changed(self):
        self._roboid._notify_motoring_device_data_changed()

    # ── Public API ────────────────────────────────────
    def lang(self, unit: _Lang):
        if unit not in ASR._VALID_LANG:
            return _err(ASR, 'lang', 'unit', unit, ASR._VALID_LANG)
        self.write(ASR.LANG, unit)

    def start(self):
        self.write(ASR.LISTEN, 1)
        
        timeout = time.time() + 2
        while self.is_active() == False and time.time() < timeout:
            time.sleep(0.01)

    def stop(self):
        self.write(ASR.LISTEN, 0)
        
        timeout = time.time() + 2
        while self.is_active() and time.time() < timeout:
            time.sleep(0.01)

    def result(self) -> str:
        return self.read(ASR.RESULT)

    def is_active(self) -> bool:
        return self.read(ASR.STATE) == 1
