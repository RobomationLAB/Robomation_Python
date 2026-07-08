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

from typing import Union
from robomation.core.error import _err


class PID10:
    """Ultrasonic distance sensor (PID slot module on CheeseStick)."""
    ID = "kr.robomation.physical.module.cheesestick.pid10"
    _modules = {}

    def __init__(self, parent):
        if parent is None or not hasattr(parent, '_index'):
            return _err(PID10, '__init__', 'parent', parent, 'CheeseStick instance — use cheesestick.PID10()')
        parent_index = parent._index
        if parent_index in PID10._modules:
            mod = PID10._modules[parent_index]
            if mod: mod.dispose()
        self._parent = parent
        self._parent_index = parent_index
        PID10._modules[parent_index] = self

    def dispose(self):
        PID10._modules.pop(self._parent_index, None)

    def start(self):
        self._parent.write(self._parent.PID, 10)

    def distance(self) -> Union[int, float]:
        return self._parent.read(self._parent.PID10_DISTANCE)

    def echo_time(self) -> Union[int, float]:
        return self._parent.read(self._parent.PID10_ECHOTIME)
