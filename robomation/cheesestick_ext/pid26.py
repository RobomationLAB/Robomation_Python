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


class PID26:
    """Environment sensor (temperature/humidity/pressure) on CheeseStick PID slot."""
    ID = "kr.robomation.physical.module.cheesestick.pid26"
    _modules = {}

    def __init__(self, parent):
        if parent is None or not hasattr(parent, '_index'):
            return _err(PID26, '__init__', 'parent', parent, 'CheeseStick instance — use cheesestick.PID26()')
        parent_index = parent._index
        if parent_index in PID26._modules:
            mod = PID26._modules[parent_index]
            if mod: mod.dispose()
        self._parent = parent
        self._parent_index = parent_index
        PID26._modules[parent_index] = self

    def dispose(self):
        PID26._modules.pop(self._parent_index, None)

    def start(self):
        self._parent.write(self._parent.PID, 26)

    def temperature(self) -> Union[int, float]:
        return self._parent.read(self._parent.PID26_TEMPERATURE)

    def humidity(self) -> Union[int, float]:
        return self._parent.read(self._parent.PID26_HUMIDITY)

    def pressure(self) -> Union[int, float]:
        return self._parent.read(self._parent.PID26_PRESSURE)
