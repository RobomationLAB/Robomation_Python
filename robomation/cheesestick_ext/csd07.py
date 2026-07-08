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

from typing import Literal, Optional, Union, get_args
from robomation.core.error import _err


# ── Module-level Literal aliases (for IDE auto-complete) ─────────────────────
_CSD07Port = Literal['Sa', 'Sb', 'Sc']

class CSD07:
    """Sound sensor module (analog input on CheeseStick S-port)."""
    ID = "kr.robomation.physical.module.cheesestick.csd07"
    _modules = {}

    _VALID_PORTS = get_args(_CSD07Port)

    def __init__(self, parent):
        if parent is None or not hasattr(parent, '_index'):
            return _err(CSD07, '__init__', 'parent', parent, 'CheeseStick instance — use cheesestick.CSD07()')
        parent_index = parent._index
        if parent_index in CSD07._modules:
            mod = CSD07._modules[parent_index]
            if mod: mod.dispose()
        self._parent = parent
        self._parent_index = parent_index
        self._port = None
        CSD07._modules[parent_index] = self

    def dispose(self):
        CSD07._modules.pop(self._parent_index, None)

    def _resolve_port(self, method, unit):
        if unit is None:
            unit = self._port
        if unit not in CSD07._VALID_PORTS:
            return _err(CSD07, method, 'unit', unit, CSD07._VALID_PORTS)
        return unit

    def set_port(self, unit: _CSD07Port):
        if unit not in CSD07._VALID_PORTS:
            return _err(CSD07, 'set_port', 'unit', unit, CSD07._VALID_PORTS)
        self._port = unit
        self._parent.set_input_mode(unit, 'analog')

    def get_input(self, unit: Optional[_CSD07Port] = None) -> Union[int, float]:
        unit = self._resolve_port('get_input', unit)
        if unit is None: return 0
        return self._parent.get_input(unit)
