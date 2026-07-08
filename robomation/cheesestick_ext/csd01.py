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

from typing import Literal, Optional, get_args
from robomation.core.error import _err


# ── Module-level Literal aliases (for IDE auto-complete) ─────────────────────
_CSD01Port = Literal['Sa', 'Sb', 'Sc']

class CSD01:
    """Tact switch module (operates on CheeseStick S-port)."""
    ID = "kr.robomation.physical.module.cheesestick.csd01"
    _modules = {}

    _VALID_PORTS = get_args(_CSD01Port)

    def __init__(self, parent):
        if parent is None or not hasattr(parent, '_index'):
            return _err(CSD01, '__init__', 'parent', parent, 'CheeseStick instance — use cheesestick.CSD01()')
        parent_index = parent._index
        if parent_index in CSD01._modules:
            mod = CSD01._modules[parent_index]
            if mod: mod.dispose()
        self._parent = parent
        self._parent_index = parent_index
        self._port = None
        CSD01._modules[parent_index] = self

    def dispose(self):
        CSD01._modules.pop(self._parent_index, None)

    def _resolve_port(self, method, unit):
        if unit is None:
            unit = self._port
        if unit not in CSD01._VALID_PORTS:
            return _err(CSD01, method, 'unit', unit, CSD01._VALID_PORTS)
        return unit

    def set_port(self, unit: _CSD01Port):
        if unit not in CSD01._VALID_PORTS:
            return _err(CSD01, 'set_port', 'unit', unit, CSD01._VALID_PORTS)
        self._port = unit
        self._parent.set_input_mode(unit, 'digital_pullup')

    def button_input(self, unit: Optional[_CSD01Port] = None) -> int:
        unit = self._resolve_port('button_input', unit)
        if unit is None: return 0
        return self._parent.get_input(unit)

    def button_pressed(self, unit: Optional[_CSD01Port] = None) -> bool:
        unit = self._resolve_port('button_pressed', unit)
        if unit is None: return False
        return self._parent.get_input(unit) == 0
