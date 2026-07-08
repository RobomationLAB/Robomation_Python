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
_CSD03Port = Literal['Sa', 'Sb', 'Sc']

class CSD03:
    """Rotary potentiometer module (analog input on CheeseStick S-port)."""
    ID = "kr.robomation.physical.module.cheesestick.csd03"
    _modules = {}

    _VALID_PORTS = get_args(_CSD03Port)

    def __init__(self, parent):
        if parent is None or not hasattr(parent, '_index'):
            return _err(CSD03, '__init__', 'parent', parent, 'CheeseStick instance — use cheesestick.CSD03()')
        parent_index = parent._index
        if parent_index in CSD03._modules:
            mod = CSD03._modules[parent_index]
            if mod: mod.dispose()
        self._parent = parent
        self._parent_index = parent_index
        self._port = None
        CSD03._modules[parent_index] = self

    def dispose(self):
        CSD03._modules.pop(self._parent_index, None)

    def _resolve_port(self, method, unit):
        if unit is None:
            unit = self._port
        if unit not in CSD03._VALID_PORTS:
            return _err(CSD03, method, 'unit', unit, CSD03._VALID_PORTS)
        return unit

    def set_port(self, unit: _CSD03Port):
        if unit not in CSD03._VALID_PORTS:
            return _err(CSD03, 'set_port', 'unit', unit, CSD03._VALID_PORTS)
        self._port = unit
        self._parent.set_input_mode(unit, 'analog')

    def set_input_range(self, unit: Optional[_CSD03Port], 
                        src_min: Union[int, float], src_max: Union[int, float], 
                        dst_min: Union[int, float], dst_max: Union[int, float]):
        unit = self._resolve_port('set_input_range', unit)
        if unit is None: return
        self._parent._set_io_range(unit, src_min, None, src_max, dst_min, None, dst_max)

    def set_input_range_median(self, unit: Optional[_CSD03Port], 
                               src_min: Union[int, float], src_median: Union[int, float], src_max: Union[int, float], 
                               dst_min: Union[int, float], dst_median: Union[int, float], dst_max: Union[int, float]):
        unit = self._resolve_port('set_input_range_median', unit)
        if unit is None: return
        self._parent._set_io_range(unit, src_min, src_median, src_max, dst_min, dst_median, dst_max)

    def get_input(self, unit: Optional[_CSD03Port] = None) -> Union[int, float]:
        unit = self._resolve_port('get_input', unit)
        if unit is None: return 0
        return self._parent.get_input(unit)
