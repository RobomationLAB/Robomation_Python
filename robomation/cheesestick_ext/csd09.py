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
from robomation.core.runner import Runner


# ── Module-level Literal aliases (for IDE auto-complete) ─────────────────────
_CSD09ServoPort = Literal['Sa', 'Sb', 'Sc']
_CSD09DCPort    = Literal['Mab', 'Mcd']
_CSD09StepMode = Literal['off', 'wave_step', 'full_step']

class CSD09:
    """Motor control module (servo/DC/stepper) on CheeseStick."""
    ID = "kr.robomation.physical.module.cheesestick.csd09"
    _modules = {}

    _VALID_SERVO_PORTS = get_args(_CSD09ServoPort)
    _VALID_DC_PORTS    = get_args(_CSD09DCPort)
    _VALID_STEP_MODES = get_args(_CSD09StepMode)
    # wire 매핑 — 펌웨어 명세 확정 시 교체
    _VALID_STEP_MODES_TO_WIRE = {m: i for i, m in enumerate(_VALID_STEP_MODES)}

    def __init__(self, parent):
        if parent is None or not hasattr(parent, '_index'):
            return _err(CSD09, '__init__', 'parent', parent, 'CheeseStick instance — use cheesestick.CSD09()')
        parent_index = parent._index
        if parent_index in CSD09._modules:
            mod = CSD09._modules[parent_index]
            if mod: mod.dispose()
        self._parent = parent
        self._parent_index = parent_index
        self._servo_port = None
        self._dc_port = None
        CSD09._modules[parent_index] = self

    def dispose(self):
        CSD09._modules.pop(self._parent_index, None)

    def _resolve_servo_port(self, method, unit):
        if unit is None:
            unit = self._servo_port
        if unit not in CSD09._VALID_SERVO_PORTS:
            _err(CSD09, method, 'unit', unit, CSD09._VALID_SERVO_PORTS)
        return unit

    def _resolve_dc_port(self, method, unit):
        if unit is None:
            unit = self._dc_port
        if unit not in CSD09._VALID_DC_PORTS:
            _err(CSD09, method, 'unit', unit, CSD09._VALID_DC_PORTS)
        return unit

    # ── Servo motor ──────────────────────────────────────────────────────────
    def start_servo_motor(self, unit: _CSD09ServoPort):
        if unit not in CSD09._VALID_SERVO_PORTS:
            return _err(CSD09, 'start_servo_motor', 'unit', unit, CSD09._VALID_SERVO_PORTS)
        self._servo_port = unit
        self._parent.write(self._parent._MODE_DEVICE_IDS[unit], self._parent._VALID_OUTPUT_MODES['analog_servo'])

    def set_servo_motor(self, unit: Union[_CSD09ServoPort, int, float, None], value: Union[int, float, None] = None):
        # Backward-compat: set_servo_motor(value) via stored servo port.
        if value is None and isinstance(unit, (int, float)):
            value = unit
            unit = self._servo_port
        unit = self._resolve_servo_port('set_servo_motor', unit)
        if unit is None: return
        if not isinstance(value, (int, float)):
            return _err(CSD09, 'set_servo_motor', 'value', value, 'int | float')
        self._parent.write(self._parent._SERVO_OUT_DEVICE_IDS[unit], value)

    def change_servo_motor(self, unit: Union[_CSD09ServoPort, int, float, None], value: Union[int, float, None] = None):
        if value is None and isinstance(unit, (int, float)):
            value = unit
            unit = self._servo_port
        unit = self._resolve_servo_port('change_servo_motor', unit)
        if unit is None: return
        if not isinstance(value, (int, float)):
            return _err(CSD09, 'change_servo_motor', 'value', value, 'int | float')
        dev = self._parent._SERVO_OUT_DEVICE_IDS[unit]
        self._parent.write(dev, self._parent.read(dev) + value)

    def stop_servo_motor(self, unit: Optional[_CSD09ServoPort] = None):
        unit = self._resolve_servo_port('stop_servo_motor', unit)
        if unit is None: return
        self._parent.write(self._parent._MODE_DEVICE_IDS[unit], 0)

    # ── DC motor ─────────────────────────────────────────────────────────────
    def start_dc_motor(self, unit: _CSD09DCPort):
        if unit not in CSD09._VALID_DC_PORTS:
            return _err(CSD09, 'start_dc_motor', 'unit', unit, CSD09._VALID_DC_PORTS)
        self._dc_port = unit
        self._parent.write(self._parent._MODE_DEVICE_IDS[unit], self._parent._VALID_OUTPUT_MODES['pwm'])

    def set_dc_motor(self, unit: Union[_CSD09DCPort, int, float, None], value: Union[int, float, None] = None):
        if value is None and isinstance(unit, (int, float)):
            value = unit
            unit = self._dc_port
        unit = self._resolve_dc_port('set_dc_motor', unit)
        if unit is None: return
        if not isinstance(value, (int, float)):
            return _err(CSD09, 'set_dc_motor', 'value', value, 'int | float')
        self._parent.write(self._parent._PWM_OUT_DEVICE_IDS[unit], value)

    def change_dc_motor(self, unit: Union[_CSD09DCPort, int, float, None], value: Union[int, float, None] = None):
        if value is None and isinstance(unit, (int, float)):
            value = unit
            unit = self._dc_port
        unit = self._resolve_dc_port('change_dc_motor', unit)
        if unit is None: return
        if not isinstance(value, (int, float)):
            return _err(CSD09, 'change_dc_motor', 'value', value, 'int | float')
        dev = self._parent._PWM_OUT_DEVICE_IDS[unit]
        self._parent.write(dev, self._parent.read(dev) + value)

    def stop_dc_motor(self, unit: Optional[_CSD09DCPort] = None):
        unit = self._resolve_dc_port('stop_dc_motor', unit)
        if unit is None: return
        self._parent.write(self._parent._PWM_OUT_DEVICE_IDS[unit], 0)

    # ── Step motor ───────────────────────────────────────────────────────────
    def start_step_motor(self):
        self._parent.write(self._parent.M_MODE, 2)  # step_motor 
        self._parent.write(self._parent.M_DRIVER, CSD09._VALID_STEP_MODES_TO_WIRE['full_step'])

    def set_step_motor_mode(self, unit: _CSD09StepMode):
        if unit not in CSD09._VALID_STEP_MODES:
            return _err(CSD09, 'set_step_motor_mode', 'unit', unit, CSD09._VALID_STEP_MODES)
        self._parent.write(self._parent.M_DRIVER, CSD09._VALID_STEP_MODES_TO_WIRE[unit])

    def set_step_motor_speed(self, value: Union[int, float]):
        if not isinstance(value, (int, float)):
            return _err(CSD09, 'set_step_motor_speed', 'value', value, 'int | float')
        self._parent.write(self._parent.M_STEP_PPS, value)

    def _rotate_step_motor_impl(self, value):
        self._parent.write(self._parent.M_STEP_MOVE, value)
        Runner.wait_until(self._parent._evaluate_m_step)

    def rotate_step_motor(self, value: Union[int, float], wait: bool = True):
        if not isinstance(value, (int, float)):
            return _err(CSD09, 'rotate_step_motor', 'value', value, 'int | float')
        Runner.dispatch(lambda: self._rotate_step_motor_impl(value), wait)

    def change_step_motor_speed(self, value: Union[int, float]):
        if not isinstance(value, (int, float)):
            return _err(CSD09, 'change_step_motor_speed', 'value', value, 'int | float')
        dev = self._parent.M_STEP_PPS
        self._parent.write(dev, self._parent.read(dev) + value)

    def stop_step_motor(self):
        # Stop motion only — keep driver power on.
        self._parent.write(self._parent.M_STEP_PPS, 0)
        self._parent.write(self._parent.M_STEP_MOVE, 0)

    def turn_off_step_motor(self):
        # Stop motion and cut driver power.
        self._parent.write(self._parent.M_DRIVER, 0)
        self._parent.write(self._parent.M_STEP_PPS, 0)
        self._parent.write(self._parent.M_STEP_MOVE, 0)

    def get_steps(self) -> int:
        return self._parent.read(self._parent.M_STEP_MOVE)
