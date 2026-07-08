# Based on the ROBOID project - http://hamster.school
# Copyright (c) 2016 Kwang-Hyun Park (akaii@kw.ac.kr)
#
# Modified by Robomation in 2026.
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

import time
import threading

from robomation.core.runner import Runner
from robomation.core.model import DeviceType, DataType, Roboid
from robomation.core.utils import Utils
from robomation.core.serial_connector import SerialConnector, Result
from robomation.roboids.hamster import Hamster  

class HamsterConnectionChecker(object):
    def __init__(self, roboid):
        self._roboid = roboid

    def check(self, info):
        return info[2] == "04"

class HamsterRoboid(Roboid):
    def __init__(self, index):
        super(HamsterRoboid, self).__init__(Hamster.ID, "Hamster", 0x00400000)
        self._index = index
        self._connector = None
        self._ready = False
        self._thread = None
        self._thread_lock = threading.Lock()

        # ── Motoring 상태 (effector: 매 사이클 read / command: _is_written latch) ──
        self._left_wheel = -128
        self._right_wheel = -128
        self._line_tracer_speed = 5
        self._left_led = 0
        self._right_led = 0
        self._sound_buzz = 0
        self._sound_note = 0

        self._io_a_mode = 0
        self._io_a_output = 0
        self._io_b_mode = 0
        self._io_b_output = 0
        self._gripper = 0
        self._shooter = 0

        self._config_ir_current = 2
        self._config_g_range = 0
        self._config_g_band = 0

        self._line_tracer_mode = 0
        self._line_tracer_mode_written = False
        self._line_tracer_id = 0
        self._line_tracer_event = 0
        self._line_tracer_state = 0
                
        self._packet_sent = 0
        self._packet_received = 0

        self._create_model()

    def _create_model(self):
        dict = self._device_dict = {}
        dict[Hamster.LEFT_WHEEL] = self._left_wheel_device = self._add_device(Hamster.LEFT_WHEEL, "LeftWheel", DeviceType.EFFECTOR, DataType.INTEGER, 1, -128, 127, -128)
        dict[Hamster.RIGHT_WHEEL] = self._right_wheel_device = self._add_device(Hamster.RIGHT_WHEEL, "RightWheel", DeviceType.EFFECTOR, DataType.INTEGER, 1, -128, 127, -128)
        dict[Hamster.LINE_TRACER_SPEED] = self._line_tracer_speed_device = self._add_device(Hamster.LINE_TRACER_SPEED, "LineTracerSpeed", DeviceType.EFFECTOR, DataType.INTEGER, 1, 1, 10, 5)
        dict[Hamster.LEFT_LED] = self._left_led_device = self._add_device(Hamster.LEFT_LED, "LeftLed", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 7, 0)
        dict[Hamster.RIGHT_LED] = self._right_led_device = self._add_device(Hamster.RIGHT_LED, "RightLed", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 7, 0)
        dict[Hamster.SOUND_BUZZ] = self._sound_buzz_device = self._add_device(Hamster.SOUND_BUZZ, "SoundBuzz", DeviceType.EFFECTOR, DataType.FLOAT, 1, 0, 6553.5, 0)
        dict[Hamster.SOUND_NOTE] = self._sound_note_device = self._add_device(Hamster.SOUND_NOTE, "SoundNote", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 88, 0)
        
        dict[Hamster.IO_A_MODE] = self._io_a_mode_device = self._add_device(Hamster.IO_A_MODE, "IoAMode", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 15, 0)
        dict[Hamster.IO_A_OUTPUT] = self._io_a_output_device = self._add_device(Hamster.IO_A_OUTPUT, "IoAOutput", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 255, 0)
        dict[Hamster.IO_B_MODE] = self._io_b_mode_device = self._add_device(Hamster.IO_B_MODE, "IoBMode", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 15, 0)
        dict[Hamster.IO_B_OUTPUT] = self._io_b_output_device = self._add_device(Hamster.IO_B_OUTPUT, "IoBOutput", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 255, 0)
        dict[Hamster.GRIPPER] = self._gripper_device = self._add_device(Hamster.GRIPPER, "Gripper", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 3, 0)
        dict[Hamster.SHOOTER] = self._shooter_device = self._add_device(Hamster.SHOOTER, "Shooter", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 255, 0)
        
        dict[Hamster.CONFIG_IR_CURRENT] = self._config_ir_current_device = self._add_device(Hamster.CONFIG_IR_CURRENT, "ConfigIrCurrent", DeviceType.EFFECTOR, DataType.INTEGER, 1, 1, 7, 2)
        dict[Hamster.CONFIG_G_RANGE] = self._config_g_range_device = self._add_device(Hamster.CONFIG_G_RANGE, "ConfigGRange", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 3, 0)
        dict[Hamster.CONFIG_G_BAND] = self._config_g_band_device = self._add_device(Hamster.CONFIG_G_BAND, "ConfigGBand", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 7, 0)
        
        dict[Hamster.LINE_TRACER_MODE] = self._line_tracer_mode_device = self._add_device(Hamster.LINE_TRACER_MODE, "LineTracerMode", DeviceType.COMMAND, DataType.INTEGER, 1, 0, 15, 0)

        dict[Hamster.IO_A_INPUT] = self._io_a_input_device = self._add_device(Hamster.IO_A_INPUT, "IoAInput", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 255, 0)
        dict[Hamster.IO_B_INPUT] = self._io_b_input_device = self._add_device(Hamster.IO_B_INPUT, "IoBInput", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 255, 0)
        
        dict[Hamster.LEFT_PROXIMITY] = self._left_proximity_device = self._add_device(Hamster.LEFT_PROXIMITY, "LeftProximity", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 100, 0)
        dict[Hamster.RIGHT_PROXIMITY] = self._right_proximity_device = self._add_device(Hamster.RIGHT_PROXIMITY, "RightProximity", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 100, 0)
        dict[Hamster.LIGHT] = self._light_device = self._add_device(Hamster.LIGHT, "Light", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 600, 0)
        dict[Hamster.LEFT_FLOOR] = self._left_floor_device = self._add_device(Hamster.LEFT_FLOOR, "LeftFloor", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 100, 0)
        dict[Hamster.RIGHT_FLOOR] = self._right_floor_device = self._add_device(Hamster.RIGHT_FLOOR, "RightFloor", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 100, 0)
        dict[Hamster.ACCELERATION_X] = self._acceleration_x_device = self._add_device(Hamster.ACCELERATION_X, "AccelerationX", DeviceType.SENSOR, DataType.INTEGER, 1, -32768, 32767, 0)
        dict[Hamster.ACCELERATION_Y] = self._acceleration_y_device = self._add_device(Hamster.ACCELERATION_Y, "AccelerationY", DeviceType.SENSOR, DataType.INTEGER, 1, -32768, 32767, 0)
        dict[Hamster.ACCELERATION_Z] = self._acceleration_z_device = self._add_device(Hamster.ACCELERATION_Z, "AccelerationZ", DeviceType.SENSOR, DataType.INTEGER, 1, -32768, 32767, 0)
        dict[Hamster.BATTERY] = self._battery_device = self._add_device(Hamster.BATTERY, "Battery", DeviceType.SENSOR, DataType.FLOAT, 1, 2.0, 5.0, 0)
        dict[Hamster.TEMPERATURE] = self._temperature_device = self._add_device(Hamster.TEMPERATURE, "Temperature", DeviceType.SENSOR, DataType.INTEGER, 1, -40, 88, 0)
        dict[Hamster.SIGNAL_STRENGTH] = self._signal_strength_device = self._add_device(Hamster.SIGNAL_STRENGTH, "SignalStrength", DeviceType.SENSOR, DataType.INTEGER, 1, -128, 0, 0)

        dict[Hamster.LINE_TRACER_STATE] = self._line_tracer_state_device = self._add_device(Hamster.LINE_TRACER_STATE, "LineTracerState", DeviceType.EVENT, DataType.INTEGER, 1, 0, 1, 0)        

    def find_device_by_id(self, device_id):
        return self._device_dict.get(device_id)

    def _run(self):
        try:
            while self._running or self._releasing > 0:
                if self._receive(self._connector):
                    self._send(self._connector)
                    if self._releasing > 0:
                        self._releasing -= 1
                time.sleep(0.01)
        except Exception:                  
            import traceback
            traceback.print_exc()
            # pass

    def _init(self, port_name=None):
        Runner.register_required()
        self._running = True
        self._releasing = 0
        thread = threading.Thread(target=self._run)
        self._thread = thread
        thread.daemon = True
        thread.start()

        tag = "Hamster[{}]".format(self._index)
        self._connector = SerialConnector(tag, HamsterConnectionChecker(self))
        result = self._connector.open(port_name)
        if result == Result.FOUND:
            while self._ready == False and self._is_disposed() == False:
                time.sleep(0.01)
        elif result == Result.NOT_AVAILABLE:
            Runner.register_checked()

    def _release(self):
        if self._ready:
            self._releasing = 5
        self._running = False
        thread = self._thread
        self._thread = None
        if thread:
            thread.join()

        connector = self._connector
        self._connector = None
        if connector:
            connector.close()

    def _dispose(self):
        if self._is_disposed() == False:
            super(HamsterRoboid, self)._dispose()
            self._release()
    
    def _reset(self):
        super(HamsterRoboid, self)._reset()

        # ── Motoring 상태 (effector: 매 사이클 read / command: _is_written latch) ──
        self._left_wheel = -128
        self._right_wheel = -128
        self._line_tracer_speed = 5
        self._left_led = 0
        self._right_led = 0
        self._sound_buzz = 0
        self._sound_note = 0

        self._io_a_mode = 0
        self._io_a_output = 0
        self._io_b_mode = 0
        self._io_b_output = 0
        self._gripper = 0
        self._shooter = 0

        self._config_ir_current = 2
        self._config_g_range = 0
        self._config_g_band = 0

        self._line_tracer_mode = 0
        self._line_tracer_mode_written = False
        self._line_tracer_id = 0
        self._line_tracer_event = 0
        self._line_tracer_state = 0
    
    def _request_motoring_data(self):
        with self._thread_lock:
            # EFFECTOR — 매 사이클 현재 값 반영
            self._left_wheel = self._left_wheel_device.read()
            self._right_wheel = self._right_wheel_device.read()
            self._line_tracer_speed = self._line_tracer_speed_device.read()
            self._left_led = self._left_led_device.read()
            self._right_led = self._right_led_device.read()
            self._sound_buzz = self._sound_buzz_device.read()
            self._sound_note = self._sound_note_device.read()

            self._io_a_mode = self._io_a_mode_device.read()
            self._io_a_output = self._io_a_output_device.read()
            self._io_b_mode = self._io_b_mode_device.read()
            self._io_b_output = self._io_b_output_device.read()
            self._gripper = self._gripper_device.read()
            self._shooter = self._shooter_device.read()

            self._config_ir_current = self._config_ir_current_device.read()
            self._config_g_range = self._config_g_range_device.read()
            self._config_g_band = self._config_g_band_device.read()
            
            # COMMAND — _is_written latch (한 번 쓰면 펌웨어로 전송)
            if self._line_tracer_mode_device._is_written():
                self._line_tracer_mode = self._line_tracer_mode_device.read()
                self._line_tracer_mode_written = True
        self._clear_written()

    def _decode_sensory_packet(self, packet):
        packet = str(packet)
        self._packet_received = 0
        value = int(packet[4:5], 16)
        if value != 1:
            return False
        
        signal_strength = self._to_int8(int(packet[6:8], 16))
        self._signal_strength_device._put(signal_strength)

        left_proximity = int(packet[8:10], 16)
        self._left_proximity_device._put(left_proximity)
        right_proximity = int(packet[10:12], 16)
        self._right_proximity_device._put(right_proximity)
        left_floor = int(packet[12:14], 16)
        self._left_floor_device._put(left_floor)
        right_floor = int(packet[14:16], 16)
        self._right_floor_device._put(right_floor)

        acc_x = self._to_int16(int(packet[16:20], 16))
        acc_y = self._to_int16(int(packet[20:24], 16))
        acc_z = self._to_int16(int(packet[24:28], 16))
        self._acceleration_x_device._put(acc_x)
        self._acceleration_y_device._put(acc_y)
        self._acceleration_z_device._put(acc_z)

        flag = int(packet[28:30], 16)
        if flag == 0:
            light = int(packet[30:34], 16)
            self._light_device._put(light)
        else:
            temperature = self._to_int8(int(packet[30:32], 16))
            temperature = Utils.round(temperature / 2.0 + 23.0)
            self._temperature_device._put(temperature)

            battery = int(packet[32:34], 16)
            battery = round(2.0 + battery / 100, 2)
            self._battery_device._put(battery)

        in_a = int(packet[34:36], 16)
        in_b = int(packet[36:38], 16)
        self._io_a_input_device._put(in_a)
        self._io_b_input_device._put(in_b)
        
        trace = int(packet[38:40], 16)
        if self._line_tracer_event:
            if trace == 0x40:
                self._line_tracer_state_device._put_empty()
                self._line_tracer_event = 0

        self._packet_received = 1
        return True

    def _encode_motoring_packet(self, address):
        result = ""
        with self._thread_lock:
            result = "00" * 2
            result += "10"

            # ── 바퀴 속도 ──
            left_wheel = 0 if self._left_wheel == -128 else self._left_wheel
            right_wheel = 0 if self._right_wheel == -128 else self._right_wheel
            result += self._to_hex(left_wheel)
            result += self._to_hex(right_wheel)

            # ── LED ──
            result += self._to_hex(self._left_led)
            result += self._to_hex(self._right_led)

            # ── 사운드 : note > buzz 우선순위 ──
            if self._sound_note > 0:
                result += "00" * 3
                result += self._to_hex(self._sound_note)
            else:
                result += self._to_hex3(Utils.round(self._sound_buzz * 100))
                result += "00"

            # ── 라인트레이싱 ──
            if self._line_tracer_mode_written:
                if self._line_tracer_mode > 0:
                    self._line_tracer_id = (self._line_tracer_id % 15) + 1
                    self._line_tracer_event = 1
                self._line_tracer_mode_written = False
            temp = self._line_tracer_speed
            temp |= (self._line_tracer_mode & 0x0f) << 3
            temp |= (self._line_tracer_id & 0x01) << 7
            result += self._to_hex(temp)

            # ── config (IR current / G range / G band)
            result += self._to_hex(self._config_ir_current)
            temp = self._config_g_band
            temp |= self._config_g_range << 4
            result += self._to_hex(temp)

            # ── IO 모드 / 출력 / 그리퍼 / 슈터 ──
            gripper = self._gripper
            shooter = self._shooter

            if gripper > 0:
                result += "AA"
                if gripper == 1:
                    result += self._to_hex(1)
                    result += self._to_hex(0)
                elif gripper == 2:
                    result += self._to_hex(0)
                    result += self._to_hex(1)
                elif gripper == 3:
                    result += self._to_hex(0)
                    result += self._to_hex(0)
            elif shooter > 0:
                result += "80"
                result += self._to_hex(shooter)
            else:
                temp = (self._io_a_mode & 0x0f) << 4
                temp |= (self._io_b_mode & 0x0f)
                result += self._to_hex(temp)
                result += self._to_hex(self._io_a_output)
                result += self._to_hex(self._io_b_output)
            result += "00" * 3
            
            result += "-"
            result += address
            result += "\r"
            self._packet_sent = 1
            return result

    def _receive(self, connector):
        if connector:
            packets = connector.read()
            if packets:
                for packet in packets:
                    if self._decode_sensory_packet(packet):
                        if self._ready == False:
                            self._ready = True
                            Runner.register_checked()
                        self._notify_sensory_device_data_changed()
                return True
        return False

    def _send(self, connector):
        if connector:
            packet = self._encode_motoring_packet(connector.get_address())
            connector.write(packet)
