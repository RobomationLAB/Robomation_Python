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
import math

from robomation.core.runner import Runner
from robomation.core.model import DeviceType, DataType, Roboid
from robomation.core.utils import Utils
from robomation.core.serial_connector import SerialConnector, Result
from robomation.roboids.turtle import Turtle  

class TurtleConnectionChecker(object):
    def __init__(self, roboid):
        self._roboid = roboid

    def check(self, info):
        return info[2] == "09"

class TurtleRoboid(Roboid):
    def __init__(self, index):
        super(TurtleRoboid, self).__init__(Turtle.ID, "Turtle", 0x00900000)
        self._index = index
        self._connector = None
        self._ready = False
        self._thread = None
        self._thread_lock = threading.Lock()

        # ── Motoring 상태 (effector: 매 사이클 read / command: _is_written latch) ──
        self._left_wheel = 0
        self._right_wheel = 0
        self._line_tracer_speed = 0
        self._line_tracer_gain = 0
        self._head_led = [0, 0, 0]
        self._sound_buzz = 0

        self._wheel_move = 0
        self._line_tracer_mode = 0
        self._sound_note = 0
        self._sound_clip = 0

        self._wheel_move_written = False
        self._line_tracer_mode_written = False
        self._sound_written = False

        # ── encode/decode 완료-이벤트 추적용 내부 상태 (Runner.wait_until 연동) ──
        self._wheel_id = 0
        self._wheel_move_prev = -1
        self._wheel_event = 0
        self._wheel_state = 0

        self._line_tracer_id = 0
        self._line_tracer_event = 0
        self._line_tracer_state = 0
        
        self._sound_id = 0
        self._sound_event = 0
        self._sound_state = 0
        
        self._event_move_id = -1
        self._event_trace_id = -1
        self._event_clip_id = -1
        self._event_pattern_id = -1
        self._event_click_id = -1
        self._event_long_click_id = -1
        self._event_long_long_click_id = -1
        
        self._packet_sent = 0
        self._packet_received = 0

        self._create_model()

    def _create_model(self):
        dict = self._device_dict = {}
        dict[Turtle.LEFT_WHEEL] = self._left_wheel_device = self._add_device(Turtle.LEFT_WHEEL, "LeftWheel", DeviceType.EFFECTOR, DataType.INTEGER, 1, -128, 127, -128)
        dict[Turtle.RIGHT_WHEEL] = self._right_wheel_device = self._add_device(Turtle.RIGHT_WHEEL, "RightWheel", DeviceType.EFFECTOR, DataType.INTEGER, 1, -128, 127, -128)
        dict[Turtle.LINE_TRACER_SPEED] = self._line_tracer_speed_device = self._add_device(Turtle.LINE_TRACER_SPEED, "LineTracerSpeed", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 7, 0)
        dict[Turtle.LINE_TRACER_GAIN] = self._line_tracer_gain_device = self._add_device(Turtle.LINE_TRACER_GAIN, "LineTracerGain", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 7, 0)
        dict[Turtle.HEAD_LED] = self._head_led_device = self._add_device(Turtle.HEAD_LED, "HeadLed", DeviceType.EFFECTOR, DataType.INTEGER, 3, 0, 255, 0)
        dict[Turtle.SOUND_BUZZ] = self._sound_buzz_device = self._add_device(Turtle.SOUND_BUZZ, "SoundBuzz", DeviceType.EFFECTOR, DataType.FLOAT, 1, 0, 167000.0, 0)
        
        dict[Turtle.WHEEL_MOVE] = self._wheel_move_device = self._add_device(Turtle.WHEEL_MOVE, "WheelMove", DeviceType.COMMAND, DataType.INTEGER, 1, 0, 65535, 0)
        dict[Turtle.LINE_TRACER_MODE] = self._line_tracer_mode_device = self._add_device(Turtle.LINE_TRACER_MODE, "LineTracerMode", DeviceType.COMMAND, DataType.INTEGER, 1, 0, 63, 0)
        dict[Turtle.SOUND_NOTE] = self._sound_note_device = self._add_device(Turtle.SOUND_NOTE, "SoundNote", DeviceType.COMMAND, DataType.INTEGER, 1, 0, 88, 0)
        dict[Turtle.SOUND_CLIP] = self._sound_clip_device = self._add_device(Turtle.SOUND_CLIP, "SoundClip", DeviceType.COMMAND, DataType.INTEGER, 1, 0, 67, 0)

        dict[Turtle.WHEEL_COUNT] = self._wheel_count_device = self._add_device(Turtle.WHEEL_COUNT, "WheelCount", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 65535, 0)
        dict[Turtle.SOUND_PLAYING] = self._sound_playing_device = self._add_device(Turtle.SOUND_PLAYING, "SoundPlaying", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[Turtle.FLOOR] = self._floor_device = self._add_device(Turtle.FLOOR, "Floor", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 100, 0)
        dict[Turtle.CARD_COLOR] = self._card_color_device = self._add_device(Turtle.CARD_COLOR, "CardColor", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 7, 0)
        dict[Turtle.CARD_PATTERN] = self._card_pattern_device = self._add_device(Turtle.CARD_PATTERN, "CardPattern", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 65, 0)
        dict[Turtle.BUTTON_PRESSED] = self._button_pressed_device = self._add_device(Turtle.BUTTON_PRESSED, "ButtonPressed", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[Turtle.ACCELERATION_X] = self._acceleration_x_device = self._add_device(Turtle.ACCELERATION_X, "AccelerationX", DeviceType.SENSOR, DataType.INTEGER, 1, -128, 127, 0)
        dict[Turtle.ACCELERATION_Y] = self._acceleration_y_device = self._add_device(Turtle.ACCELERATION_Y, "AccelerationY", DeviceType.SENSOR, DataType.INTEGER, 1, -128, 127, 0)
        dict[Turtle.ACCELERATION_Z] = self._acceleration_z_device = self._add_device(Turtle.ACCELERATION_Z, "AccelerationZ", DeviceType.SENSOR, DataType.INTEGER, 1, -128, 127, 0)
        dict[Turtle.TEMPERATURE] = self._temperature_device = self._add_device(Turtle.TEMPERATURE, "Temperature", DeviceType.SENSOR, DataType.INTEGER, 1, -40, 88, 0)
        dict[Turtle.SIGNAL_STRENGTH] = self._signal_strength_device = self._add_device(Turtle.SIGNAL_STRENGTH, "SignalStrength", DeviceType.SENSOR, DataType.INTEGER, 1, -128, 0, 0)
        dict[Turtle.BATTERY] = self._battery_device = self._add_device(Turtle.BATTERY, "Battery", DeviceType.SENSOR, DataType.FLOAT, 1, 2.0, 5.0, 0)

        dict[Turtle.WHEEL_STATE] = self._wheel_state_device = self._add_device(Turtle.WHEEL_STATE, "WheelState", DeviceType.EVENT, DataType.INTEGER, 1, 0, 1, 0)
        dict[Turtle.LINE_TRACER_STATE] = self._line_tracer_state_device = self._add_device(Turtle.LINE_TRACER_STATE, "LineTracerState", DeviceType.EVENT, DataType.INTEGER, 1, 0, 3, 0)
        dict[Turtle.SOUND_STATE] = self._sound_state_device = self._add_device(Turtle.SOUND_STATE, "SoundState", DeviceType.EVENT, DataType.INTEGER, 1, 0, 1, 0)
        dict[Turtle.BUTTON_CLICK] = self._button_click_device = self._add_device(Turtle.BUTTON_CLICK, "ButtonClick", DeviceType.EVENT, DataType.INTEGER, 1, 0, 3, 0)
        dict[Turtle.BUTTON_LONG_CLICK] = self._button_long_click_device = self._add_device(Turtle.BUTTON_LONG_CLICK, "ButtonLongClick", DeviceType.EVENT, DataType.INTEGER, 1, 0, 3, 0)
        dict[Turtle.BUTTON_LONG_LONG_CLICK] = self._button_long_long_click_device = self._add_device(Turtle.BUTTON_LONG_LONG_CLICK, "ButtonLongLongClick", DeviceType.EVENT, DataType.INTEGER, 1, 0, 3, 0)
        

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

        tag = "Turtle[{}]".format(self._index)
        self._connector = SerialConnector(tag, TurtleConnectionChecker(self))
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
            super(TurtleRoboid, self)._dispose()
            self._release()
    
    def _reset(self):
        super(TurtleRoboid, self)._reset()

        # ── Motoring 상태 (effector: 매 사이클 read / command: _is_written latch) ──
        self._left_wheel = 0
        self._right_wheel = 0
        self._line_tracer_speed = 0
        self._line_tracer_gain = 0
        self._head_led = [0, 0, 0]
        self._sound_buzz = 0

        self._wheel_move = 0
        self._line_tracer_mode = 0
        self._sound_note = 0
        self._sound_clip = 0

        self._wheel_move_written = False
        self._line_tracer_mode_written = False
        self._sound_written = False

        # ── encode/decode 완료-이벤트 추적용 내부 상태 (Runner.wait_until 연동) ──
        self._wheel_id = 0
        self._wheel_move_prev = -1
        self._wheel_event = 0
        self._wheel_state = 0

        self._line_tracer_id = 0
        self._line_tracer_event = 0
        self._line_tracer_state = 0
        
        self._sound_id = 0
        self._sound_event = 0
        self._sound_state = 0
        
        self._event_move_id = -1
        self._event_trace_id = -1
        self._event_clip_id = -1
        self._event_pattern_id = -1
        self._event_click_id = -1
        self._event_long_click_id = -1
        self._event_long_long_click_id = -1

    def _request_motoring_data(self):
        with self._thread_lock:
            # EFFECTOR — 매 사이클 현재 값 반영
            self._left_wheel = self._left_wheel_device.read()
            self._right_wheel = self._right_wheel_device.read()
            self._line_tracer_speed = self._line_tracer_speed_device.read()
            self._line_tracer_gain = self._line_tracer_gain_device.read()
            self._head_led = [self._head_led_device.read(0), self._head_led_device.read(1), self._head_led_device.read(2)]
            self._sound_buzz = self._sound_buzz_device.read()
            
            # COMMAND — _is_written latch (한 번 쓰면 펌웨어로 전송)
            if self._wheel_move_device._is_written():
                self._wheel_move = self._wheel_move_device.read()
                self._wheel_move_written = True
            if self._line_tracer_mode_device._is_written():
                self._line_tracer_mode = self._line_tracer_mode_device.read()
                self._line_tracer_mode_written = True
            if self._sound_note_device._is_written():
                self._sound_note = self._sound_note_device.read()
                self._sound_written = True
            if self._sound_clip_device._is_written():
                self._sound_clip = self._sound_clip_device.read()
                self._sound_written = True
        self._clear_written()

    @staticmethod
    def _RGB_HSV(red, green, blue):
        r = red / 1000
        g = green / 1000
        b = blue / 1000
        
        c_max = max(r,g,b)
        c_min = min(r,g,b)
        delta = c_max - c_min
        
        hue = 0
        sat = 0
        val = c_max
        
        if delta == 0: hue = 0
        elif c_max == r: hue = 60 * math.fmod((g-b)/delta, 60)
        elif c_max == g: hue = 60 * ((b-r)/delta + 2)
        elif c_max == b: hue = 60 * ((r-g)/delta + 4)
        
        if hue < 0: hue = 360 + hue

        if c_max != 0: sat = delta/c_max

        return hue, sat, val

    def _decode_sensory_packet(self, packet):
        packet = str(packet)
        self._packet_received = 0
        value = int(packet[0:1], 16)
        if value != 1:
            return False
        
        red = int(packet[2:6], 16)
        green = int(packet[6:10], 16)
        blue = int(packet[10:14], 16)
        # off = int(packet[14:16], 16)

        hue, sat, val = self._RGB_HSV(red, green, blue) 
        color = 0                                           # unknown
        if sat > 0.2:
            if val > 0.2:
                if hue < 15: color = 1                      # red
                elif hue >= 30 and hue < 70: color = 2      # yellow
                elif hue >= 80 and hue < 160: color = 3     # green
                elif hue >= 170 and hue < 200: color = 4    # cyan
                elif hue >= 210 and hue < 270: color = 5    # blue
                elif hue >= 290 and hue < 330: color = 6    # magenta
        elif val > 0.2: color = 7                           # white
        self._card_color_device._put(color)

        # ── 버튼 이벤트 바이트 ──
        states = int(packet[16:18], 16)
        click = (states >> 2) & 0x03
        if click != self._event_click_id and self._event_click_id != -1:
            self._button_click_device._put_empty()
        self._event_click_id = click
        long_click = (states >> 4) & 0x03
        if long_click != self._event_long_click_id and self._event_long_click_id != -1:
            self._button_long_click_device._put_empty()
        self._event_long_click_id = long_click
        long_long_click = (states >> 6) & 0x03
        if long_long_click != self._event_long_long_click_id and self._event_long_long_click_id != -1:
            self._button_long_long_click_device._put_empty()
        self._event_long_long_click_id = long_long_click

        # ── 바닥 센서 ──
        floor = int(packet[18:20], 16)
        self._floor_device._put(floor)

        # ── 카드 패턴 ──
        states = int(packet[20:22], 16)
        pattern_id = (states >> 7) & 0x01
        if pattern_id != self._event_pattern_id and self._event_pattern_id != -1:
            first = (states >> 4) & 0x07
            second = (states >> 1) & 0x07
            value = int(str(first) + str(second))
            self._card_pattern_device._put(value)
        self._event_pattern_id = pattern_id

        # ── 버튼 입력 ──
        pressed = states & 0x01
        self._button_pressed_device._put(pressed)

        # ── 가속도 센서 ──
        acc_x = self._to_int8(int(packet[26:28], 16))
        acc_y = self._to_int8(int(packet[28:30], 16))
        acc_z = self._to_int8(int(packet[30:32], 16))
        self._acceleration_x_device._put(acc_x)
        self._acceleration_y_device._put(acc_y)
        self._acceleration_z_device._put(acc_z)
        
        # ── 기타 센서 ──
        temperature = self._to_int8(int(packet[32:34], 16))
        temperature = Utils.round(temperature / 2.0 + 24.0)
        self._temperature_device._put(temperature)

        signal_strength = self._to_int8(int(packet[34:36], 16))
        self._signal_strength_device._put(signal_strength)

        battery = int(packet[36:38], 16)
        battery = round(2.0 + battery / 100, 2)
        self._battery_device._put(battery)
        
        # ── 상태/이벤트 바이트 ──
        states = int(packet[38:40], 16)

        # WHEEL MOVE 완료 이벤트
        move_id = (states >> 4) & 0x01
        if move_id != self._event_move_id:
            if move_id == 0 and self._event_move_id == 1:
                self._wheel_state_device._put_empty()
            self._event_move_id = move_id

        # LINE_TRACER 완료 이벤트
        trace_id = (states >> 2) & 0x03
        if trace_id != self._event_trace_id:
            if trace_id == 2 and self._event_trace_id == 3:
                self._line_tracer_state_device._put_empty()
            self._event_trace_id = trace_id

        # SOUND CLIP 완료 이벤트
        clip_id = states & 0x01
        if clip_id != self._event_clip_id:
            if clip_id == 0 and self._event_clip_id == 1:
                self._sound_state_device._put_empty()
            self._event_clip_id = clip_id
        self._sound_playing_device._put(clip_id)

        self._packet_received = 1
        return True

    def _encode_motoring_packet(self, address):
        result = ""
        with self._thread_lock:
            result = "10"

            left_wheel = 0 if self._left_wheel == -128 else self._left_wheel * 10
            right_wheel = 0 if self._right_wheel == -128 else self._right_wheel * 10
            
            # ── 바퀴 속도 ──
            result += self._to_hex2(left_wheel)
            result += self._to_hex2(right_wheel)

            # ── 바퀴 이동(WHEEL_MOVE) : 완료 이벤트용 id/event latch ──
            move = self._wheel_move
            if self._wheel_move_written:
                if move != 0 or self._wheel_move_prev != 0:
                    self._wheel_id = (self._wheel_id % 255) + 1
                self._wheel_event = 1 if move > 0 else 0
                self._wheel_move_prev = move
                self._wheel_move_written = False
            result += self._to_hex(self._wheel_id)
            result += self._to_hex2(move)

            # ── 라인트레이싱 : 완료 이벤트용 id latch ──
            trace = self._line_tracer_mode
            if self._line_tracer_mode_written:
                if trace > 0:
                    self._line_tracer_id = (self._line_tracer_id % 255) + 1
                    self._line_tracer_event = 1
                else:
                    self._line_tracer_event = 0
                self._line_tracer_mode_written = False
            result += self._to_hex(((self._line_tracer_id & 0x01) << 7) | trace)

            # ── LED ──
            result += self._to_hex(self._head_led[0])
            result += self._to_hex(self._head_led[1])
            result += self._to_hex(self._head_led[2])
            
            result += "00"

            # ── 사운드 : clip > note > buzz 우선순위, 완료 이벤트용 flag latch ──
            if self._sound_written:
                if self._sound_clip > 0:
                    self._sound_id = (self._sound_id % 255) + 1
                    self._sound_event = 1
                else:
                    self._sound_event = 0
                self._sound_written = False
            if self._sound_clip > 0:
                result += "00" * 4
                result += self._to_hex(((self._sound_id & 0x01) << 7) | self._sound_clip)
            elif self._sound_note > 0:
                result += "00" * 3
                result += self._to_hex(self._sound_note)
                result += "00"
            else:
                result += self._to_hex3(Utils.round(self._sound_buzz  * 100))
                result += "00" * 2

            # 라인트레이싱 speed / gain
            temp = (self._line_tracer_speed & 0x0f) << 4
            temp |= self._line_tracer_gain & 0x0f
            result += self._to_hex(temp)

            result += "00"

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
