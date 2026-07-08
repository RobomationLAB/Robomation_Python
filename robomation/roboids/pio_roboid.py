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
from robomation.roboids.pio import Pio  

class PioConnectionChecker(object):
    def __init__(self, roboid):
        self._roboid = roboid

    def check(self, info):
        return info[2] == "20"

class PioRoboid(Roboid):
    def __init__(self, index):
        super(PioRoboid, self).__init__(Pio.ID, "Pio", 0x02000000)
        self._index = index
        self._connector = None
        self._ready = False
        self._thread = None
        self._thread_lock = threading.Lock()

        # ── Motoring 상태 (effector: 매 사이클 read / command: _is_written latch) ──
        self._turbo = 0
        self._left_wheel = 0
        self._right_wheel = 0
        self._neck_speed = 0
        self._left_rgb = [0, 0, 0]
        self._left_color = 0
        self._left_brightness = 100
        self._right_rgb = [0, 0, 0]
        self._right_color = 0
        self._right_brightness = 100
        self._left_pattern_color = 0
        self._right_pattern_color = 0
        self._sound_buzz = 0

        self._wheel_move = 0
        self._neck_angle = 0
        self._eye_pattern = 0
        self._sound_note = 0
        self._sound_clip = 0
        self._sound_melody = 0
        # self._behavior = 0

        self._wheel_move_written = False
        self._neck_angle_written = False
        self._eye_pattern_written = False
        self._sound_written = False
        # self._behavior_written = False

        # ── encode/decode 완료-이벤트 추적용 내부 상태 (Runner.wait_until 연동) ──
        self._wheel_id = 0
        self._wheel_move_prev = -1
        self._wheel_event = 0
        self._wheel_state = 0
        self._wheel_count = 0

        self._neck_id = 0
        self._neck_angle_prev = -1
        self._neck_event = 0
        self._neck_state = 0
        self._neck_count = 0

        self._eye_pattern_id = 0
        self._eye_pattern_event = 0
        self._eye_pattern_state = 0
        
        self._sound_id = 0
        self._sound_event = 0
        self._sound_state = 0

        # self._behavior_id = 0
        # self._behavior_event = 0
        # self._behavior_state = 0
        
        self._event_move_id = -1
        self._event_neck_id = -1
        self._event_eye_pattern_id = -1
        self._event_sound_id = -1
        self._event_keypad_id = -1
        # self._event_behavior_id = -1
        
        self._packet_sent = 0
        self._packet_received = 0

        self._create_model()

    def _create_model(self):
        dict = self._device_dict = {}
        dict[Pio.TURBO] = self._turbo_device = self._add_device(Pio.TURBO, "Turbo", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[Pio.LEFT_WHEEL] = self._left_wheel_device = self._add_device(Pio.LEFT_WHEEL, "LeftWheel", DeviceType.EFFECTOR, DataType.INTEGER, 1, -128, 127, -128)
        dict[Pio.RIGHT_WHEEL] = self._right_wheel_device = self._add_device(Pio.RIGHT_WHEEL, "RightWheel", DeviceType.EFFECTOR, DataType.INTEGER, 1, -128, 127, -128)
        dict[Pio.NECK_SPEED] = self._neck_speed_device = self._add_device(Pio.NECK_SPEED, "NeckSpeed", DeviceType.EFFECTOR, DataType.INTEGER, 1, -6, 6, 4)
        dict[Pio.LEFT_RGB] = self._left_rgb_device = self._add_device(Pio.LEFT_RGB, "LeftRgb", DeviceType.EFFECTOR, DataType.INTEGER, 3, 0, 255, 0)
        dict[Pio.LEFT_COLOR] = self._left_color_device = self._add_device(Pio.LEFT_COLOR, "LeftColor", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 7, 0)
        dict[Pio.LEFT_BRIGHTNESS] = self._left_brightness_device = self._add_device(Pio.LEFT_BRIGHTNESS, "LeftBrightness", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 100, 100)        
        dict[Pio.LEFT_PATTERN_COLOR] = self._left_pattern_color_device = self._add_device(Pio.LEFT_PATTERN_COLOR, "LeftPatternColor", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 7, 0)        
        dict[Pio.RIGHT_RGB] = self._right_rgb_device = self._add_device(Pio.RIGHT_RGB, "RightRgb", DeviceType.EFFECTOR, DataType.INTEGER, 3, 0, 255, 0)
        dict[Pio.RIGHT_COLOR] = self._right_color_device = self._add_device(Pio.RIGHT_COLOR, "RightColor", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 7, 0)
        dict[Pio.RIGHT_BRIGHTNESS] = self._right_brightness_device = self._add_device(Pio.RIGHT_BRIGHTNESS, "RightBrightness", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 100, 100)
        dict[Pio.RIGHT_PATTERN_COLOR] = self._right_pattern_color_device = self._add_device(Pio.RIGHT_PATTERN_COLOR, "RightPatternColor", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 7, 0)        
        dict[Pio.SOUND_BUZZ] = self._sound_buzz_device = self._add_device(Pio.SOUND_BUZZ, "SoundBuzz", DeviceType.EFFECTOR, DataType.FLOAT, 1, 0, 6553.5, 0)
        
        dict[Pio.WHEEL_MOVE] = self._wheel_move_device = self._add_device(Pio.WHEEL_MOVE, "WheelMove", DeviceType.COMMAND, DataType.INTEGER, 1, 0, 65535, 0)
        dict[Pio.NECK_ANGLE] = self._neck_angle_device = self._add_device(Pio.NECK_ANGLE, "NeckAngle", DeviceType.COMMAND, DataType.INTEGER, 1, -45, 45, 0)
        dict[Pio.EYE_PATTERN] = self._eye_pattern_device = self._add_device(Pio.EYE_PATTERN, "EyePattern", DeviceType.COMMAND, DataType.INTEGER, 1, 0, 3, 0)
        dict[Pio.SOUND_NOTE] = self._sound_note_device = self._add_device(Pio.SOUND_NOTE, "SoundNote", DeviceType.COMMAND, DataType.INTEGER, 1, 0, 88, 0)
        dict[Pio.SOUND_CLIP] = self._sound_clip_device = self._add_device(Pio.SOUND_CLIP, "SoundClip", DeviceType.COMMAND, DataType.INTEGER, 1, 0, 22, 0)
        dict[Pio.SOUND_MELODY] = self._sound_melody_device = self._add_device(Pio.SOUND_MELODY, "SoundMelody", DeviceType.COMMAND, DataType.INTEGER, 1, 0, 15, 0)
        # dict[Pio.BEHAVIOR] = self._behavior_device = self._add_device(Pio.BEHAVIOR, "Behavior", DeviceType.COMMAND, DataType.INTEGER, 1, 0, 5, 0)

        dict[Pio.WHEEL_COUNT] = self._wheel_count_device = self._add_device(Pio.WHEEL_COUNT, "WheelCount", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 65535, 0)
        dict[Pio.WHEEL_MOVING] = self._wheel_moving_device = self._add_device(Pio.WHEEL_MOVING, "WheelMoving", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[Pio.NECK_COUNT] = self._neck_count_device = self._add_device(Pio.NECK_COUNT, "NeckCount", DeviceType.SENSOR, DataType.INTEGER, 1, -45, 45, 0)
        dict[Pio.NECK_MOVING] = self._neck_moving_device = self._add_device(Pio.NECK_MOVING, "NeckMoving", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[Pio.SOUND_PLAYING] = self._sound_playing_device = self._add_device(Pio.SOUND_PLAYING, "SoundPlaying", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[Pio.KEYPAD] = self._keypad_device = self._add_device(Pio.KEYPAD, "Keypad", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 128, 0)
        dict[Pio.BATTERY] = self._battery_device = self._add_device(Pio.BATTERY, "Battery", DeviceType.SENSOR, DataType.FLOAT, 1, 2.0, 5.0, 0)
        dict[Pio.SIGNAL_STRENGTH] = self._signal_strength_device = self._add_device(Pio.SIGNAL_STRENGTH, "SignalStrength", DeviceType.SENSOR, DataType.INTEGER, 1, -128, 0, 0)
        # dict[Pio.CODES] = self._codes_device = self._add_device(Pio.CODES, "Codes", DeviceType.SENSOR, DataType.FLOAT, 16, 0, 15, 0)

        dict[Pio.WHEEL_STATE] = self._wheel_state_device = self._add_device(Pio.WHEEL_STATE, "WheelState", DeviceType.EVENT, DataType.INTEGER, 1, 0, 3, 0)
        dict[Pio.NECK_STATE] = self._neck_state_device = self._add_device(Pio.NECK_STATE, "NeckState", DeviceType.EVENT, DataType.INTEGER, 1, 0, 3, 0)
        dict[Pio.EYE_PATTERN_STATE] = self._eye_pattern_state_device = self._add_device(Pio.EYE_PATTERN_STATE, "EyePatternState", DeviceType.EVENT, DataType.INTEGER, 1, 0, 3, 0)
        dict[Pio.SOUND_STATE] = self._sound_state_device = self._add_device(Pio.SOUND_STATE, "SoundState", DeviceType.EVENT, DataType.INTEGER, 1, 0, 3, 0)
        dict[Pio.KEYPAD_STATE] = self._keypad_state_device = self._add_device(Pio.KEYPAD_STATE, "KeypadState", DeviceType.EVENT, DataType.INTEGER, 1, 0, 3, 0)
        # dict[Pio.BEHAVIOR_STATE] = self._behavior_state_device = self._add_device(Pio.BEHAVIOR_STATE, "BehaviorState", DeviceType.EVENT, DataType.INTEGER, 1, 0, 3, 0)        

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

        tag = "Pio[{}]".format(self._index)
        self._connector = SerialConnector(tag, PioConnectionChecker(self))
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
            super(PioRoboid, self)._dispose()
            self._release()
    
    def _reset(self):
        super(PioRoboid, self)._reset()

        # ── Motoring 상태 (effector: 매 사이클 read / command: _is_written latch) ──
        self._turbo = 0
        self._left_wheel = 0
        self._right_wheel = 0
        self._neck_speed = 0
        self._left_rgb = [0, 0, 0]
        self._left_color = 0
        self._left_brightness = 100
        self._right_rgb = [0, 0, 0]
        self._right_color = 0
        self._right_brightness = 100
        self._left_pattern_color = 0
        self._right_pattern_color = 0
        self._sound_buzz = 0

        self._wheel_move = 0
        self._neck_angle = 0
        self._eye_pattern = 0
        self._sound_note = 0
        self._sound_clip = 0
        self._sound_melody = 0
        # self._behavior = 0

        self._wheel_move_written = False
        self._neck_angle_written = False
        self._eye_pattern_written = False
        self._sound_written = False
        # self._behavior_written = False

        # ── encode/decode 완료-이벤트 추적용 내부 상태 (Runner.wait_until 연동) ──
        self._wheel_id = 0
        self._wheel_move_prev = -1
        self._wheel_event = 0
        self._wheel_state = 0
        self._wheel_count = 0

        self._neck_id = 0
        self._neck_angle_prev = -1
        self._neck_event = 0
        self._neck_state = 0
        self._neck_count = 0

        self._eye_pattern_id = 0
        self._eye_pattern_event = 0
        self._eye_pattern_state = 0
        
        self._sound_id = 0
        self._sound_event = 0
        self._sound_state = 0

        # self._behavior_id = 0
        # self._behavior_event = 0
        # self._behavior_state = 0
        
        self._event_move_id = -1
        self._event_neck_id = -1
        self._event_eye_pattern_id = -1
        self._event_sound_id = -1
        self._event_keypad_id = -1
        # self._event_behavior_id = -1

    def _request_motoring_data(self):
        with self._thread_lock:
            # EFFECTOR — 매 사이클 현재 값 반영
            self._turbo = self._turbo_device.read()
            self._left_wheel = self._left_wheel_device.read()
            self._right_wheel = self._right_wheel_device.read()
            self._left_rgb = [self._left_rgb_device.read(0), self._left_rgb_device.read(1), self._left_rgb_device.read(2)]
            self._left_color = self._left_color_device.read()
            self._left_brightness = self._left_brightness_device.read()
            self._left_pattern_color = self._left_pattern_color_device.read()
            self._right_rgb = [self._right_rgb_device.read(0), self._right_rgb_device.read(1), self._right_rgb_device.read(2)]
            self._right_color = self._right_color_device.read()
            self._right_brightness = self._right_brightness_device.read()
            self._right_pattern_color = self._right_pattern_color_device.read()
            self._neck_speed = self._neck_speed_device.read()
            self._sound_buzz = self._sound_buzz_device.read()
            
            # COMMAND — _is_written latch (한 번 쓰면 펌웨어로 전송)
            if self._wheel_move_device._is_written():
                self._wheel_move = self._wheel_move_device.read()
                self._wheel_move_written = True
            if self._neck_angle_device._is_written():
                self._neck_angle = self._neck_angle_device.read()
                self._neck_angle_written = True
            if self._eye_pattern_device._is_written():
                self._eye_pattern = self._eye_pattern_device.read()
                self._eye_pattern_written = True
            if self._sound_note_device._is_written():
                self._sound_note = self._sound_note_device.read()
                self._sound_written = True
            if self._sound_clip_device._is_written():
                self._sound_clip = self._sound_clip_device.read()
                self._sound_written = True
            if self._sound_melody_device._is_written():
                self._sound_melody = self._sound_melody_device.read()
                self._sound_written = True
            # if self._behavior_device._is_written():
            #     self._behavior = self._behavior_device.read()
            #     self._behavior_written = True
        self._clear_written()

    def _decode_sensory_packet(self, packet):
        packet = str(packet)
        self._packet_received = 0
        value = int(packet[0:1], 16)
        if value != 1:
            return False
        
        wheel_count = int(packet[4:6] + packet[2:4], 16)
        self._wheel_count_device._put(wheel_count)
        
        neck_count = self._to_int8(int(packet[6:8], 16))
        self._neck_count_device._put(neck_count)

        keypad = int(packet[8:10], 16)
        self._keypad_device._put(keypad)
        
        # ── 이벤트 바이트 ──
        states = int(packet[10:12], 16)

        # WHEEL MOVE 완료 이벤트
        move_id = (states >> 6) & 0x03
        if move_id != self._event_move_id and self._event_move_id != -1:
            self._wheel_state_device._put_empty()
        self._event_move_id = move_id

        # NECK ANGLE 완료 이벤트
        neck_id = (states >> 4) & 0x03
        if neck_id != self._event_neck_id and self._event_neck_id != -1:
            self._neck_state_device._put_empty()
        self._event_neck_id = neck_id

        # EYE PATTERN 완료 이벤트
        eye_pattern_id = (states >> 2) & 0x03
        if eye_pattern_id != self._event_eye_pattern_id and self._event_eye_pattern_id != -1:
            self._eye_pattern_state_device._put_empty()
        self._event_eye_pattern_id = eye_pattern_id

        # ── 이벤트 바이트 ──
        states = int(packet[12:14], 16)
        
        # MELODY / CLIP 완료 이벤트
        sound_id = (states >> 6) & 0x03
        if sound_id != self._event_sound_id and self._event_sound_id != -1:
            self._sound_state_device._put_empty()
        self._event_sound_id = sound_id

        # KEYPAD 완료 이벤트
        keypad_id = (states >> 4) & 0x03
        if keypad_id != self._event_keypad_id and self._event_keypad_id != -1:
            self._keypad_state_device._put_empty()
        self._event_keypad_id = keypad_id

        # ── 상태 바이트 ──
        states = int(packet[14:16], 16)

        wheel_moving = (states >> 3) & 0x01
        self._wheel_moving_device._put(wheel_moving)

        neck_moving = (states >> 2) & 0x01
        self._neck_moving_device._put(neck_moving)

        sound_playing = (states >> 1) & 0x01
        self._sound_playing_device._put(sound_playing)

        # codes = []
        # for i in range(0, 8):
        #     codes.append(int(packet[16+i*2 : 16+i*2+1], 16))
        #     codes.append(int(packet[16+i*2+1 : 16+(i+1)*2], 16))
        # self._codes_device._put(codes)

        # ── BEHAVIOR 이벤트 바이트 ──
        # states = int(packet[34:36], 16)

        # behavior_id = states & 0x03
        # if behavior_id != self._event_behavior_id and self._event_behavior_id != -1:
        #     self._behavior_state_device._put_empty()
        # self._event_behavior_id = behavior_id

        battery = int(packet[36:38], 16)
        battery = round(2.0 + battery / 100, 2)
        self._battery_device._put(battery)

        signal_strength = self._to_int8(int(packet[38:40], 16))
        self._signal_strength_device._put(signal_strength)

        self._packet_received = 1
        return True

    def _encode_motoring_packet(self, address):
        result = ""
        with self._thread_lock:
            result = "10"
            if self._turbo == 1:
                result = "11"

            # ── 바퀴 속도 & 이동 ──
            move = self._wheel_move
            if self._wheel_move_written:
                if move != 0 or self._wheel_move_prev != 0:
                    self._wheel_id = (self._wheel_id % 255) + 1
                self._wheel_count = 0
                self._wheel_event = 1 if move > 0 else 0
                self._wheel_move_prev = move
                self._wheel_move_written = False
            
            temp = self._wheel_id & 0x0f
            if self._wheel_event != 0:
                temp |= 1 << 4
            result += self._to_hex(temp)
        
            left_wheel = 0 if self._left_wheel == -128 else self._left_wheel
            right_wheel = 0 if self._right_wheel == -128 else self._right_wheel
            result += self._to_hex(left_wheel)
            result += self._to_hex(right_wheel)
            result += self._to_hex2(move)

            # ── 목 속도 & 회전 ──
            if self._neck_angle_written:
                self._neck_id = (self._neck_id % 15) + 1
                self._neck_event = 1
                self._neck_angle_written = False
            
            temp = self._neck_id & 0x0f
            if self._neck_event != 0:
                temp |= 1 << 4
            result += self._to_hex(temp)
            result += self._to_hex(self._neck_speed)
            result += self._to_hex(self._neck_angle)

            # ── 눈 패턴 ──
            if self._eye_pattern_written:
                self._eye_pattern_id = (self._eye_pattern_id % 3) + 1
                self._eye_pattern_event = 1
                self._eye_pattern_written = False
            if self._eye_pattern_event != 0:
                temp = self._eye_pattern & 0x0f
                temp |= (8 + (self._eye_pattern_id & 0x03)) << 4
                result += self._to_hex(temp)
                result += self._to_hex(self._left_pattern_color)
                result += "00" * 2
                result += self._to_hex(self._right_pattern_color)
                result += "00" * 2
            else:
                left_rgb = self._left_rgb
                left_color = self._left_color
                left_brightness = self._left_brightness
                right_rgb = self._right_rgb
                right_color = self._right_color
                right_brightness = self._right_brightness

                temp = 0 if left_color == 0 else (1 << 6)
                temp += 0 if right_color == 0 else (1 << 2)
                result += self._to_hex(temp)

                # left eye data
                if left_color == 0:
                    result += self._to_hex(left_rgb[0])
                    result += self._to_hex(left_rgb[1])
                    result += self._to_hex(left_rgb[2])
                else:
                    result += self._to_hex(left_color)
                    result += self._to_hex(left_brightness)
                    result += "00"

                # right eye data
                if right_color == 0:
                    result += self._to_hex(right_rgb[0])
                    result += self._to_hex(right_rgb[1])
                    result += self._to_hex(right_rgb[2])
                else:
                    result += self._to_hex(right_color)
                    result += self._to_hex(right_brightness)
                    result += "00"

            result += "00"
            # temp = (self._behavior << 2) + self._behavior_id & 0x03
            # result += self._to_hex(temp)

            # ── 사운드 : melody > clip > note > buzz 우선순위, 완료 이벤트용 flag latch ──
            if self._sound_written:
                if self._sound_melody > 0 or self._sound_clip > 0 or self._sound_note > 0:
                    self._sound_id = (self._sound_id % 15) + 1
                    self._sound_event = 1
                else:
                    self._sound_event = 0
                self._sound_written = False
            if self._sound_melody > 0:
                result += self._to_hex(0x30 | self._sound_id)
                result += self._to_hex(self._sound_melody)
                result += "00"
            elif self._sound_clip > 0:
                result += self._to_hex(0x20 | self._sound_id)
                result += self._to_hex(self._sound_clip)
                result += "00"
            elif self._sound_note > 0:
                result += self._to_hex(0x10 | self._sound_id)
                result += self._to_hex(self._sound_note)
                result += "00"
            else:
                result += "00"
                result += self._to_hex2(Utils.round(self._sound_buzz * 10))

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
