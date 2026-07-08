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
# from robomation.core.utils import Utils
from robomation.core.serial_connector import SerialConnector, Result
from robomation.roboids.raccoonbot import RaccoonBot

class RaccoonBotConnectionChecker(object):
    def __init__(self, roboid):
        self._roboid = roboid

    def check(self, info):
        return info[2] in ("30", "3F")

class RaccoonBotRoboid(Roboid):
    def __init__(self, index):
        super(RaccoonBotRoboid, self).__init__(RaccoonBot.ID, "RaccoonBot", 0x03000000)
        self._index = index
        self._connector = None
        self._ready = False
        self._thread = None
        self._thread_lock = threading.Lock()

        # Multiplexer (Default / Peripheral / End Effector)
        self._mux = 0

        # ── Motoring 상태 ──
        # EFFECTOR (매 사이클 read)
        self._mode = 0
        self._motor_off = [0, 0, 0, 0]
        self._speed = [0, 0, 0, 0]
        self._angle_max_speed = 70
        self._end_effector_lock = 0
        self._end_effector_control = 0
        self._conveyor_mode = 0
        self._conveyor_speed = 0

        # COMMAND (_is_written latch)
        self._angle_j1 = 0.0
        self._angle_j2 = 0.0
        self._angle_j3 = 0.0
        self._angle_j4 = 0.0
        self._sound_note = 0
        self._sound_clip = 0
        self._conveyor_distance = 0

        self._angle_j1_written = False
        self._angle_j2_written = False
        self._angle_j3_written = False
        self._angle_j4_written = False
        self._sound_note_written = False
        self._sound_clip_written = False
        self._conveyor_distance_written = False

        # ── encode/decode 완료-이벤트 추적용 내부 상태 (Runner.wait_until 연동) ──
        # ANGLE_STATE / SOUND_STATE / CONVEYOR_STATE 와 짝지어 사용
        self._angle_id = 0
        self._angle_event = 0
        self._angle_state = 0
        self._angle_count = 0

        self._sound_id = 0
        self._sound_event = 0
        self._sound_state = 0

        self._conveyor_id = 0
        self._conveyor_distance_prev = -1
        self._conveyor_event = 0
        self._conveyor_state = 0
        self._conveyor_count = 0

        self._event_angle_id = -1
        self._event_clip_id = -1
        self._event_teach_click_id = -1
        self._event_play_click_id = -1
        self._event_power_click_id = -1
        self._event_delete_click_id = -1
        self._event_teach_long_click_id = -1
        self._event_play_long_click_id = -1
        self._event_delete_long_click_id = -1
        self._event_conveyor_id = -1
        self._event_conveyor_button_click_id = -1
        self._event_conveyor_button_long_click_id = -1

        self._packet_sent = 0
        self._packet_received = 0

        self._create_model()
        
        default_angle = RaccoonBot._DEFAULT_ANGLE['home']
        self._angle_j1_device.write(default_angle[0])
        self._angle_j2_device.write(default_angle[1])
        self._angle_j3_device.write(default_angle[2])
        self._angle_j4_device.write(default_angle[3])

    def _create_model(self):
        dict = self._device_dict = {}
        dict[RaccoonBot.MODE] = self._mode_device = self._add_device(RaccoonBot.MODE, "Mode", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[RaccoonBot.MOTOR_OFF] = self._motor_off_device = self._add_device(RaccoonBot.MOTOR_OFF, "MotorOff", DeviceType.EFFECTOR, DataType.INTEGER, 4, 0, 1, 0)
        dict[RaccoonBot.SPEED] = self._speed_device = self._add_device(RaccoonBot.SPEED, "Speed", DeviceType.EFFECTOR, DataType.INTEGER, 4, -100, 100, 0)
        dict[RaccoonBot.ANGLE_MAX_SPEED] = self._angle_max_speed_device = self._add_device(RaccoonBot.ANGLE_MAX_SPEED, "AngleMaxSpeed", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 100, 70)
        dict[RaccoonBot.END_EFFECTOR_LOCK] = self._end_effector_lock_device = self._add_device(RaccoonBot.END_EFFECTOR_LOCK, "EndEffectorLock", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 2, 0)
        dict[RaccoonBot.END_EFFECTOR_CONTROL] = self._end_effector_control_device = self._add_device(RaccoonBot.END_EFFECTOR_CONTROL, "EndEffectorControl", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[RaccoonBot.CONVEYOR_MODE] = self._conveyor_mode_device = self._add_device(RaccoonBot.CONVEYOR_MODE, "ConveyorMode", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[RaccoonBot.CONVEYOR_SPEED] = self._conveyor_speed_device = self._add_device(RaccoonBot.CONVEYOR_SPEED, "ConveyorSpeed", DeviceType.EFFECTOR, DataType.INTEGER, 1, -100, 100, 0)
        dict[RaccoonBot.PERIPHERAL] = self._peripheral_device = self._add_device(RaccoonBot.PERIPHERAL, "Peripheral", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 2, 0)

        dict[RaccoonBot.ANGLE_J1] = self._angle_j1_device = self._add_device(RaccoonBot.ANGLE_J1, "AngleJ1", DeviceType.COMMAND, DataType.FLOAT, 1, -120, 120, 0)
        dict[RaccoonBot.ANGLE_J2] = self._angle_j2_device = self._add_device(RaccoonBot.ANGLE_J2, "AngleJ2", DeviceType.COMMAND, DataType.FLOAT, 1, -90, 30, 0)
        dict[RaccoonBot.ANGLE_J3] = self._angle_j3_device = self._add_device(RaccoonBot.ANGLE_J3, "AngleJ3", DeviceType.COMMAND, DataType.FLOAT, 1, -150, 0, 0)
        dict[RaccoonBot.ANGLE_J4] = self._angle_j4_device = self._add_device(RaccoonBot.ANGLE_J4, "AngleJ4", DeviceType.COMMAND, DataType.FLOAT, 1, -105, 105, 0)
        dict[RaccoonBot.SOUND_NOTE] = self._sound_note_device = self._add_device(RaccoonBot.SOUND_NOTE, "SoundNote", DeviceType.COMMAND, DataType.INTEGER, 1, 0, 88, 0)
        dict[RaccoonBot.SOUND_CLIP] = self._sound_clip_device = self._add_device(RaccoonBot.SOUND_CLIP, "SoundClip", DeviceType.COMMAND, DataType.INTEGER, 1, 0, 37, 0)
        dict[RaccoonBot.CONVEYOR_DISTANCE] = self._conveyor_distance_device = self._add_device(RaccoonBot.CONVEYOR_DISTANCE, "ConveyorDistance", DeviceType.COMMAND, DataType.INTEGER, 1, -32768, 32767, 0)

        dict[RaccoonBot.ENCODER_J1] = self._encoder_j1_device = self._add_device(RaccoonBot.ENCODER_J1, "EncoderJ1", DeviceType.SENSOR, DataType.FLOAT, 1, -120, 120, 0)
        dict[RaccoonBot.ENCODER_J2] = self._encoder_j2_device = self._add_device(RaccoonBot.ENCODER_J2, "EncoderJ2", DeviceType.SENSOR, DataType.FLOAT, 1, -90, 30, 0)
        dict[RaccoonBot.ENCODER_J3] = self._encoder_j3_device = self._add_device(RaccoonBot.ENCODER_J3, "EncoderJ3", DeviceType.SENSOR, DataType.FLOAT, 1, -150, 0, 0)
        dict[RaccoonBot.ENCODER_J4] = self._encoder_j4_device = self._add_device(RaccoonBot.ENCODER_J4, "EncoderJ4", DeviceType.SENSOR, DataType.FLOAT, 1, -105, 105, 0)
        dict[RaccoonBot.END_EFFECTOR_DEVICE] = self._end_effector_device_device = self._add_device(RaccoonBot.END_EFFECTOR_DEVICE, "EndEffectorDevice", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 4, 0)
        dict[RaccoonBot.END_EFFECTOR_STATE] = self._end_effector_state_device = self._add_device(RaccoonBot.END_EFFECTOR_STATE, "EndEffectorState", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[RaccoonBot.TEACH_PRESSED] = self._teach_pressed_device = self._add_device(RaccoonBot.TEACH_PRESSED, "TeachPressed", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[RaccoonBot.PLAY_PRESSED] = self._play_pressed_device = self._add_device(RaccoonBot.PLAY_PRESSED, "PlayPressed", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[RaccoonBot.POWER_PRESSED] = self._power_pressed_device = self._add_device(RaccoonBot.POWER_PRESSED, "PowerPressed", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[RaccoonBot.DELETE_PRESSED] = self._delete_pressed_device = self._add_device(RaccoonBot.DELETE_PRESSED, "DeletePressed", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[RaccoonBot.WARNING] = self._warning_device = self._add_device(RaccoonBot.WARNING, "Warning", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[RaccoonBot.SIGNAL_STRENGTH] = self._signal_strength_device = self._add_device(RaccoonBot.SIGNAL_STRENGTH, "SignalStrength", DeviceType.SENSOR, DataType.INTEGER, 1, -128, 0, 0)
        dict[RaccoonBot.BATTERY] = self._battery_device = self._add_device(RaccoonBot.BATTERY, "Battery", DeviceType.SENSOR, DataType.FLOAT, 1, 2.0, 5.0, 0)
        dict[RaccoonBot.CONVEYOR_COUNT] = self._conveyor_count_device = self._add_device(RaccoonBot.CONVEYOR_COUNT, "ConveyorCount", DeviceType.SENSOR, DataType.INTEGER, 1, -32768, 32767, 0)
        dict[RaccoonBot.CONVEYOR_DIRECTION] = self._conveyor_direction_device = self._add_device(RaccoonBot.CONVEYOR_DIRECTION, "ConveyorDirection", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[RaccoonBot.CONVEYOR_RUNNING] = self._conveyor_running_device = self._add_device(RaccoonBot.CONVEYOR_RUNNING, "ConveyorRunning", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[RaccoonBot.CONVEYOR_BUTTON_PRESSED] = self._conveyor_button_pressed_device = self._add_device(RaccoonBot.CONVEYOR_BUTTON_PRESSED, "ConveyorButtonPressed", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)
        
        dict[RaccoonBot.TEACH_CLICK] = self._teach_click_device = self._add_device(RaccoonBot.TEACH_CLICK, "TeachClick", DeviceType.EVENT, DataType.INTEGER, 1, 0, 255, 0)
        dict[RaccoonBot.PLAY_CLICK] = self._play_click_device = self._add_device(RaccoonBot.PLAY_CLICK, "PlayClick", DeviceType.EVENT, DataType.INTEGER, 1, 0, 255, 0)
        dict[RaccoonBot.POWER_CLICK] = self._power_click_device = self._add_device(RaccoonBot.POWER_CLICK, "PowerClick", DeviceType.EVENT, DataType.INTEGER, 1, 0, 255, 0)
        dict[RaccoonBot.DELETE_CLICK] = self._delete_click_device = self._add_device(RaccoonBot.DELETE_CLICK, "DeleteClick", DeviceType.EVENT, DataType.INTEGER, 1, 0, 255, 0)
        dict[RaccoonBot.TEACH_LONG_CLICK] = self._teach_long_click_device = self._add_device(RaccoonBot.TEACH_LONG_CLICK, "TeachLongLick", DeviceType.EVENT, DataType.INTEGER, 1, 0, 255, 0)
        dict[RaccoonBot.PLAY_LONG_CLICK] = self._play_longclick_device = self._add_device(RaccoonBot.PLAY_LONG_CLICK, "PlayLongClick", DeviceType.EVENT, DataType.INTEGER, 1, 0, 255, 0)
        dict[RaccoonBot.DELETE_LONG_CLICK] = self._delete_long_click_device = self._add_device(RaccoonBot.DELETE_LONG_CLICK, "DeleteLongClick", DeviceType.EVENT, DataType.INTEGER, 1, 0, 255, 0)
        dict[RaccoonBot.CONVEYOR_BUTTON_CLICK] = self._conveyor_button_click_device = self._add_device(RaccoonBot.CONVEYOR_BUTTON_CLICK, "ConveyorButtonClick", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 255, 0)
        dict[RaccoonBot.CONVEYOR_BUTTON_LONG_CLICK] = self._conveyor_button_long_click_device = self._add_device(RaccoonBot.CONVEYOR_BUTTON_LONG_CLICK, "ConveyorButtonLongClick", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 255, 0)
        
        dict[RaccoonBot.ANGLE_STATE] = self._angle_state_device = self._add_device(RaccoonBot.ANGLE_STATE, "AngleState", DeviceType.EVENT, DataType.INTEGER, 1, 0, 3, 0)
        dict[RaccoonBot.SOUND_STATE] = self._sound_state_device = self._add_device(RaccoonBot.SOUND_STATE, "SoundState", DeviceType.EVENT, DataType.INTEGER, 1, 0, 3, 0)
        dict[RaccoonBot.CONVEYOR_STATE] = self._conveyor_state_device = self._add_device(RaccoonBot.CONVEYOR_STATE, "ConveyorState", DeviceType.EVENT, DataType.INTEGER, 1, 0, 3, 0)

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

        tag = "RaccoonBot[{}]".format(self._index)
        self._connector = SerialConnector(tag, RaccoonBotConnectionChecker(self))
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
            super(RaccoonBotRoboid, self)._dispose()
            self._release()
    
    def _reset(self):
        super(RaccoonBotRoboid, self)._reset()

        # Multiplexer (Default / Peripheral / End Effector)
        self._mux = 0

        # ── Motoring 상태 ──
        self._mode = 0
        self._motor_off = [0, 0, 0, 0]
        self._speed = [0, 0, 0, 0]
        self._angle_max_speed = 70
        self._end_effector_lock = 0
        self._end_effector_control = 0
        self._conveyor_mode = 0
        self._conveyor_speed = 0

        self._angle_j1 = 0.0
        self._angle_j2 = 0.0
        self._angle_j3 = 0.0
        self._angle_j4 = 0.0
        self._sound_note = 0
        self._sound_clip = 0
        self._conveyor_distance = 0

        self._angle_j1_written = False
        self._angle_j2_written = False
        self._angle_j3_written = False
        self._angle_j4_written = False
        self._sound_note_written = False
        self._sound_clip_written = False
        self._conveyor_distance_written = False

        # ── encode/decode 완료-이벤트 추적용 내부 상태 ──
        self._angle_id = 0
        self._angle_event = 0
        self._angle_state = 0
        self._angle_count = 0

        self._sound_id = 0
        self._sound_event = 0
        self._sound_state = 0

        self._conveyor_id = 0
        self._conveyor_distance_prev = -1
        self._conveyor_event = 0
        self._conveyor_state = 0
        self._conveyor_count = 0

        self._event_angle_id = -1
        self._event_clip_id = -1
        self._event_teach_click_id = -1
        self._event_play_click_id = -1
        self._event_power_click_id = -1
        self._event_delete_click_id = -1
        self._event_teach_long_click_id = -1
        self._event_play_long_click_id = -1
        self._event_delete_long_click_id = -1
        self._event_conveyor_id = -1
        self._event_conveyor_button_click_id = -1
        self._event_conveyor_button_long_click_id = -1


    def _request_motoring_data(self):
        with self._thread_lock:
            # EFFECTOR — 매 사이클 현재 값 반영
            self._mode = self._mode_device.read()
            self._motor_off = [self._motor_off_device.read(i) for i in range(4)]
            self._speed = [self._speed_device.read(i) for i in range(4)]
            self._angle_max_speed = self._angle_max_speed_device.read()
            self._end_effector_lock = self._end_effector_lock_device.read()
            self._end_effector_control = self._end_effector_control_device.read()
            self._conveyor_mode = self._conveyor_mode_device.read()
            self._conveyor_speed = self._conveyor_speed_device.read()
            self._peripheral = self._peripheral_device.read()

            # COMMAND — _is_written latch (한 번 쓰면 펌웨어로 전송)
            if self._angle_j1_device._is_written():
                self._angle_j1 = self._angle_j1_device.read()
                self._angle_j1_written = True
            if self._angle_j2_device._is_written():
                self._angle_j2 = self._angle_j2_device.read()
                self._angle_j2_written = True
            if self._angle_j3_device._is_written():
                self._angle_j3 = self._angle_j3_device.read()
                self._angle_j3_written = True
            if self._angle_j4_device._is_written():
                self._angle_j4 = self._angle_j4_device.read()
                self._angle_j4_written = True
            if self._sound_note_device._is_written():
                self._sound_note = self._sound_note_device.read()
                self._sound_note_written = True
            if self._sound_clip_device._is_written():
                self._sound_clip = self._sound_clip_device.read()
                self._sound_clip_written = True
            if self._conveyor_distance_device._is_written():
                self._conveyor_distance = self._conveyor_distance_device.read()
                self._conveyor_distance_written = True
        self._clear_written()


    def _decode_sensory_packet(self, packet):
        packet = str(packet)
        self._packet_received = 0
        value = int(packet[0:1], 16)
        if value != 1:
            return False
        
        DEG = 360 / 4096
        encoder_j1 = self._to_int16(int(packet[2:6], 16))
        encoder_j1 = round(encoder_j1 * DEG, 3)
        self._encoder_j1_device._put(encoder_j1)
        encoder_j2 = self._to_int16(int(packet[6:10], 16))
        encoder_j2 = round(encoder_j2 * DEG, 3)
        self._encoder_j2_device._put(encoder_j2)
        encoder_j3 = self._to_int16(int(packet[10:14], 16))
        encoder_j3 = round(encoder_j3 * DEG, 3)
        self._encoder_j3_device._put(encoder_j3)
        encoder_j4 = self._to_int16(int(packet[14:18], 16))
        encoder_j4 = round(encoder_j4 * DEG, 3)
        self._encoder_j4_device._put(encoder_j4)
        
        # ── 상태/이벤트 바이트 ──
        states = int(packet[20:22], 16)
        
        warning = (states >> 7) & 0x01
        self._warning_device._put(warning)

        # ANGLE 완료 이벤트
        angle_id = (states >> 4) & 0x03
        if angle_id != self._event_angle_id and self._event_angle_id != -1:
            self._angle_state_device._put_empty()
        self._event_angle_id = angle_id

        # SOUND CLIP 완료 이벤트
        clip_id = (states >> 1) & 0x03
        if clip_id != self._event_clip_id and self._event_clip_id != -1:
            self._sound_state_device._put_empty()
        self._event_clip_id = clip_id

        # MultiPlexer
        mux = int(packet[24:25], 16)
        self._mux = mux

        if mux == 0:  # Default
            pressed = int(packet[26:28], 16)
            teach_pressed = pressed & 0x01
            self._teach_pressed_device._put(teach_pressed)
            play_pressed = (pressed >> 1) & 0x01
            self._play_pressed_device._put(play_pressed)
            power_pressed = (pressed >> 2) & 0x01
            self._power_pressed_device._put(power_pressed)
            delete_pressed = (pressed >> 3) & 0x01
            self._delete_pressed_device._put(delete_pressed)

            click = int(packet[28:30], 16)
            teach_click = click & 0x03
            if teach_click != self._event_teach_click_id and self._event_teach_click_id != -1:
                self._teach_click_device._put_empty()
            self._event_teach_click_id = teach_click
            play_click = (click >> 2) & 0x03
            if play_click != self._event_play_click_id and self._event_play_click_id != -1:
                self._play_click_device._put_empty()
            self._event_play_click_id = play_click
            power_click = (click >> 4) & 0x03
            if power_click != self._event_power_click_id and self._event_power_click_id != -1:
                self._power_click_device._put_empty()
            self._event_power_click_id = power_click
            delete_click = (click >> 6) & 0x03
            if delete_click != self._event_delete_click_id and self._event_delete_click_id != -1:
                self._delete_click_device._put_empty()
            self._event_delete_click_id = delete_click

            long_click = int(packet[30:32], 16)
            teach_long_click = long_click & 0x03
            if teach_long_click != self._event_teach_long_click_id and self._event_teach_long_click_id != -1:
                self._teach_long_click_device._put_empty()
            self._event_teach_long_click_id = teach_long_click
            play_long_click = (long_click >> 2) & 0x03
            if play_long_click != self._event_play_long_click_id and self._event_play_long_click_id != -1:
                self._play_long_click_device._put_empty()
            self._event_play_long_click_id = play_long_click
            delete_long_click = (long_click >> 6) & 0x03
            if delete_long_click != self._event_delete_long_click_id and self._event_delete_long_click_id != -1:
                self._delete_long_click_device._put_empty()
            self._event_delete_long_click_id = delete_long_click

            signal_strength = self._to_int8(int(packet[36:38], 16))
            self._signal_strength_device._put(signal_strength)

            battery = int(packet[38:40], 16)
            battery = round(2.0 + battery / 100, 2)
            self._battery_device._put(battery)

        elif mux == 1:  # Peripheral
            peripheral = int(packet[25:26], 16)  # Peripheral 종류 
            self._peripheral_device._put(peripheral)
            if peripheral == 1:  # Conveyor
                conveyor_count = int(packet[28:32], 16)
                self._conveyor_count_device._put(conveyor_count)

                # BUTTON 이벤트
                button = int(packet[32:34], 16)
                pressed = button & 0x01
                self._conveyor_button_pressed_device._put(pressed)
                click = (button >> 1) & 0x03
                if click != self._event_conveyor_button_click_id and self._event_conveyor_button_click_id != -1:
                    self._conveyor_button_click_device._put_empty()
                self._event_conveyor_button_click_id = click
                long_click = (button >> 3) & 0x03
                if long_click != self._event_conveyor_button_long_click_id and self._event_conveyor_button_long_click_id != -1:
                    self._conveyor_button_long_click_device._put_empty()
                self._event_conveyor_button_long_click_id = long_click

                # Conveyor 상태
                state = int(packet[34:36], 16)
                
                direction = (state >> 3) & 0x01
                self._conveyor_direction_device._put(direction)

                conveyor_id = (state >> 1) & 0x03
                if conveyor_id != self._event_conveyor_id and self._event_conveyor_id != -1:
                    self._conveyor_state_device._put_empty()
                self._event_conveyor_id = conveyor_id

                running = state & 0x01
                self._conveyor_running_device._put(running)
            elif peripheral == 2:  # Slider
                pass
            elif peripheral == 3:  # Carrier
                pass
        elif mux == 2:  # End Effector
            end_effector = int(packet[25:26], 16)
            self._end_effector_device_device._put(end_effector)
            if end_effector != 0:
                state = int(packet[28:30], 16)
                self._end_effector_state_device._put(state)
        else:   # none
            pass

        self._packet_received = 1
        return True


    def _encode_motoring_packet(self, address):
        result = ""
        with self._thread_lock:
            mode = self._mode
            lock = self._end_effector_lock

            if mode == 0:  # Speed Mode
                if lock == 0:  # none
                    result = "10"
                elif lock == 1:  # horizontal
                    result = "1A"
                elif lock == 2:  # vertical
                    result = "1B"
                for i in range(0, 4):
                    value = 127 if self._motor_off[i] else min(max(self._speed[i], -100), 100)
                    result += self._to_hex(value)
                result += '00' * 7
            elif mode == 1:  # Angle Mode
                if lock == 0:  # none
                    result = "11"
                elif lock == 1:  # horizontal
                    result = "1C"
                elif lock == 2:  # vertical
                    result = "1D"

                # ── joint angle limit clamp ──
                TO_ANGLE = 4096 / 360
                limits = RaccoonBot._JOINT['limits']
                a1 = min(max(self._angle_j1, limits['J1']['min']), limits['J1']['max'])
                a2 = min(max(self._angle_j2, limits['J2']['min']), limits['J2']['max'])
                a3 = min(max(self._angle_j3, limits['J3']['min']), limits['J3']['max'])
                a4 = min(max(self._angle_j4, limits['J4']['min']), limits['J4']['max'])

                # motor_off 면 2048 (mid-position), 아니면 4096/360 스케일 (음수는 _to_hex2 가 16-bit 2's complement 처리)
                p1 = 2048 if self._motor_off[0] else int(a1 * TO_ANGLE)
                p2 = 2048 if self._motor_off[1] else int(a2 * TO_ANGLE)
                p3 = 2048 if self._motor_off[2] else int(a3 * TO_ANGLE)
                p4 = 2048 if self._motor_off[3] else int(a4 * TO_ANGLE)

                # ── ANGLE 완료 이벤트 latch id : 4개 중 하나라도 새로 쓰였으면 증가 ──
                if self._angle_j1_written or self._angle_j2_written or self._angle_j3_written or self._angle_j4_written:
                    self._angle_id = (self._angle_id % 255) + 1
                    self._angle_count = 0
                    self._angle_event = 1
                    self._angle_j1_written = False
                    self._angle_j2_written = False
                    self._angle_j3_written = False
                    self._angle_j4_written = False

                # packet[1] = id, packet[2..9] = 4 angles (big-endian 16bit each)
                result += self._to_hex(self._angle_id)
                result += self._to_hex2(p1)
                result += self._to_hex2(p2)
                result += self._to_hex2(p3)
                result += self._to_hex2(p4)

                # packet[10] = max speed (모터 4개 전부 off → 127, max_speed=0 → default 70)
                max_speed = (
                    127 
                    if self._motor_off[0] and self._motor_off[1] and self._motor_off[2] and self._motor_off[3]
                    else self._angle_max_speed
                )
                result += self._to_hex(max_speed)
                result += '00'

            # Slot & Device Control
            mux = self._mux
            result += str(mux)

            if mux == 0:    # Default
                result += str(0)
                result += '00' * 4

                # ── SOUND 완료 이벤트 latch id : 둘 중 하나라도 쓰였으면 증가 ──
                if self._sound_note_written or self._sound_clip_written:
                    self._sound_id = (self._sound_id % 255) + 1
                result += self._to_hex(self._sound_id)
                
                sound_clip = self._sound_clip
                if self._sound_clip_written:
                    self._sound_event = 1 if sound_clip > 0 else 0
                if sound_clip > 0:
                    result += self._to_hex(sound_clip + 128)
                else:
                    result += self._to_hex(self._sound_note)
                self._sound_note_written = False
                self._sound_clip_written = False

                result += '00'
            elif mux == 1:  # Peripheral
                peripheral = self._peripheral_device.read()
                result += str(peripheral)
                if peripheral == 1:  # Conveyor                    
                    distance = self._conveyor_distance                    

                    result += self._to_hex(0 if distance == 0 else 1)
                    result += self._to_hex(self._conveyor_speed)
                    if self._conveyor_distance_written:
                        if distance != 0 or self._conveyor_distance_prev != 0:
                            self._conveyor_id = (self._conveyor_id % 255) + 1
                        self._conveyor_count = 0
                        self._conveyor_event = 1 if distance > 0 else 0
                        self._conveyor_distance_prev = distance
                        self._conveyor_distance_written = False
                    result += self._to_hex(self._conveyor_id)
                    result += self._to_hex2(self._conveyor_distance)
                    result += '00' * 2
                elif peripheral == 2:  # Slider
                    result += '00' * 8
                elif peripheral == 3:  # Carrier
                    result += '00' * 8
            elif mux == 2:  # End Effector
                end_effector = self._end_effector_device_device.read()
                result += str(end_effector)
                result += '00'
                result += self._to_hex(self._end_effector_control)
                result += '00' * 5
            elif mux == 3:  # None
                result += '00' * 8

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
