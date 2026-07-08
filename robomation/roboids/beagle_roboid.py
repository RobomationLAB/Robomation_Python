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
import struct

from robomation.core.runner import Runner
from robomation.core.model import DeviceType, DataType, Roboid
from robomation.core.utils import Utils
from robomation.core.serial_connector import NusSerialConnector, Result
from robomation.roboids.beagle import Beagle  

class BeagleConnectionChecker(object):
    def __init__(self, roboid):
        self._roboid = roboid

    def check(self, info):
        return info[2] == "14"

class BeagleRoboid(Roboid):
    def __init__(self, index):
        super(BeagleRoboid, self).__init__(Beagle.ID, "Beagle", 0x01400000)
        self._index = index
        self._connector = None
        self._ready = False
        self._thread = None
        self._thread_lock = threading.Lock()

        # ── Motoring 상태 (effector: 매 사이클 read / command: _is_written latch) ──
        self._motor_sleep = 0
        self._motor_acceleration = 5
        self._left_wheel = 0
        self._right_wheel = 0
        self._sound_buzz = 0

        self._servo_a_speed = 5
        self._servo_a_angle = 0
        self._servo_b_speed = 5
        self._servo_b_angle = 0
        self._servo_c_speed = 5
        self._servo_c_angle = 0

        self._gyroscope_odr_bw = 0x20
        self._gyroscope_range = 0x02
        self._accelerometer_odr_bw = 0x08
        self._accelerometer_range = 0x01
        self._magnetometer_odr = 0x20
        self._magnetometer_rep_xy = 0x17
        self._magnetometer_rep_z = 0x52

        self._lidar_mode = 0
        self._lidar_connect = 0
        self._lidar_quadrant = [[65535] * 90 for _ in range(4)]

        self._wheel_move = 0
        self._sound_note = 0
        self._sound_clip = 0

        self._wheel_move_written = False
        self._sound_written = False

        # ── encode/decode 완료-이벤트 추적용 내부 상태 (Runner.wait_until 연동) ──

        self._wheel_id = 0
        self._wheel_move_prev = -1
        self._wheel_event = 0
        self._wheel_state = 0
        self._wheel_count = 0
        
        self._sound_id = 0
        self._sound_event = 0
        self._sound_state = 0
        
        self._event_move_id = -1
        self._event_clip_id = -1
        
        self._packet_sent = 0
        self._packet_received = 0

        self._create_model()

    def _create_model(self):
        dict = self._device_dict = {}
        dict[Beagle.MOTOR_SLEEP] = self._motor_sleep_device = self._add_device(Beagle.MOTOR_SLEEP, "MotorSleep", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[Beagle.MOTOR_ACCELERATION] = self._motor_acceleration_device = self._add_device(Beagle.MOTOR_ACCELERATION, "MotorAcceleration", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 15, 5)
        dict[Beagle.LEFT_WHEEL] = self._left_wheel_device = self._add_device(Beagle.LEFT_WHEEL, "LeftWheel", DeviceType.EFFECTOR, DataType.INTEGER, 1, -128, 127, -128)
        dict[Beagle.RIGHT_WHEEL] = self._right_wheel_device = self._add_device(Beagle.RIGHT_WHEEL, "RightWheel", DeviceType.EFFECTOR, DataType.INTEGER, 1, -128, 127, -128)
        dict[Beagle.SOUND_BUZZ] = self._sound_buzz_device = self._add_device(Beagle.SOUND_BUZZ, "SoundBuzz", DeviceType.EFFECTOR, DataType.FLOAT, 1, 0, 6553.5, 0)
        
        dict[Beagle.SERVO_A_SPEED] = self._servo_a_speed_device = self._add_device(Beagle.SERVO_A_SPEED, "ServoASpeed", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 10, 5)
        dict[Beagle.SERVO_A_ANGLE] = self._servo_a_angle_device = self._add_device(Beagle.SERVO_A_ANGLE, "ServoAAngle", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 180, 0)
        dict[Beagle.SERVO_B_SPEED] = self._servo_b_speed_device = self._add_device(Beagle.SERVO_B_SPEED, "ServoBSpeed", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 10, 5)
        dict[Beagle.SERVO_B_ANGLE] = self._servo_b_angle_device = self._add_device(Beagle.SERVO_B_ANGLE, "ServoBAngle", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 180, 0)
        dict[Beagle.SERVO_C_SPEED] = self._servo_c_speed_device = self._add_device(Beagle.SERVO_C_SPEED, "ServoCSpeed", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 10, 5)
        dict[Beagle.SERVO_C_ANGLE] = self._servo_c_angle_device = self._add_device(Beagle.SERVO_C_ANGLE, "ServoCAngle", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 180, 0)
        
        dict[Beagle.GYROSCOPE_ODR_BW] = self._gyroscope_odr_bw_device = self._add_device(Beagle.GYROSCOPE_ODR_BW, "GyroscopeOdrBw", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 128, 0x20)
        dict[Beagle.GYROSCOPE_RANGE] = self._gyroscope_range_device = self._add_device(Beagle.GYROSCOPE_RANGE, "GyroscopeRange", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 16, 0x02)
        dict[Beagle.ACCELEROMETER_ODR_BW] = self._accelerometer_odr_bw_device = self._add_device(Beagle.ACCELEROMETER_ODR_BW, "AccelerometerOdrBw", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 128, 0x08)
        dict[Beagle.ACCELEROMETER_RANGE] = self._accelerometer_range_device = self._add_device(Beagle.ACCELEROMETER_RANGE, "AccelerometerRange", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 16, 0x01)
        dict[Beagle.MAGNETOMETER_ODR] = self._magnetometer_odr_device = self._add_device(Beagle.MAGNETOMETER_ODR, "MagnetometerOdr", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 128, 0x20)
        dict[Beagle.MAGNETOMETER_REP_XY] = self._magnetometer_rep_xy_device = self._add_device(Beagle.MAGNETOMETER_REP_XY, "MagnetometerRepXY", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 255, 0x17)  # 47 = 2 * 0x17 + 1
        dict[Beagle.MAGNETOMETER_REP_Z] = self._magnetometer_rep_z_device = self._add_device(Beagle.MAGNETOMETER_REP_Z, "MagnetometerRepZ", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 255, 0x52)  # 83 = 0x52 + 1

        dict[Beagle.LIDAR_MODE] = self._lidar_mode_device = self._add_device(Beagle.LIDAR_MODE, "LidarMode", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 2, 0)
        dict[Beagle.LIDAR_CONNECT] = self._lidar_connect_device = self._add_device(Beagle.LIDAR_CONNECT, "LidarConnect", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)

        dict[Beagle.WHEEL_MOVE] = self._wheel_move_device = self._add_device(Beagle.WHEEL_MOVE, "WheelMove", DeviceType.COMMAND, DataType.INTEGER, 1, 0, 65535, 0)
        dict[Beagle.SOUND_NOTE] = self._sound_note_device = self._add_device(Beagle.SOUND_NOTE, "SoundNote", DeviceType.COMMAND, DataType.INTEGER, 1, 0, 88, 0)
        dict[Beagle.SOUND_CLIP] = self._sound_clip_device = self._add_device(Beagle.SOUND_CLIP, "SoundClip", DeviceType.COMMAND, DataType.INTEGER, 1, 0, 53, 0)

        dict[Beagle.INDEX] = self._index_device = self._add_device(Beagle.INDEX, "Index", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 255, 0)
        dict[Beagle.TIMESTAMP] = self._timestamp_device = self._add_device(Beagle.TIMESTAMP, "Timestamp", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 65535, 0)
        dict[Beagle.WHEEL_COUNT] = self._wheel_count_device = self._add_device(Beagle.WHEEL_COUNT, "WheelCount", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 65535, 0)
        dict[Beagle.SOUND_PLAYING] = self._sound_playing_device = self._add_device(Beagle.SOUND_PLAYING, "SoundPlaying", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[Beagle.LEFT_ENCODER] = self._left_encoder_device = self._add_device(Beagle.LEFT_ENCODER, "LeftEncoder", DeviceType.SENSOR, DataType.INTEGER, 1, -214783648, 214783647, 0)
        dict[Beagle.RIGHT_ENCODER] = self._right_encoder_device = self._add_device(Beagle.RIGHT_ENCODER, "RightEncoder", DeviceType.SENSOR, DataType.INTEGER, 1, -214783648, 214783647, 0)

        dict[Beagle.GYROSCOPE_INDEX] = self._gyroscope_index_device = self._add_device(Beagle.GYROSCOPE_INDEX, "GyroscopeIndex", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 15, 0)
        dict[Beagle.GYROSCOPE_X] = self._gyroscope_x_device = self._add_device(Beagle.GYROSCOPE_X, "GyroscopeX", DeviceType.SENSOR, DataType.FLOAT, 1, -250.0, 250.0, 0)
        dict[Beagle.GYROSCOPE_Y] = self._gyroscope_y_device = self._add_device(Beagle.GYROSCOPE_Y, "GyroscopeY", DeviceType.SENSOR, DataType.FLOAT, 1, -250.0, 250.0, 0)
        dict[Beagle.GYROSCOPE_Z] = self._gyroscope_z_device = self._add_device(Beagle.GYROSCOPE_Z, "GyroscopeZ", DeviceType.SENSOR, DataType.FLOAT, 1, -250.0, 250.0, 0)
        dict[Beagle.ACCELEROMETER_INDEX] = self._accelerometer_index_device = self._add_device(Beagle.ACCELEROMETER_INDEX, "AccelerometerIndex", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 15, 0)
        dict[Beagle.ACCELEROMETER_X] = self._accelerometer_x_device = self._add_device(Beagle.ACCELEROMETER_X, "AccelerometerX", DeviceType.SENSOR, DataType.FLOAT, 1, -2.0, 2.0, 0)
        dict[Beagle.ACCELEROMETER_Y] = self._accelerometer_y_device = self._add_device(Beagle.ACCELEROMETER_Y, "AccelerometerY", DeviceType.SENSOR, DataType.FLOAT, 1, -2.0, 2.0, 0)
        dict[Beagle.ACCELEROMETER_Z] = self._accelerometer_z_device = self._add_device(Beagle.ACCELEROMETER_Z, "AccelerometerZ", DeviceType.SENSOR, DataType.FLOAT, 1, -2.0, 2.0, 0)
        dict[Beagle.MAGNETOMETER_INDEX] = self._magnetometer_index_device = self._add_device(Beagle.MAGNETOMETER_INDEX, "MagnetometerIndex", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 15, 0)
        dict[Beagle.MAGNETOMETER_X] = self._magnetometer_x_device = self._add_device(Beagle.MAGNETOMETER_X, "MagnetometerX", DeviceType.SENSOR, DataType.FLOAT, 1, -100.0, 100.0, 0)
        dict[Beagle.MAGNETOMETER_Y] = self._magnetometer_y_device = self._add_device(Beagle.MAGNETOMETER_Y, "MagnetometerY", DeviceType.SENSOR, DataType.FLOAT, 1, -100.0, 100.0, 0)
        dict[Beagle.MAGNETOMETER_Z] = self._magnetometer_z_device = self._add_device(Beagle.MAGNETOMETER_Z, "MagnetometerZ", DeviceType.SENSOR, DataType.FLOAT, 1, -100.0, 100.0, 0)
        
        dict[Beagle.BATTERY] = self._battery_device = self._add_device(Beagle.BATTERY, "Battery", DeviceType.SENSOR, DataType.FLOAT, 1, 2.0, 5.0, 0)
        dict[Beagle.TEMPERATURE] = self._temperature_device = self._add_device(Beagle.TEMPERATURE, "Temperature", DeviceType.SENSOR, DataType.INTEGER, 1, -40, 88, 0)
        dict[Beagle.SIGNAL_STRENGTH] = self._signal_strength_device = self._add_device(Beagle.SIGNAL_STRENGTH, "SignalStrength", DeviceType.SENSOR, DataType.INTEGER, 1, -128, 0, 0)

        dict[Beagle.LIDAR_INDEX] = self._lidar_index_device = self._add_device(Beagle.LIDAR_INDEX, "LidarIndex", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 255, 0)
        dict[Beagle.LIDAR_READY] = self._lidar_ready_device = self._add_device(Beagle.LIDAR_READY, "LidarReady", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[Beagle.LIDAR_ARRAY] = self._lidar_array_device = self._add_device(Beagle.LIDAR_ARRAY, "LidarArray", DeviceType.SENSOR, DataType.INTEGER, 360, 0, 65535, 65535)
        dict[Beagle.LIDAR_DIRECTIONS] = self._lidar_directions_device = self._add_device(Beagle.LIDAR_DIRECTIONS, "LidarDirections", DeviceType.SENSOR, DataType.INTEGER, 8, 0, 65535, 65535)
        dict[Beagle.LIDAR_VALID] = self._lidar_valid_device = self._add_device(Beagle.LIDAR_VALID, "LidarValid", DeviceType.SENSOR, DataType.INTEGER, 4, 0, 1, 0)

        dict[Beagle.WHEEL_STATE] = self._wheel_state_device = self._add_device(Beagle.WHEEL_STATE, "WheelState", DeviceType.EVENT, DataType.INTEGER, 1, 0, 3, 0)
        dict[Beagle.SOUND_STATE] = self._sound_state_device = self._add_device(Beagle.SOUND_STATE, "SoundState", DeviceType.EVENT, DataType.INTEGER, 1, 0, 3, 0)

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

        tag = "Beagle[{}]".format(self._index)
        self._connector = NusSerialConnector(tag, BeagleConnectionChecker(self))
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
            super(BeagleRoboid, self)._dispose()
            self._release()
    
    def _reset(self):
        super(BeagleRoboid, self)._reset()

        # ── Motoring 상태 (effector: 매 사이클 read / command: _is_written latch) ──
        self._motor_sleep = 0
        self._motor_acceleration = 5
        self._left_wheel = 0
        self._right_wheel = 0
        self._sound_buzz = 0

        self._servo_a_speed = 5
        self._servo_a_angle = 0
        self._servo_b_speed = 5
        self._servo_b_angle = 0
        self._servo_c_speed = 5
        self._servo_c_angle = 0

        self._gyroscope_odr_bw = 0x20
        self._gyroscope_range = 0x02
        self._accelerometer_odr_bw = 0x08
        self._accelerometer_range = 0x01
        self._magnetometer_odr = 0x20
        self._magnetometer_rep_xy = 0x17
        self._magnetometer_rep_z = 0x52

        self._lidar_mode = 0
        self._lidar_connect = 0
        self._lidar_quadrant = [[65535] * 90 for _ in range(4)]

        self._wheel_move = 0
        self._sound_note = 0
        self._sound_clip = 0

        self._wheel_move_written = False
        self._sound_written = False

        # ── encode/decode 완료-이벤트 추적용 내부 상태 (Runner.wait_until 연동) ──
        self._wheel_id = 0
        self._wheel_move_prev = -1
        self._wheel_event = 0
        self._wheel_state = 0
        self._wheel_count = 0
        
        self._sound_id = 0
        self._sound_event = 0
        self._sound_state = 0
        
        self._event_move_id = -1
        self._event_clip_id = -1

        self._packet_sent = 0
        self._packet_received = 0

    def _request_motoring_data(self):
        with self._thread_lock:
            # EFFECTOR — 매 사이클 현재 값 반영
            self._motor_sleep = self._motor_sleep_device.read()
            self._motor_acceleration = self._motor_acceleration_device.read()
            self._left_wheel = self._left_wheel_device.read()
            self._right_wheel = self._right_wheel_device.read()
            self._sound_buzz = self._sound_buzz_device.read()

            self._servo_a_speed = self._servo_a_speed_device.read()
            self._servo_a_angle = self._servo_a_angle_device.read()
            self._servo_b_speed = self._servo_b_speed_device.read()
            self._servo_b_angle = self._servo_b_angle_device.read()
            self._servo_c_speed = self._servo_c_speed_device.read()
            self._servo_c_angle = self._servo_c_angle_device.read()

            self._gyroscope_odr_bw = self._gyroscope_odr_bw_device.read()
            self._gyroscope_range = self._gyroscope_range_device.read()
            self._accelerometer_odr_bw = self._accelerometer_odr_bw_device.read()
            self._accelerometer_range = self._accelerometer_range_device.read()
            self._magnetometer_odr = self._magnetometer_odr_device.read()
            self._magnetometer_rep_xy = self._magnetometer_rep_xy_device.read()
            self._magnetometer_rep_z = self._magnetometer_rep_z_device.read()

            self._lidar_mode = self._lidar_mode_device.read()
            self._lidar_connect = self._lidar_connect_device.read()
            
            # COMMAND — _is_written latch (한 번 쓰면 펌웨어로 전송)
            if self._wheel_move_device._is_written():
                self._wheel_move = self._wheel_move_device.read()
                self._wheel_move_written = True
            if self._sound_note_device._is_written():
                self._sound_note = self._sound_note_device.read()
                self._sound_written = True
            if self._sound_clip_device._is_written():
                self._sound_clip = self._sound_clip_device.read()
                self._sound_written = True
        self._clear_written()

    def _int32_to_float32(self, value):
        value = self._to_int32(value)
        return struct.unpack('>f', struct.pack('>i', value))[0]
    
    @staticmethod
    def _get_average_lidar(data, start, sz):
        sum = 0.0
        cnt = 0
        for i in range(start, start+sz):
            idx = i + len(data) if i < 0 else i
            val = data[idx]
            if val != 0xffff:
                sum += val
                cnt += 1
        return 0xffff if cnt == 0 else round(sum / cnt, 3)
    
    @staticmethod
    def _fill_checksum(packet, sz):
        suma = 0
        sumb = 0
        for i in range(sz):
            suma += packet[i]
            sumb += suma
        packet[sz] = suma & 0xff
        packet[sz+1] = sumb & 0xff

    def _decode_sensory_packet(self, packet):
        if len(packet) < 2: return False
        self._packet_received = 0
        
        if packet[0] == 0x10:   # RoboticsCar Data
            if packet[1] != 29: return False

            # index
            index = packet[2]
            self._index_device._put(index)

            # timestamp
            timestamp = packet[3] & 0xff
            timestamp |= (packet[4] & 0xff) << 8
            self._timestamp_device._put(timestamp)

            # left encoder
            value = packet[5] & 0xff
            value |= (packet[6] & 0xff) << 8
            value |= (packet[7] & 0xff) << 16
            value |= (packet[8] & 0xff) << 24
            self._left_encoder_device._put(self._to_int32(value))

            # right encoder
            value = packet[9] & 0xff
            value |= (packet[10] & 0xff) << 8
            value |= (packet[11] & 0xff) << 16
            value |= (packet[12] & 0xff) << 24
            self._right_encoder_device._put(self._to_int32(value))

            # temperature
            temperature = Utils.round(packet[13] / 2.0 + 23.0)
            self._temperature_device._put(temperature)

            # signal strength
            signal_strength = packet[14]
            self._signal_strength_device._put(signal_strength)

            # battery
            battery = round(2.0 + packet[15] / 100, 2)
            self._battery_device._put(battery)

            # ── 상태/이벤트 바이트 ──
            states = packet[17]

            motor_sleep = (states >> 3) & 0x01
            self._motor_sleep_device._put(motor_sleep)

            motor_acceleration = (states >> 4) & 0x0f
            self._motor_acceleration_device._put(motor_acceleration)

            # WHEEL MOVE 완료 이벤트
            move_id = (states >> 1) & 0x03
            if move_id != self._event_move_id and self._event_move_id != -1:
                self._wheel_state_device._put_empty()
            self._event_move_id = move_id

            # SOUND CLIP 완료 이벤트
            states = packet[18]
            clip_id = (states >> 1) & 0x03
            if clip_id != self._event_clip_id and self._event_clip_id != -1:
                self._sound_state_device._put_empty()
            self._event_clip_id = clip_id

            sound_playing = states & 0x01
            self._sound_playing_device._put(sound_playing)

            # Servo
            ''' 
            servo_a_speed = packet[19]
            servo_a_angle = max(packet[20], 180)
            servo_b_speed = packet[21]
            servo_b_angle = max(packet[22], 180)
            servo_c_speed = packet[23]
            servo_c_angle = max(packet[24], 180)
            self._servo_a_speed_device._put(servo_a_speed)
            self._servo_a_angle_device._put(servo_a_angle)
            self._servo_b_speed_device._put(servo_b_speed)
            self._servo_b_angle_device._put(servo_b_angle)
            self._servo_c_speed_device._put(servo_c_speed)
            self._servo_c_angle_device._put(servo_c_angle)
            '''

            self._packet_received = 1
        elif packet[0] == 0x20:   # IMU Data
            if packet[1] < 38 or len(packet) < packet[1] + 2: return False

            # index
            index = packet[2]
            self._index_device._put(index)

            # timestamp
            timestamp = packet[3] & 0xff
            timestamp |= (packet[4] & 0xff) << 8
            self._timestamp_device._put(timestamp)

            # Gyroscope / Accelerometer / Magnetometer Option
            '''
            gyrosocpe_odr_bw = packet[5] or 0x20
            gyroscope_range = packet[6] or 0x02
            accelerometer_odr_bw = packet[7] or 0x08
            accelerometer_range = packet[8] or 0x01
            magnetometer_odr = packet[9] or 0x20
            magnetometer_rep_xy = packet[10] or 0x17
            magnetometer_rep_z = packet[11] or 0x52
            self._gyroscope_odr_bw_device._put(gyrosocpe_odr_bw)
            self._gyroscope_range_device._put(gyroscope_range)
            self._accelerometer_odr_bw_device._put(accelerometer_odr_bw)
            self._accelerometer_range_device._put(accelerometer_range)
            self._magnetometer_odr_device._put(magnetometer_odr)
            self._magnetometer_rep_xy_device._put(magnetometer_rep_xy)
            self._magnetometer_rep_z_device._put(magnetometer_rep_z)
            '''
            
            # Gyroscope / Accelerometer / Magnetometer Value
            cur = 12
            while cur < packet[1] + 2:
                type = (packet[cur] >> 4) & 0x0f
                index = packet[cur] & 0x0f
                if type == 0:       # none
                    break
                if type == 1:       # Gyroscope
                    self._gyroscope_index_device._put(index)
                    value = packet[cur+1] & 0xff
                    value |= (packet[cur+2] & 0xff) << 8
                    value = round(self._to_int16(value) * 250 / 32768, 6)
                    self._gyroscope_x_device._put(value)
                    value = packet[cur+3] & 0xff
                    value |= (packet[cur+4] & 0xff) << 8
                    value = round(self._to_int16(value) * 250 / 32768, 6)
                    self._gyroscope_y_device._put(value)
                    value = packet[cur+5] & 0xff
                    value |= (packet[cur+6] & 0xff) << 8
                    value = round(self._to_int16(value) * 250 / 32768, 6)
                    self._gyroscope_z_device._put(value)
                    cur += 7
                elif type == 2:     # Accelerometer
                    self._accelerometer_index_device._put(index)
                    value = packet[cur+1] & 0xff
                    value |= (packet[cur+2] & 0xff) << 8
                    value = round(self._to_int16(value) / -1024, 6)
                    self._accelerometer_x_device._put(value)
                    value = packet[cur+3] & 0xff
                    value |= (packet[cur+4] & 0xff) << 8
                    value = round(self._to_int16(value) / -1024, 6)
                    self._accelerometer_y_device._put(value)
                    value = packet[cur+5] & 0xff
                    value |= (packet[cur+6] & 0xff) << 8
                    value = round(self._to_int16(value) / -1024, 6)
                    self._accelerometer_z_device._put(value)
                    cur += 8
                elif type == 3:     # Magnetometer
                    self._magnetometer_index_device._put(index)
                    value = packet[cur+1] & 0xff
                    value |= (packet[cur+2] & 0xff) << 8
                    value |= (packet[cur+3] & 0xff) << 16
                    value |= (packet[cur+4] & 0xff) << 24
                    value = round(self._int32_to_float32(value), 6)
                    self._magnetometer_x_device._put(value)
                    value = packet[cur+5] & 0xff
                    value |= (packet[cur+6] & 0xff) << 8
                    value |= (packet[cur+7] & 0xff) << 16
                    value |= (packet[cur+8] & 0xff) << 24
                    value = round(self._int32_to_float32(value), 6)
                    self._magnetometer_y_device._put(value)
                    value = packet[cur+9] & 0xff
                    value |= (packet[cur+10] & 0xff) << 8
                    value |= (packet[cur+11] & 0xff) << 16
                    value |= (packet[cur+12] & 0xff) << 24
                    value = round(self._int32_to_float32(value), 6)
                    self._magnetometer_z_device._put(value)
                    cur += 13
            self._packet_received = 2
        elif packet[0] == 0x30:   # Lidar Data
            if packet[1] != 189: return False
            if packet[2] != 0xfe or packet[3] != 0x10: return False

            cur_lidar_index = self._lidar_index_device.read()
            new_lidar_index = (packet[4] & 0xff)
            if cur_lidar_index == new_lidar_index: return False

            lidar_ready = (packet[5] & 0x01) ^ 0x01
            quadrant = packet[6] & 0xff
            if quadrant < 1 or quadrant > 4: return False

            self._lidar_index_device._put(new_lidar_index)
            self._lidar_ready_device._put(lidar_ready)

            # resolution = (packet[7] & 0xFF) / 10.0

            sz = packet[9] & 0xff
            sz |= (packet[10] & 0xff) << 8
            arr = self._lidar_quadrant[quadrant-1]
            for i in range(sz):
                if 2 * i + 11 >= len(packet): break
                value = packet[2*i+11] & 0xff
                value |= (packet[2*i+12] & 0xff) << 8
                if value >= 0xff00 and value < 0xffff:
                    mode = self._lidar_mode_device.read()
                    if mode == 0: value = 0
                    elif mode == 1: value = 0x50
                    else: value = value & 0xff
                arr[i] = value

            self._lidar_quadrant[quadrant-1] = arr
            self._lidar_valid_device._put_at(quadrant-1, True)

            if quadrant == 4:
                lidar_valid = self._lidar_valid_device.read()
                valid = True
                for i in range(4):
                    values = self._lidar_quadrant[i]
                    if lidar_valid[i] == 0 or values[0] == 0 or values[0] == 0xFFFF:
                        valid = False
                    lidar_valid[i] = 0
                self._lidar_valid_device._put_array(lidar_valid)
                if valid:
                    self._lidar_ready_device._put(1)
                    for i in range(4):
                        values = self._lidar_quadrant[i]
                        for j, val in enumerate(values):
                            self._lidar_array_device._put_at(90*i+j, val)

                    arr = self._lidar_array_device.read()
                    length = len(arr)
                    if length > 16:
                        sz = length // 8
                        start = -(sz // 2)
                        for i in range(8):
                            avg = self._get_average_lidar(arr, start, sz)
                            self._lidar_directions_device._put_at(i, avg)
                            start += sz
            self._packet_received = 3
        return True

    def _encode_motoring_packet(self):
        if self._packet_sent == 0:      # RoboticsCar
            length = 22
            packet = [0] * (length+5)
            packet[0] = 0x52
            packet[1] = 0x49
            packet[2] = length
            packet[3] = 0x10
            packet[4] = length-2
            
            # Index
            index = self._index_device.read()
            packet[5] = (index % 255 + 1) & 0xff

            # Motor
            value = (self._wheel_id & 0x03) << 6
            value |= (self._motor_sleep & 0x01) << 5
            value |= (self._motor_acceleration & 0x0f)
            packet[6] = value

            # 바퀴 속도
            speed = 0 if self._left_wheel == -128 else self._left_wheel * 30
            packet[7] = speed & 0xff
            packet[8] = (speed >> 8) & 0xff

            speed = 0 if self._right_wheel == -128 else self._right_wheel * 30
            packet[9] = speed & 0xff
            packet[10] = (speed >> 8) & 0xff

            # ── 바퀴 이동(WHEEL_MOVE) : 완료 이벤트용 id/event latch ──
            move = self._wheel_move
            if self._wheel_move_written:
                if move != 0 or self._wheel_move_prev != 0:
                    self._wheel_id = (self._wheel_id % 255) + 1
                self._wheel_count = 0
                self._wheel_event = 1 if move > 0 else 0
                self._wheel_move_prev = move
                self._wheel_move_written = False
            packet[11] = self._wheel_id & 0xff
            packet[12] = move & 0xff
            packet[13] = (move >> 8) & 0xff

            # ── 사운드 : clip > note > buzz 우선순위, 완료 이벤트용 flag latch ──
            if self._sound_written:
                if self._sound_clip > 0:
                    self._sound_id = (self._sound_id % 255) + 1
                    self._sound_event = 1
                else:
                    self._sound_event = 0
                self._sound_written = False
            if self._sound_clip > 0:
                packet[18] = ((self._sound_id & 0x01) << 7) | self._sound_clip
            elif self._sound_note > 0:
                packet[17] = self._sound_note
            else:
                buzz = int(self._sound_buzz * 100)
                packet[14] = buzz & 0xff
                packet[15] = (buzz >> 8) & 0xff
                packet[16] = (buzz >> 16) & 0xff

            # Servo
            packet[19] = self._servo_a_speed or 5
            packet[20] = self._servo_a_angle
            packet[21] = self._servo_b_speed or 5
            packet[22] = self._servo_b_angle
            packet[23] = self._servo_c_speed or 5
            packet[24] = self._servo_c_angle

            self._fill_checksum(packet, length+3)
            self._packet_sent = 1
        elif self._packet_sent == 1:    # IMU
            length = 10
            packet = [0] * (length+5)
            packet[0] = 0x52
            packet[1] = 0x49
            packet[2] = length
            packet[3] = 0x20
            packet[4] = length-2
            
            gyroscope_index = self._gyroscope_index_device.read()
            accelerometer_index = self._accelerometer_index_device.read()
            magnetometer_index = self._magnetometer_index_device.read()

            # Index
            index = (gyroscope_index + 1) & 0x03
            index |= ((accelerometer_index + 1) & 0x03) << 2
            index |= ((magnetometer_index + 1) & 0x03) << 4
            packet[5] = index

            # Gyroscope
            packet[6] = self._gyroscope_odr_bw & 0xff
            packet[7] = self._gyroscope_range & 0xff

            # Accelerometer
            packet[8] = self._accelerometer_odr_bw & 0xff
            packet[9] = self._accelerometer_range & 0xff

            # Magnetometer
            packet[10] = self._magnetometer_odr & 0xff
            packet[11] = self._magnetometer_rep_xy & 0xff
            packet[12] = self._magnetometer_rep_z & 0xff

            self._fill_checksum(packet, length+3)
            self._packet_sent = 2
        elif self._packet_sent == 2:    # Lidar
            length = 6
            packet = [0] * (length+5)
            packet[0] = 0x52
            packet[1] = 0x49
            packet[2] = length
            packet[3] = 0x30
            packet[4] = length-2
            
            # Lidar Command 헤더 & 메시지 ID
            packet[5] = 0xfd
            packet[6] = 0xe0
            
            # Index
            index = self._index_device.read()
            packet[7] = (index % 255 + 1) & 0xff

            connect = self._lidar_connect
            packet[8] = 0x0f if connect else 0xf0

            self._fill_checksum(packet, length+3)
            self._packet_sent = 0
        return packet

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
            packet = self._encode_motoring_packet()
            connector.write(packet)
