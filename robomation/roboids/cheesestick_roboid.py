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
from robomation.roboids.cheesestick import CheeseStick  

class _5x5Matrix:
    def __init__(self):
        self._data = {}

    def set(self, x, y, value):
        if y not in self._data:
            self._data[y] = {}
        self._data[y][x] = value
    
    def get(self, x, y):
        row = self._data.get(y)
        if row is None or not row.get(x):
            return 0
        return row[x]
    
    def clear(self):
        self._data = {}

class CheeseStickConnectionChecker(object):
    def __init__(self, roboid):
        self._roboid = roboid

    def check(self, info):
        return info[2] == "0D"

class CheeseStickRoboid(Roboid):
    def __init__(self, index):
        super(CheeseStickRoboid, self).__init__(CheeseStick.ID, "CheeseStick", 0x00d00000)
        self._index = index
        self._connector = None
        self._ready = False
        self._thread = None
        self._thread_lock = threading.Lock()

        # ── CheeseStick 디바이스 ──
        # ── Motoring 상태 (effector: 매 사이클 read / command: _is_written latch) ──
        # ── S 포트 ──
        self._s_pulse_detect = 0
        self._s_neo = 0
        
        self._sa_mode = 0
        self._sa_pull = 0
        self._sa_bgv = 0
        self._sa_src_range = [None] * 3  # min, median, max
        self._sa_dst_range = [None] * 3  # min, median, max
        self._sa_output_digital = 0
        self._sa_output_pwm = 0
        self._sa_output_servo = 0

        self._sb_mode = 0
        self._sb_pull = 0
        self._sb_bgv = 0
        self._sb_src_range = [None] * 3  # min, median, max
        self._sb_dst_range = [None] * 3  # min, median, max
        self._sb_output_digital = 0
        self._sb_output_pwm = 0
        self._sb_output_servo = 0

        self._sc_mode = 0
        self._sc_pull = 0
        self._sc_bgv = 0
        self._sc_src_range = [None] * 3  # min, median, max
        self._sc_dst_range = [None] * 3  # min, median, max
        self._sc_output_digital = 0
        self._sc_output_pwm = 0
        self._sc_output_servo = 0

        # ── L 포트 ──
        self._l_pulse_detect = 0

        self._la_mode = 0
        self._la_pull = 0
        self._la_bgv = 0
        self._la_src_range = [None] * 3  # min, median, max
        self._la_dst_range = [None] * 3  # min, median, max
        self._la_output_digital = 0
        self._la_output_pwm = 0
        self._la_output_servo = 0

        self._lb_mode = 0
        self._lb_pull = 0
        self._lb_bgv = 0
        self._lb_src_range = [None] * 3  # min, median, max
        self._lb_dst_range = [None] * 3  # min, median, max
        self._lb_output_digital = 0
        self._lb_output_pwm = 0
        self._lb_output_servo = 0

        self._lc_mode = 0
        self._lc_pull = 0
        self._lc_bgv = 0
        self._lc_src_range = [None] * 3  # min, median, max
        self._lc_dst_range = [None] * 3  # min, median, max
        self._lc_output_digital = 0
        self._lc_output_pwm = 0
        self._lc_output_servo = 0

        # ── M 포트 ──
        self._m_mode = 0
        self._m_driver = 0
        self._m_step_pps = 0

        self._mab_mode = 0
        self._mab_motor_a = 0
        self._mab_motor_b = 0
        self._mab_output_digital = 0
        self._mab_output_pwm = 0
        self._mab_output_servo = 0

        self._mcd_mode = 0
        self._mcd_motor_c = 0
        self._mcd_motor_d = 0
        self._mcd_output_digital = 0
        self._mcd_output_pwm = 0
        self._mcd_output_servo = 0

        self._sound_buzz = 0
        self._accel_g_range = 0
        self._accel_bandwidth = 0

        self._m_step_move = 0
        self._sound_note = 0
        self._sound_clip = 0

        self._m_step_move_written = False
        self._sound_written = False

        # ── encode/decode 완료-이벤트 추적용 내부 상태 (Runner.wait_until 연동) ──
        self._m_step_id = 0
        self._m_step_move_prev = -1
        self._m_step_event = 0
        self._m_step_state = 0
        self._m_step_count = 0
        
        self._sound_id = 0
        self._sound_event = 0
        self._sound_state = 0
        
        self._event_m_step_id = -1
        self._event_clip_id = -1
        self._event_sc_pulse_input_detect_id = -1
        self._event_lc_pulse_input_detect_id = -1
        self._event_tap_id = -1
        self._event_fall_id = -1
        
        # ── NeoPixel ──
        # ── Motoring 상태 (effector: 매 사이클 read / command: _is_written latch) ──
        self._neo_mode = 0
        self._neo_from = 1
        self._neo_to = 1
        self._neo_increment = 1
        self._neo_brightness = 100

        self._neo_red = 0
        self._neo_red_change = 0
        self._neo_green = 0
        self._neo_green_change = 0
        self._neo_blue = 0
        self._neo_blue_change = 0
        self._neo_white = 0
        self._neo_white_change = 0

        self._neo_pattern_mode = 0
        self._neo_pattern_block = 255
        self._neo_pattern_skip = 0
        self._neo_pattern_clear = 0

        self._neo_shift_mode = 0
        self._neo_shift_direction = 0
        self._neo_shift_pixel = 1

        self._neo_command = 0

        self._neo_command_written = False

        # ── encode/decode 완료-이벤트 추적용 내부 상태 (Runner.wait_until 연동) ──
        self._neo_command_id = 0
        
        # ── HAT 디바이스 ──
        self._hat = 0

        # ── HAT010 ──
        # ── Motoring 상태 (effector: 매 사이클 read / command: _is_written latch) ──
        self._hat010_matrix = _5x5Matrix()
        self._hat010_x = 0
        self._hat010_y = 0
        self._hat010_origin_x = 0
        self._hat010_origin_y = 0
        self._hat010_brightness = 0

        self._hat010_led = 0
        self._hat010_draw = [0] * 25
        self._hat010_clear = 0

        self._hat010_led_written = False
        self._hat010_draw_written = False
        self._hat010_clear_written = False

        # ── encode/decode 완료-이벤트 추적용 내부 상태 (Runner.wait_until 연동) ──
        self._hat010_led_id = 0
        self._hat010_draw_id = 0

        # ── HAT022 ──
        # ── Motoring 상태 (effector: 매 사이클 read / command: _is_written latch) ──
        # ── encode/decode 완료-이벤트 추적용 내부 상태 (Runner.wait_until 연동) ──

        # ── PID 디바이스 ──
        self._pid = 0

        # ── PID10 ──
        # ── Motoring 상태 (effector: 매 사이클 read / command: _is_written latch) ──
        # ── encode/decode 완료-이벤트 추적용 내부 상태 (Runner.wait_until 연동) ──

        # ── PID13 ──
        # ── Motoring 상태 (effector: 매 사이클 read / command: _is_written latch) ──
        # ── encode/decode 완료-이벤트 추적용 내부 상태 (Runner.wait_until 연동) ──

        # ── PID26 ──
        # ── Motoring 상태 (effector: 매 사이클 read / command: _is_written latch) ──
        # ── encode/decode 완료-이벤트 추적용 내부 상태 (Runner.wait_until 연동) ──

        # ── SUT 디바이스 ──
        # self._sut = 0

        self._packet_sent = 0
        self._packet_received = 0

        self._create_model()

    def _create_model(self):
        dict = self._device_dict = {}

        # ── CheeseStick Base : Effectors (S 포트) ─────────────────────────────
        dict[CheeseStick.S_PULSE_DETECT] = self._s_pulse_detect_device = self._add_device(CheeseStick.S_PULSE_DETECT, "SPulseDetect", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.S_NEO] = self._s_neo_device = self._add_device(CheeseStick.S_NEO, "SNeo", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)

        dict[CheeseStick.SA_MODE] = self._sa_mode_device = self._add_device(CheeseStick.SA_MODE, "SaMode", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 3, 0)
        dict[CheeseStick.SA_PULL] = self._sa_pull_device = self._add_device(CheeseStick.SA_PULL, "SaPull", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 2, 0)
        dict[CheeseStick.SA_BGV] = self._sa_bgv_device = self._add_device(CheeseStick.SA_BGV, "SaBgv", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.SA_SRC_RANGE] = self._sa_src_range_device = self._add_device(CheeseStick.SA_SRC_RANGE, "SaSrcRange", DeviceType.EFFECTOR, DataType.INTEGER, 3, 0, 255, 0)
        dict[CheeseStick.SA_DST_RANGE] = self._sa_dst_range_device = self._add_device(CheeseStick.SA_DST_RANGE, "SaDstRange", DeviceType.EFFECTOR, DataType.FLOAT, 3, -100.0, 100.0, 0)
        dict[CheeseStick.SA_OUTPUT_DIGITAL] = self._sa_output_digital_device = self._add_device(CheeseStick.SA_OUTPUT_DIGITAL, "SaOutputDigital", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.SA_OUTPUT_PWM] = self._sa_output_pwm_device = self._add_device(CheeseStick.SA_OUTPUT_PWM, "SaOutputPwm", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 100, 0)
        dict[CheeseStick.SA_OUTPUT_SERVO] = self._sa_output_servo_device = self._add_device(CheeseStick.SA_OUTPUT_SERVO, "SaOutputServo", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 255, 0)

        dict[CheeseStick.SB_MODE] = self._sb_mode_device = self._add_device(CheeseStick.SB_MODE, "SbMode", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 3, 0)
        dict[CheeseStick.SB_PULL] = self._sb_pull_device = self._add_device(CheeseStick.SB_PULL, "SbPull", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 2, 0)
        dict[CheeseStick.SB_BGV] = self._sb_bgv_device = self._add_device(CheeseStick.SB_BGV, "SbBgv", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.SB_SRC_RANGE] = self._sb_src_range_device = self._add_device(CheeseStick.SB_SRC_RANGE, "SbSrcRange", DeviceType.EFFECTOR, DataType.INTEGER, 3, 0, 255, 0)
        dict[CheeseStick.SB_DST_RANGE] = self._sb_dst_range_device = self._add_device(CheeseStick.SB_DST_RANGE, "SbDstRange", DeviceType.EFFECTOR, DataType.FLOAT, 3, -100.0, 100.0, 0)
        dict[CheeseStick.SB_OUTPUT_DIGITAL] = self._sb_output_digital_device = self._add_device(CheeseStick.SB_OUTPUT_DIGITAL, "SbOutputDigital", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.SB_OUTPUT_PWM] = self._sb_output_pwm_device = self._add_device(CheeseStick.SB_OUTPUT_PWM, "SbOutputPwm", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 100, 0)
        dict[CheeseStick.SB_OUTPUT_SERVO] = self._sb_output_servo_device = self._add_device(CheeseStick.SB_OUTPUT_SERVO, "SbOutputServo", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 255, 0)

        dict[CheeseStick.SC_MODE] = self._sc_mode_device = self._add_device(CheeseStick.SC_MODE, "ScMode", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 3, 0)
        dict[CheeseStick.SC_PULL] = self._sc_pull_device = self._add_device(CheeseStick.SC_PULL, "ScPull", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 2, 0)
        dict[CheeseStick.SC_BGV] = self._sc_bgv_device = self._add_device(CheeseStick.SC_BGV, "ScBgv", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.SC_SRC_RANGE] = self._sc_src_range_device = self._add_device(CheeseStick.SC_SRC_RANGE, "ScSrcRange", DeviceType.EFFECTOR, DataType.INTEGER, 3, 0, 255, 0)
        dict[CheeseStick.SC_DST_RANGE] = self._sc_dst_range_device = self._add_device(CheeseStick.SC_DST_RANGE, "ScDstRange", DeviceType.EFFECTOR, DataType.FLOAT, 3, -100.0, 100.0, 0)
        dict[CheeseStick.SC_OUTPUT_DIGITAL] = self._sc_output_digital_device = self._add_device(CheeseStick.SC_OUTPUT_DIGITAL, "ScOutputDigital", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.SC_OUTPUT_PWM] = self._sc_output_pwm_device = self._add_device(CheeseStick.SC_OUTPUT_PWM, "ScOutputPwm", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 100, 0)
        dict[CheeseStick.SC_OUTPUT_SERVO] = self._sc_output_servo_device = self._add_device(CheeseStick.SC_OUTPUT_SERVO, "ScOutputServo", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 255, 0)

        # ── CheeseStick Base : Effectors (L 포트) ─────────────────────────────
        dict[CheeseStick.L_PULSE_DETECT] = self._l_pulse_detect_device = self._add_device(CheeseStick.L_PULSE_DETECT, "LPulseDetect", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)

        dict[CheeseStick.LA_MODE] = self._la_mode_device = self._add_device(CheeseStick.LA_MODE, "LaMode", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 3, 0)
        dict[CheeseStick.LA_PULL] = self._la_pull_device = self._add_device(CheeseStick.LA_PULL, "LaPull", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 2, 0)
        dict[CheeseStick.LA_BGV] = self._la_bgv_device = self._add_device(CheeseStick.LA_BGV, "LaBgv", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.LA_SRC_RANGE] = self._la_src_range_device = self._add_device(CheeseStick.LA_SRC_RANGE, "LaSrcRange", DeviceType.EFFECTOR, DataType.INTEGER, 3, 0, 255, 0)
        dict[CheeseStick.LA_DST_RANGE] = self._la_dst_range_device = self._add_device(CheeseStick.LA_DST_RANGE, "LaDstRange", DeviceType.EFFECTOR, DataType.FLOAT, 3, -100.0, 100.0, 0)
        dict[CheeseStick.LA_OUTPUT_DIGITAL] = self._la_output_digital_device = self._add_device(CheeseStick.LA_OUTPUT_DIGITAL, "LaOutputDigital", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.LA_OUTPUT_PWM] = self._la_output_pwm_device = self._add_device(CheeseStick.LA_OUTPUT_PWM, "LaOutputPwm", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 100, 0)
        dict[CheeseStick.LA_OUTPUT_SERVO] = self._la_output_servo_device = self._add_device(CheeseStick.LA_OUTPUT_SERVO, "LaOutputServo", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 255, 0)

        dict[CheeseStick.LB_MODE] = self._lb_mode_device = self._add_device(CheeseStick.LB_MODE, "LbMode", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 3, 0)
        dict[CheeseStick.LB_PULL] = self._lb_pull_device = self._add_device(CheeseStick.LB_PULL, "LbPull", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 2, 0)
        dict[CheeseStick.LB_BGV] = self._lb_bgv_device = self._add_device(CheeseStick.LB_BGV, "LbBgv", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.LB_SRC_RANGE] = self._lb_src_range_device = self._add_device(CheeseStick.LB_SRC_RANGE, "LbSrcRange", DeviceType.EFFECTOR, DataType.INTEGER, 3, 0, 255, 0)
        dict[CheeseStick.LB_DST_RANGE] = self._lb_dst_range_device = self._add_device(CheeseStick.LB_DST_RANGE, "LbDstRange", DeviceType.EFFECTOR, DataType.FLOAT, 3, -100.0, 100.0, 0)
        dict[CheeseStick.LB_OUTPUT_DIGITAL] = self._lb_output_digital_device = self._add_device(CheeseStick.LB_OUTPUT_DIGITAL, "LbOutputDigital", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.LB_OUTPUT_PWM] = self._lb_output_pwm_device = self._add_device(CheeseStick.LB_OUTPUT_PWM, "LbOutputPwm", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 100, 0)
        dict[CheeseStick.LB_OUTPUT_SERVO] = self._lb_output_servo_device = self._add_device(CheeseStick.LB_OUTPUT_SERVO, "LbOutputServo", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 255, 0)

        dict[CheeseStick.LC_MODE] = self._lc_mode_device = self._add_device(CheeseStick.LC_MODE, "LcMode", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 3, 0)
        dict[CheeseStick.LC_PULL] = self._lc_pull_device = self._add_device(CheeseStick.LC_PULL, "LcPull", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 2, 0)
        dict[CheeseStick.LC_BGV] = self._lc_bgv_device = self._add_device(CheeseStick.LC_BGV, "LcBgv", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.LC_SRC_RANGE] = self._lc_src_range_device = self._add_device(CheeseStick.LC_SRC_RANGE, "LcSrcRange", DeviceType.EFFECTOR, DataType.INTEGER, 3, 0, 255, 0)
        dict[CheeseStick.LC_DST_RANGE] = self._lc_dst_range_device = self._add_device(CheeseStick.LC_DST_RANGE, "LcDstRange", DeviceType.EFFECTOR, DataType.FLOAT, 3, -100.0, 100.0, 0)
        dict[CheeseStick.LC_OUTPUT_DIGITAL] = self._lc_output_digital_device = self._add_device(CheeseStick.LC_OUTPUT_DIGITAL, "LcOutputDigital", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.LC_OUTPUT_PWM] = self._lc_output_pwm_device = self._add_device(CheeseStick.LC_OUTPUT_PWM, "LcOutputPwm", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 100, 0)
        dict[CheeseStick.LC_OUTPUT_SERVO] = self._lc_output_servo_device = self._add_device(CheeseStick.LC_OUTPUT_SERVO, "LcOutputServo", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 255, 0)

        # ── CheeseStick Base : Effectors (M 포트) ─────────────────────────────
        dict[CheeseStick.M_MODE] = self._m_mode_device = self._add_device(CheeseStick.M_MODE, "MMode", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 3, 0)
        dict[CheeseStick.M_DRIVER] = self._m_driver_device = self._add_device(CheeseStick.M_DRIVER, "MDriver", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 2, 0)
        dict[CheeseStick.M_STEP_PPS] = self._m_step_pps_device = self._add_device(CheeseStick.M_STEP_PPS, "MStepPps", DeviceType.EFFECTOR, DataType.INTEGER, 1, -1000, 1000, 0)

        dict[CheeseStick.MAB_MODE] = self._mab_mode_device = self._add_device(CheeseStick.MAB_MODE, "MabMode", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 2, 0)
        dict[CheeseStick.MAB_MOTOR_A] = self._mab_motor_a_device = self._add_device(CheeseStick.MAB_MOTOR_A, "MabMotorA", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.MAB_MOTOR_B] = self._mab_motor_b_device = self._add_device(CheeseStick.MAB_MOTOR_B, "MabMotorB", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.MAB_OUTPUT_DIGITAL] = self._mab_output_digital_device = self._add_device(CheeseStick.MAB_OUTPUT_DIGITAL, "MabOutputDigital", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.MAB_OUTPUT_PWM] = self._mab_output_pwm_device = self._add_device(CheeseStick.MAB_OUTPUT_PWM, "MabOutputPwm", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 100, 0)
        dict[CheeseStick.MAB_OUTPUT_SERVO] = self._mab_output_servo_device = self._add_device(CheeseStick.MAB_OUTPUT_SERVO, "MabOutputServo", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 255, 0)

        dict[CheeseStick.MCD_MODE] = self._mcd_mode_device = self._add_device(CheeseStick.MCD_MODE, "McdMode", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 2, 0)
        dict[CheeseStick.MCD_MOTOR_C] = self._mcd_motor_c_device = self._add_device(CheeseStick.MCD_MOTOR_C, "McdMotorC", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.MCD_MOTOR_D] = self._mcd_motor_d_device = self._add_device(CheeseStick.MCD_MOTOR_D, "McdMotorD", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.MCD_OUTPUT_DIGITAL] = self._mcd_output_digital_device = self._add_device(CheeseStick.MCD_OUTPUT_DIGITAL, "McdOutputDigital", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.MCD_OUTPUT_PWM] = self._mcd_output_pwm_device = self._add_device(CheeseStick.MCD_OUTPUT_PWM, "McdOutputPwm", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 100, 0)
        dict[CheeseStick.MCD_OUTPUT_SERVO] = self._mcd_output_servo_device = self._add_device(CheeseStick.MCD_OUTPUT_SERVO, "McdOutputServo", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 255, 0)

        # ── CheeseStick Base : Effectors (Sound / Accel config) ──────────────
        dict[CheeseStick.SOUND_BUZZ] = self._sound_buzz_device = self._add_device(CheeseStick.SOUND_BUZZ, "SoundBuzz", DeviceType.EFFECTOR, DataType.FLOAT, 1, 0, 6553.5, 0)
        dict[CheeseStick.ACCEL_G_RANGE] = self._accel_g_range_device = self._add_device(CheeseStick.ACCEL_G_RANGE, "AccelGRange", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 3, 0)
        dict[CheeseStick.ACCEL_BANDWIDTH] = self._accel_bandwidth_device = self._add_device(CheeseStick.ACCEL_BANDWIDTH, "AccelBandwidth", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 7, 0)
        # dict[CheeseStick.LED_RED_POWER] = self._led_red_power_device = self._add_device(CheeseStick.LED_RED_POWER, "LedRedPower", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)
        # dict[CheeseStick.LED_RED_REMOTE] = self._led_red_remote_device = self._add_device(CheeseStick.LED_RED_REMOTE, "LedRedRemote", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)
        # dict[CheeseStick.LED_BLUE_POWER] = self._led_blue_power_device = self._add_device(CheeseStick.LED_BLUE_POWER, "LedBluePower", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)
        # dict[CheeseStick.LED_BLUE_REMOTE] = self._led_blue_remote_device = self._add_device(CheeseStick.LED_BLUE_REMOTE, "LedBlueRemote", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)

        # ── CheeseStick Base : Commands ──────────────────────────────────────
        dict[CheeseStick.M_STEP_MOVE] = self._m_step_move_device = self._add_device(CheeseStick.M_STEP_MOVE, "MStepMove", DeviceType.COMMAND, DataType.INTEGER, 1, 0, 65535, 0)
        dict[CheeseStick.SOUND_NOTE] = self._sound_note_device = self._add_device(CheeseStick.SOUND_NOTE, "SoundNote", DeviceType.COMMAND, DataType.INTEGER, 1, 0, 88, 0)
        dict[CheeseStick.SOUND_CLIP] = self._sound_clip_device = self._add_device(CheeseStick.SOUND_CLIP, "SoundClip", DeviceType.COMMAND, DataType.INTEGER, 1, 0, 53, 0)

        # ── CheeseStick Base : Sensors ───────────────────────────────────────
        dict[CheeseStick.SA_INPUT] = self._sa_input_device = self._add_device(CheeseStick.SA_INPUT, "SaInput", DeviceType.SENSOR, DataType.INTEGER, 1, -100, 255, 0)
        dict[CheeseStick.SB_INPUT] = self._sb_input_device = self._add_device(CheeseStick.SB_INPUT, "SbInput", DeviceType.SENSOR, DataType.INTEGER, 1, -100, 255, 0)
        dict[CheeseStick.SC_INPUT] = self._sc_input_device = self._add_device(CheeseStick.SC_INPUT, "ScInput", DeviceType.SENSOR, DataType.INTEGER, 1, -100, 255, 0)
        dict[CheeseStick.SC_PULSE_INPUT_STATE] = self._sc_pulse_input_state_device = self._add_device(CheeseStick.SC_PULSE_INPUT_STATE, "ScPulseInputState", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.SC_PULSE_INPUT_COUNT] = self._sc_pulse_input_count_device = self._add_device(CheeseStick.SC_PULSE_INPUT_COUNT, "ScPulseInputCount", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 127, 0)

        dict[CheeseStick.LA_INPUT] = self._la_input_device = self._add_device(CheeseStick.LA_INPUT, "LaInput", DeviceType.SENSOR, DataType.INTEGER, 1, -100, 255, 0)
        dict[CheeseStick.LB_INPUT] = self._lb_input_device = self._add_device(CheeseStick.LB_INPUT, "LbInput", DeviceType.SENSOR, DataType.INTEGER, 1, -100, 255, 0)
        dict[CheeseStick.LC_INPUT] = self._lc_input_device = self._add_device(CheeseStick.LC_INPUT, "LcInput", DeviceType.SENSOR, DataType.INTEGER, 1, -100, 255, 0)
        dict[CheeseStick.LC_PULSE_INPUT_STATE] = self._lc_pulse_input_state_device = self._add_device(CheeseStick.LC_PULSE_INPUT_STATE, "LcPulseInputState", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.LC_PULSE_INPUT_COUNT] = self._lc_pulse_input_count_device = self._add_device(CheeseStick.LC_PULSE_INPUT_COUNT, "LcPulseInputCount", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 127, 0)

        dict[CheeseStick.ACCELERATION_X] = self._acceleration_x_device = self._add_device(CheeseStick.ACCELERATION_X, "AccelerationX", DeviceType.SENSOR, DataType.INTEGER, 1, -2048, 2047, 0)
        dict[CheeseStick.ACCELERATION_Y] = self._acceleration_y_device = self._add_device(CheeseStick.ACCELERATION_Y, "AccelerationY", DeviceType.SENSOR, DataType.INTEGER, 1, -2048, 2047, 0)
        dict[CheeseStick.ACCELERATION_Z] = self._acceleration_z_device = self._add_device(CheeseStick.ACCELERATION_Z, "AccelerationZ", DeviceType.SENSOR, DataType.INTEGER, 1, -2048, 2047, 0)
        dict[CheeseStick.TEMPERATURE] = self._temperature_device = self._add_device(CheeseStick.TEMPERATURE, "Temperature", DeviceType.SENSOR, DataType.INTEGER, 1, -40, 88, 0)
        dict[CheeseStick.SIGNAL_STRENGTH] = self._signal_strength_device = self._add_device(CheeseStick.SIGNAL_STRENGTH, "SignalStrength", DeviceType.SENSOR, DataType.INTEGER, 1, -128, 0, 0)
        dict[CheeseStick.BATTERY] = self._battery_device = self._add_device(CheeseStick.BATTERY, "Battery", DeviceType.SENSOR, DataType.FLOAT, 1, 2.0, 5.0, 0)

        # ── CheeseStick Base : Events ────────────────────────────────────────
        dict[CheeseStick.M_STEP_STATE] = self._m_step_state_device = self._add_device(CheeseStick.M_STEP_STATE, "MStepState", DeviceType.EVENT, DataType.INTEGER, 1, 0, 0, 3)
        dict[CheeseStick.SOUND_STATE] = self._sound_state_device = self._add_device(CheeseStick.SOUND_STATE, "SoundState", DeviceType.EVENT, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.SC_PULSE_INPUT_DETECT] = self._sc_pulse_input_detect_device = self._add_device(CheeseStick.SC_PULSE_INPUT_DETECT, "ScPulseInputDetect", DeviceType.EVENT, DataType.INTEGER, 0, 0, 0, 0)
        dict[CheeseStick.LC_PULSE_INPUT_DETECT] = self._lc_pulse_input_detect_device = self._add_device(CheeseStick.LC_PULSE_INPUT_DETECT, "LcPulseInputDetect", DeviceType.EVENT, DataType.INTEGER, 0, 0, 0, 0)
        dict[CheeseStick.TAP_STATE] = self._tap_state_device = self._add_device(CheeseStick.TAP_STATE, "TapState", DeviceType.EVENT, DataType.INTEGER, 0, 0, 0, 0)
        dict[CheeseStick.FALL_STATE] = self._fall_state_device = self._add_device(CheeseStick.FALL_STATE, "FallState", DeviceType.EVENT, DataType.INTEGER, 0, 0, 0, 0)

        # ── NeoPixel ──
        dict[CheeseStick.NEO_MODE] = self._neo_mode_device = self._add_device(CheeseStick.NEO_MODE, "NeoMode", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.NEO_FROM] = self._neo_from_device = self._add_device(CheeseStick.NEO_FROM, "NeoFrom", DeviceType.EFFECTOR, DataType.INTEGER, 1, 1, 144, 1)
        dict[CheeseStick.NEO_TO] = self._neo_to_device = self._add_device(CheeseStick.NEO_TO, "NeoTo", DeviceType.EFFECTOR, DataType.INTEGER, 1, 1, 144, 1)
        dict[CheeseStick.NEO_INCREMENT] = self._neo_increment_device = self._add_device(CheeseStick.NEO_INCREMENT, "NeoIncrement", DeviceType.EFFECTOR, DataType.INTEGER, 1, 1, 143, 1)
        dict[CheeseStick.NEO_BRIGHTNESS] = self._neo_brightness_device = self._add_device(CheeseStick.NEO_BRIGHTNESS, "NeoBrightness", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 100, 100)

        dict[CheeseStick.NEO_RED] = self._neo_red_device = self._add_device(CheeseStick.NEO_RED, "NeoRed", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 255, 0)
        dict[CheeseStick.NEO_RED_CHANGE] = self._neo_red_change_device = self._add_device(CheeseStick.NEO_RED_CHANGE, "NeoRedChange", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.NEO_GREEN] = self._neo_green_device = self._add_device(CheeseStick.NEO_GREEN, "NeoGreen", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 255, 0)
        dict[CheeseStick.NEO_GREEN_CHANGE] = self._neo_green_change_device = self._add_device(CheeseStick.NEO_GREEN_CHANGE, "NeoGreenChange", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.NEO_BLUE] = self._neo_blue_device = self._add_device(CheeseStick.NEO_BLUE, "NeoBlue", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 255, 0)
        dict[CheeseStick.NEO_BLUE_CHANGE] = self._neo_blue_change_device = self._add_device(CheeseStick.NEO_BLUE_CHANGE, "NeoBlueChange", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.NEO_WHITE] = self._neo_white_device = self._add_device(CheeseStick.NEO_WHITE, "NeoWhite", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 255, 0)
        dict[CheeseStick.NEO_WHITE_CHANGE] = self._neo_white_change_device = self._add_device(CheeseStick.NEO_WHITE_CHANGE, "NeoWhiteChange", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)

        dict[CheeseStick.NEO_PATTERN_MODE] = self._neo_pattern_mode_device = self._add_device(CheeseStick.NEO_PATTERN_MODE, "NeoPatternMode", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 255, 0)
        dict[CheeseStick.NEO_PATTERN_BLOCK] = self._neo_pattern_block_device = self._add_device(CheeseStick.NEO_PATTERN_BLOCK, "NeoPatternBlock", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 255, 255)
        dict[CheeseStick.NEO_PATTERN_SKIP] = self._neo_pattern_skip_device = self._add_device(CheeseStick.NEO_PATTERN_SKIP, "NeoPatternSkip", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 127, 0)
        dict[CheeseStick.NEO_PATTERN_CLEAR] = self._neo_pattern_clear_device = self._add_device(CheeseStick.NEO_PATTERN_CLEAR, "NeoPatternClear", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)

        dict[CheeseStick.NEO_SHIFT_MODE] = self._neo_shift_mode_device = self._add_device(CheeseStick.NEO_SHIFT_MODE, "NeoShiftMode", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.NEO_SHIFT_DIRECTION] = self._neo_shift_direction_device = self._add_device(CheeseStick.NEO_SHIFT_DIRECTION, "NeoShiftDirection", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.NEO_SHIFT_PIXEL] = self._neo_shift_pixel_device = self._add_device(CheeseStick.NEO_SHIFT_PIXEL, "NeoShiftPixel", DeviceType.EFFECTOR, DataType.INTEGER, 1, 1, 143, 1)

        dict[CheeseStick.NEO_COMMAND] = self._neo_command_device = self._add_device(CheeseStick.NEO_COMMAND, "NeoCommand", DeviceType.COMMAND, DataType.INTEGER, 1, 0, 5, 0)

        # ── HAT 디바이스 ─────────────────────────────────────────────────────
        dict[CheeseStick.HAT] = self._hat_device = self._add_device(CheeseStick.HAT, "Hat", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 255, 0)

        # ── HAT010 (5x5 LED matrix + 2 buttons) ──────────────────────────────
        dict[CheeseStick.HAT010_X] = self._hat010_x_device = self._add_device(CheeseStick.HAT010_X, "Hat010X", DeviceType.EFFECTOR, DataType.INTEGER, 1, -100, 100, 0)
        dict[CheeseStick.HAT010_Y] = self._hat010_y_device = self._add_device(CheeseStick.HAT010_Y, "Hat010Y", DeviceType.EFFECTOR, DataType.INTEGER, 1, -100, 100, 0)
        dict[CheeseStick.HAT010_ORIGIN_X] = self._hat010_origin_x_device = self._add_device(CheeseStick.HAT010_ORIGIN_X, "Hat010OriginX", DeviceType.EFFECTOR, DataType.INTEGER, 1, -100, 100, 0)
        dict[CheeseStick.HAT010_ORIGIN_Y] = self._hat010_origin_y_device = self._add_device(CheeseStick.HAT010_ORIGIN_Y, "Hat010OriginY", DeviceType.EFFECTOR, DataType.INTEGER, 1, -100, 100, 0)
        dict[CheeseStick.HAT010_BRIGHTNESS] = self._hat010_brightness_device = self._add_device(CheeseStick.HAT010_BRIGHTNESS, "Hat010Brightness", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 100, 100)

        dict[CheeseStick.HAT010_LED] = self._hat010_led_device = self._add_device(CheeseStick.HAT010_LED, "Hat010Led", DeviceType.COMMAND, DataType.INTEGER, 1, 0, 9, 0)
        dict[CheeseStick.HAT010_DRAW] = self._hat010_draw_device = self._add_device(CheeseStick.HAT010_DRAW, "Hat010Draw", DeviceType.COMMAND, DataType.INTEGER, 25, 0, 9, 0)
        dict[CheeseStick.HAT010_CLEAR] = self._hat010_clear_device = self._add_device(CheeseStick.HAT010_CLEAR, "Hat010Clear", DeviceType.COMMAND, DataType.INTEGER, 1, 0, 1, 0)

        dict[CheeseStick.HAT010_BUTTON_A] = self._hat010_button_a_device = self._add_device(CheeseStick.HAT010_BUTTON_A, "Hat010ButtonA", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.HAT010_BUTTON_B] = self._hat010_button_b_device = self._add_device(CheeseStick.HAT010_BUTTON_B, "Hat010ButtonB", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)

        dict[CheeseStick.HAT010_LED_STATE] = self._hat010_led_state_device = self._add_device(CheeseStick.HAT010_LED_STATE, "Hat010LedState", DeviceType.EVENT, DataType.INTEGER, 0, 0, 0, 0)
        dict[CheeseStick.HAT010_DRAW_STATE] = self._hat010_draw_state_device = self._add_device(CheeseStick.HAT010_DRAW_STATE, "Hat010DrawState", DeviceType.EVENT, DataType.INTEGER, 0, 0, 0, 0)
        dict[CheeseStick.HAT010_BUTTON_A_STATE] = self._hat010_button_a_state_device = self._add_device(CheeseStick.HAT010_BUTTON_A_STATE, "Hat010ButtonAState", DeviceType.EVENT, DataType.INTEGER, 0, 0, 0, 0)
        dict[CheeseStick.HAT010_BUTTON_B_STATE] = self._hat010_button_b_state_device = self._add_device(CheeseStick.HAT010_BUTTON_B_STATE, "Hat010ButtonBState", DeviceType.EVENT, DataType.INTEGER, 0, 0, 0, 0)

        # ── HAT022 (12 capacitive notes + LEFT/RIGHT/FN) ─────────────────────
        dict[CheeseStick.HAT022_C] = self._hat022_c_device = self._add_device(CheeseStick.HAT022_C, "Hat022C", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.HAT022_C_SHARP] = self._hat022_c_sharp_device = self._add_device(CheeseStick.HAT022_C_SHARP, "Hat022CSharp", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.HAT022_D] = self._hat022_d_device = self._add_device(CheeseStick.HAT022_D, "Hat022D", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.HAT022_D_SHARP] = self._hat022_d_sharp_device = self._add_device(CheeseStick.HAT022_D_SHARP, "Hat022DSharp", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.HAT022_E] = self._hat022_e_device = self._add_device(CheeseStick.HAT022_E, "Hat022E", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.HAT022_F] = self._hat022_f_device = self._add_device(CheeseStick.HAT022_F, "Hat022F", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.HAT022_F_SHARP] = self._hat022_f_sharp_device = self._add_device(CheeseStick.HAT022_F_SHARP, "Hat022FSharp", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.HAT022_G] = self._hat022_g_device = self._add_device(CheeseStick.HAT022_G, "Hat022G", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.HAT022_G_SHARP] = self._hat022_g_sharp_device = self._add_device(CheeseStick.HAT022_G_SHARP, "Hat022GSharp", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.HAT022_A] = self._hat022_a_device = self._add_device(CheeseStick.HAT022_A, "Hat022A", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.HAT022_A_SHARP] = self._hat022_a_sharp_device = self._add_device(CheeseStick.HAT022_A_SHARP, "Hat022ASharp", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.HAT022_B] = self._hat022_b_device = self._add_device(CheeseStick.HAT022_B, "Hat022B", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.HAT022_LEFT] = self._hat022_left_device = self._add_device(CheeseStick.HAT022_LEFT, "Hat022Left", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.HAT022_RIGHT] = self._hat022_right_device = self._add_device(CheeseStick.HAT022_RIGHT, "Hat022Right", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.HAT022_FN] = self._hat022_fn_device = self._add_device(CheeseStick.HAT022_FN, "Hat022Fn", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)

        # ── PID 디바이스 ─────────────────────────────────────────────────────
        dict[CheeseStick.PID] = self._pid_device = self._add_device(CheeseStick.PID, "Pid", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 255, 0)

        # ── PID10 (ultrasonic distance) ──────────────────────────────────────
        # dict[CheeseStick.PID10_DISTANCE] = self._pid10_distance_device = self._add_device(CheeseStick.PID10_DISTANCE, "Pid10Distance", DeviceType.SENSOR, DataType.INTEGER, 1, 20, 2000, 20)
        # dict[CheeseStick.PID10_ECHOTIME] = self._pid10_echotime_device = self._add_device(CheeseStick.PID10_ECHOTIME, "Pid10EchoTime", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 65535, 0)

        # ── PID13 (joystick + 2 buttons) ─────────────────────────────────────
        dict[CheeseStick.PID13_X] = self._pid13_x_device = self._add_device(CheeseStick.PID13_X, "Pid13X", DeviceType.SENSOR, DataType.INTEGER, 1, -128, 127, 0)
        dict[CheeseStick.PID13_Y] = self._pid13_y_device = self._add_device(CheeseStick.PID13_Y, "Pid13Y", DeviceType.SENSOR, DataType.INTEGER, 1, -128, 127, 0)
        dict[CheeseStick.PID13_BUTTON_A] = self._pid13_button_a_device = self._add_device(CheeseStick.PID13_BUTTON_A, "Pid13ButtonA", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)
        dict[CheeseStick.PID13_BUTTON_B] = self._pid13_button_b_device = self._add_device(CheeseStick.PID13_BUTTON_B, "Pid13ButtonB", DeviceType.SENSOR, DataType.INTEGER, 1, 0, 1, 0)

        dict[CheeseStick.PID13_BUTTON_A_STATE] = self._pid13_button_a_state_device = self._add_device(CheeseStick.PID13_BUTTON_A_STATE, "Pid13ButtonAState", DeviceType.EVENT, DataType.INTEGER, 0, 0, 0, 0)
        dict[CheeseStick.PID13_BUTTON_B_STATE] = self._pid13_button_b_state_device = self._add_device(CheeseStick.PID13_BUTTON_B_STATE, "Pid13ButtonBState", DeviceType.EVENT, DataType.INTEGER, 0, 0, 0, 0)

        # ── PID26 (env: pressure / temp / humidity) ──────────────────────────
        dict[CheeseStick.PID26_PRESSURE] = self._pid26_pressure_device = self._add_device(CheeseStick.PID26_PRESSURE, "Pid26Pressure", DeviceType.SENSOR, DataType.FLOAT, 1, 300.0, 1100.0, 300.0)
        dict[CheeseStick.PID26_TEMPERATURE] = self._pid26_temperature_device = self._add_device(CheeseStick.PID26_TEMPERATURE, "Pid26Temperature", DeviceType.SENSOR, DataType.FLOAT, 1, -40.0, 85.0, 0)
        dict[CheeseStick.PID26_HUMIDITY] = self._pid26_humidity_device = self._add_device(CheeseStick.PID26_HUMIDITY, "Pid26Humidity", DeviceType.SENSOR, DataType.FLOAT, 1, 10.0, 90.0, 0)

        # ── SUT 디바이스 ─────────────────────────────────────────────────────
        # dict[CheeseStick.SUT] = self._sut_device = self._add_device(CheeseStick.SUT, "Sut", DeviceType.EFFECTOR, DataType.INTEGER, 1, 0, 255, 0)


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

        tag = "CheeseStick[{}]".format(self._index)
        self._connector = SerialConnector(tag, CheeseStickConnectionChecker(self))
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
            super(CheeseStickRoboid, self)._dispose()
            self._release()
    
    def _reset(self):
        super(CheeseStickRoboid, self)._reset()

        # ── CheeseStick 디바이스 ──
        # ── Motoring 상태 (effector: 매 사이클 read / command: _is_written latch) ──
        # ── S 포트 ──
        self._s_pulse_detect = 0
        self._s_neo = 0
        
        self._sa_mode = 0
        self._sa_pull = 0
        self._sa_bgv = 0
        self._sa_src_range = [None] * 3  # min, median, max
        self._sa_dst_range = [None] * 3  # min, median, max
        self._sa_output_digital = 0
        self._sa_output_pwm = 0
        self._sa_output_servo = 0

        self._sb_mode = 0
        self._sb_pull = 0
        self._sb_bgv = 0
        self._sb_src_range = [None] * 3  # min, median, max
        self._sb_dst_range = [None] * 3  # min, median, max
        self._sb_output_digital = 0
        self._sb_output_pwm = 0
        self._sb_output_servo = 0

        self._sc_mode = 0
        self._sc_pull = 0
        self._sc_bgv = 0
        self._sc_src_range = [None] * 3  # min, median, max
        self._sc_dst_range = [None] * 3  # min, median, max
        self._sc_output_digital = 0
        self._sc_output_pwm = 0
        self._sc_output_servo = 0

        # ── L 포트 ──
        self._l_pulse_detect = 0

        self._la_mode = 0
        self._la_pull = 0
        self._la_bgv = 0
        self._la_src_range = [None] * 3  # min, median, max
        self._la_dst_range = [None] * 3  # min, median, max
        self._la_output_digital = 0
        self._la_output_pwm = 0
        self._la_output_servo = 0

        self._lb_mode = 0
        self._lb_pull = 0
        self._lb_bgv = 0
        self._lb_src_range = [None] * 3  # min, median, max
        self._lb_dst_range = [None] * 3  # min, median, max
        self._lb_output_digital = 0
        self._lb_output_pwm = 0
        self._lb_output_servo = 0

        self._lc_mode = 0
        self._lc_pull = 0
        self._lc_bgv = 0
        self._lc_src_range = [None] * 3  # min, median, max
        self._lc_dst_range = [None] * 3  # min, median, max
        self._lc_output_digital = 0
        self._lc_output_pwm = 0
        self._lc_output_servo = 0

        # ── M 포트 ──
        self._m_mode = 0
        self._m_driver = 0
        self._m_step_pps = 0

        self._mab_mode = 0
        self._mab_motor_a = 0
        self._mab_motor_b = 0
        self._mab_output_digital = 0
        self._mab_output_pwm = 0
        self._mab_output_servo = 0

        self._mcd_mode = 0
        self._mcd_motor_c = 0
        self._mcd_motor_d = 0
        self._mcd_output_digital = 0
        self._mcd_output_pwm = 0
        self._mcd_output_servo = 0

        self._sound_buzz = 0
        self._accel_g_range = 0
        self._accel_bandwidth = 0

        self._m_step_move = 0
        self._sound_note = 0
        self._sound_clip = 0

        self._m_step_move_written = False
        self._sound_written = False

        # ── encode/decode 완료-이벤트 추적용 내부 상태 (Runner.wait_until 연동) ──
        self._m_step_id = 0
        self._m_step_move_prev = -1
        self._m_step_event = 0
        self._m_step_state = 0
        self._m_step_count = 0
        
        self._sound_id = 0
        self._sound_event = 0
        self._sound_state = 0
        
        self._event_m_step_id = -1
        self._event_clip_id = -1
        self._event_sc_pulse_input_detect_id = -1
        self._event_lc_pulse_input_detect_id = -1
        self._event_tap_id = -1
        self._event_fall_id = -1
        
        # ── NeoPixel ──
        # ── Motoring 상태 (effector: 매 사이클 read / command: _is_written latch) ──
        self._neo_mode = 0
        self._neo_from = 1
        self._neo_to = 1
        self._neo_increment = 1
        self._neo_brightness = 100

        self._neo_red = 0
        self._neo_red_change = 0
        self._neo_green = 0
        self._neo_green_change = 0
        self._neo_blue = 0
        self._neo_blue_change = 0
        self._neo_white = 0
        self._neo_white_change = 0

        self._neo_pattern_mode = 0
        self._neo_pattern_block = 255
        self._neo_pattern_skip = 0
        self._neo_pattern_clear = 0

        self._neo_shift_mode = 0
        self._neo_shift_direction = 0
        self._neo_shift_pixel = 1

        self._neo_command = 0

        self._neo_command_written = False

        # ── encode/decode 완료-이벤트 추적용 내부 상태 (Runner.wait_until 연동) ──
        self._neo_command_id = 0

        # ── HAT 디바이스 ──
        self._hat = 0

        # ── HAT010 ──
        # ── Motoring 상태 (effector: 매 사이클 read / command: _is_written latch) ──
        self._hat010_matrix = _5x5Matrix()
        self._hat010_x = 0
        self._hat010_y = 0
        self._hat010_origin_x = 0
        self._hat010_origin_y = 0
        self._hat010_brightness = 0

        self._hat010_led = 0
        self._hat010_draw = [0] * 25
        self._hat010_clear = 0

        self._hat010_led_written = False
        self._hat010_draw_written = False
        self._hat010_clear_written = False

        # ── encode/decode 완료-이벤트 추적용 내부 상태 (Runner.wait_until 연동) ──
        self._hat010_led_id = 0
        self._hat010_draw_id = 0

        # ── HAT022 ──
        # ── Motoring 상태 (effector: 매 사이클 read / command: _is_written latch) ──
        # ── encode/decode 완료-이벤트 추적용 내부 상태 (Runner.wait_until 연동) ──

        # ── PID 디바이스 ──
        self._pid = 0

        # ── PID10 ──
        # ── Motoring 상태 (effector: 매 사이클 read / command: _is_written latch) ──
        # ── encode/decode 완료-이벤트 추적용 내부 상태 (Runner.wait_until 연동) ──

        # ── PID13 ──
        # ── Motoring 상태 (effector: 매 사이클 read / command: _is_written latch) ──
        # ── encode/decode 완료-이벤트 추적용 내부 상태 (Runner.wait_until 연동) ──

        # ── PID26 ──
        # ── Motoring 상태 (effector: 매 사이클 read / command: _is_written latch) ──
        # ── encode/decode 완료-이벤트 추적용 내부 상태 (Runner.wait_until 연동) ──

        # ── SUT 디바이스 ─────────────────────────────────────────────────────
        # self._sut = 0

    def _request_motoring_data(self):
        with self._thread_lock:
            # ── CheeseStick Base : EFFECTOR — 매 사이클 현재 값 반영 ──
            # S 포트
            self._s_pulse_detect = self._s_pulse_detect_device.read()
            self._s_neo = self._s_neo_device.read()

            self._sa_mode = self._sa_mode_device.read()
            self._sa_pull = self._sa_pull_device.read()
            self._sa_bgv = self._sa_bgv_device.read()
            if self._sa_src_range_device._is_written():
                self._sa_src_range = [self._sa_src_range_device.read(0), self._sa_src_range_device.read(1), self._sa_src_range_device.read(2)]
            if self._sa_dst_range_device._is_written():
                self._sa_dst_range = [self._sa_dst_range_device.read(0), self._sa_dst_range_device.read(1), self._sa_dst_range_device.read(2)]
            self._sa_output_digital = self._sa_output_digital_device.read()
            self._sa_output_pwm = self._sa_output_pwm_device.read()
            self._sa_output_servo = self._sa_output_servo_device.read()

            self._sb_mode = self._sb_mode_device.read()
            self._sb_pull = self._sb_pull_device.read()
            self._sb_bgv = self._sb_bgv_device.read()
            if self._sb_src_range_device._is_written():
                self._sb_src_range = [self._sb_src_range_device.read(0), self._sb_src_range_device.read(1), self._sb_src_range_device.read(2)]
            if self._sb_dst_range_device._is_written():
                self._sb_dst_range = [self._sb_dst_range_device.read(0), self._sb_dst_range_device.read(1), self._sb_dst_range_device.read(2)]
            self._sb_output_digital = self._sb_output_digital_device.read()
            self._sb_output_pwm = self._sb_output_pwm_device.read()
            self._sb_output_servo = self._sb_output_servo_device.read()

            self._sc_mode = self._sc_mode_device.read()
            self._sc_pull = self._sc_pull_device.read()
            self._sc_bgv = self._sc_bgv_device.read()
            if self._sc_src_range_device._is_written():
                self._sc_src_range = [self._sc_src_range_device.read(0), self._sc_src_range_device.read(1), self._sc_src_range_device.read(2)]
            if self._sc_dst_range_device._is_written():
                self._sc_dst_range = [self._sc_dst_range_device.read(0), self._sc_dst_range_device.read(1), self._sc_dst_range_device.read(2)]
            self._sc_output_digital = self._sc_output_digital_device.read()
            self._sc_output_pwm = self._sc_output_pwm_device.read()
            self._sc_output_servo = self._sc_output_servo_device.read()

            # L 포트
            self._l_pulse_detect = self._l_pulse_detect_device.read()

            self._la_mode = self._la_mode_device.read()
            self._la_pull = self._la_pull_device.read()
            self._la_bgv = self._la_bgv_device.read()
            if self._la_src_range_device._is_written():
                self._la_src_range = [self._la_src_range_device.read(0), self._la_src_range_device.read(1), self._la_src_range_device.read(2)]
            if self._la_dst_range_device._is_written():
                self._la_dst_range = [self._la_dst_range_device.read(0), self._la_dst_range_device.read(1), self._la_dst_range_device.read(2)]
            self._la_output_digital = self._la_output_digital_device.read()
            self._la_output_pwm = self._la_output_pwm_device.read()
            self._la_output_servo = self._la_output_servo_device.read()

            self._lb_mode = self._lb_mode_device.read()
            self._lb_pull = self._lb_pull_device.read()
            self._lb_bgv = self._lb_bgv_device.read()
            if self._lb_src_range_device._is_written():
                self._lb_src_range = [self._lb_src_range_device.read(0), self._lb_src_range_device.read(1), self._lb_src_range_device.read(2)]
            if self._lb_dst_range_device._is_written():
                self._lb_dst_range = [self._lb_dst_range_device.read(0), self._lb_dst_range_device.read(1), self._lb_dst_range_device.read(2)]
            self._lb_output_digital = self._lb_output_digital_device.read()
            self._lb_output_pwm = self._lb_output_pwm_device.read()
            self._lb_output_servo = self._lb_output_servo_device.read()

            self._lc_mode = self._lc_mode_device.read()
            self._lc_pull = self._lc_pull_device.read()
            self._lc_bgv = self._lc_bgv_device.read()
            if self._lc_src_range_device._is_written():
                self._lc_src_range = [self._lc_src_range_device.read(0), self._lc_src_range_device.read(1), self._lc_src_range_device.read(2)]
            if self._lc_dst_range_device._is_written():
                self._lc_dst_range = [self._lc_dst_range_device.read(0), self._lc_dst_range_device.read(1), self._lc_dst_range_device.read(2)]
            self._lc_output_digital = self._lc_output_digital_device.read()
            self._lc_output_pwm = self._lc_output_pwm_device.read()
            self._lc_output_servo = self._lc_output_servo_device.read()

            # M 포트
            self._m_mode = self._m_mode_device.read()
            self._m_driver = self._m_driver_device.read()
            self._m_step_pps = self._m_step_pps_device.read()

            self._mab_mode = self._mab_mode_device.read()
            self._mab_motor_a = self._mab_motor_a_device.read()
            self._mab_motor_b = self._mab_motor_b_device.read()
            self._mab_output_digital = self._mab_output_digital_device.read()
            self._mab_output_pwm = self._mab_output_pwm_device.read()
            self._mab_output_servo = self._mab_output_servo_device.read()

            self._mcd_mode = self._mcd_mode_device.read()
            self._mcd_motor_c = self._mcd_motor_c_device.read()
            self._mcd_motor_d = self._mcd_motor_d_device.read()
            self._mcd_output_digital = self._mcd_output_digital_device.read()
            self._mcd_output_pwm = self._mcd_output_pwm_device.read()
            self._mcd_output_servo = self._mcd_output_servo_device.read()

            # Sound / Accel config
            self._sound_buzz = self._sound_buzz_device.read()
            self._accel_g_range = self._accel_g_range_device.read()
            self._accel_bandwidth = self._accel_bandwidth_device.read()

            # ── CheeseStick Base : COMMAND — _is_written latch ──
            if self._m_step_move_device._is_written():
                self._m_step_move = self._m_step_move_device.read()
                self._m_step_move_written = True
            if self._sound_note_device._is_written():
                self._sound_note = self._sound_note_device.read()
                self._sound_written = True
            if self._sound_clip_device._is_written():
                self._sound_clip = self._sound_clip_device.read()
                self._sound_written = True

            # ── NeoPixel : EFFECTOR ──
            self._neo_mode = self._neo_mode_device.read()
            self._neo_from = self._neo_from_device.read()
            self._neo_to = self._neo_to_device.read()
            self._neo_increment = self._neo_increment_device.read()
            self._neo_brightness = self._neo_brightness_device.read()

            self._neo_red = self._neo_red_device.read()
            self._neo_red_change = self._neo_red_change_device.read()
            self._neo_green = self._neo_green_device.read()
            self._neo_green_change = self._neo_green_change_device.read()
            self._neo_blue = self._neo_blue_device.read()
            self._neo_blue_change = self._neo_blue_change_device.read()
            self._neo_white = self._neo_white_device.read()
            self._neo_white_change = self._neo_white_change_device.read()

            self._neo_pattern_mode = self._neo_pattern_mode_device.read()
            self._neo_pattern_block = self._neo_pattern_block_device.read()
            self._neo_pattern_skip = self._neo_pattern_skip_device.read()
            self._neo_pattern_clear = self._neo_pattern_clear_device.read()

            self._neo_shift_mode = self._neo_shift_mode_device.read()
            self._neo_shift_direction = self._neo_shift_direction_device.read()
            self._neo_shift_pixel = self._neo_shift_pixel_device.read()

            # ── NeoPixel : COMMAND — _is_written latch ──
            if self._neo_command_device._is_written():
                self._neo_command = self._neo_command_device.read()
                self._neo_command_written = True

            # ── HAT : SLOT ──
            self._hat = self._hat_device.read()

            # ── HAT010 : EFFECTOR ──
            self._hat010_x = self._hat010_x_device.read()
            self._hat010_y = self._hat010_y_device.read()
            self._hat010_origin_x = self._hat010_origin_x_device.read()
            self._hat010_origin_y = self._hat010_origin_y_device.read()
            self._hat010_brightness = self._hat010_brightness_device.read()

            # ── HAT010 : COMMAND — _is_written latch ──
            if self._hat010_led_device._is_written():
                self._hat010_led = self._hat010_led_device.read()
                self._hat010_led_written = True
            if self._hat010_draw_device._is_written():
                self._hat010_draw = [self._hat010_draw_device.read(i) for i in range(25)]
                self._hat010_draw_written = True
            if self._hat010_clear_device._is_written():
                self._hat010_clear = self._hat010_clear_device.read()
                self._hat010_clear_written = True

            # ── PID : SLOT ──
            self._pid = self._pid_device.read()

            # ── SUT : SLOT ──
            # self._sut = self._sut_device.read()

        self._clear_written()

    def _calc_io_range(self, hex, port):
        src_range = dst_range = None
        if port == 'Sa':
            src_range = self._sa_src_range
            dst_range = self._sa_dst_range
        elif port == 'Sb':
            src_range = self._sb_src_range
            dst_range = self._sb_dst_range
        elif port == 'Sc':
            src_range = self._sc_src_range
            dst_range = self._sc_dst_range
        elif port == 'La':
            src_range = self._la_src_range
            dst_range = self._la_dst_range
        elif port == 'Lb':
            src_range = self._lb_src_range
            dst_range = self._lb_dst_range
        elif port == 'Lc':
            src_range = self._lc_src_range
            dst_range = self._lc_dst_range

        src_min = src_range[0]
        src_median = src_range[1]
        src_max = src_range[2]
        dst_min = dst_range[0]
        dst_median = dst_range[1]
        dst_max = dst_range[2]

        if src_min == None or src_max == None or dst_min == None or dst_max == None: 
            return hex
        if hex < src_min: return dst_min
        if hex > src_max: return dst_max
        if src_median == None:
            return Utils.round(dst_min + (hex / (src_max - src_min) * (dst_max - dst_min)))
        if hex == src_median:
            if src_median == src_min: return dst_min
            elif src_median == src_max: return dst_max
            else: return dst_median
        elif hex < src_median: return Utils.round(dst_min + (hex / (src_median - src_min) * (dst_median - dst_min)))
        else: return Utils.round(dst_min + (hex / (src_max - src_median) * (dst_max - dst_median)))

    @staticmethod
    def _hz_to_buzz(hz):
        if hz < 0: return 0
        if hz <= 100.0: return Utils.round(100 * hz)
        if hz <= 1000.0: return Utils.round((100 * hz + 30000) / 4)
        if hz <= 9999.69: return Utils.round((100 * hz - 100001) / 64 + 32501)
        if hz <= 99999.06: return Utils.round((100 * hz - 999970) / 512 + 46564)
        if hz <= 167746.91: return Utils.round((100 * hz - 9999907) / 8192 + 64143)
        return 0

    def _decode_sensory_packet(self, packet):
        packet = str(packet)
        self._packet_received = 0

        mode = int(packet[0:1], 16)
        product_id = int(packet[1:2], 16)
        if mode == 1:       # CheeseStick
            sa_input = int(packet[2:4], 16)
            if self._sa_mode == 0:  # digital
                self._sa_input_device._put(sa_input)
            elif self._sa_mode == 1:  # analog
                self._sa_input_device._put(self._calc_io_range(sa_input, 'Sa'))
            sb_input = int(packet[4:6], 16)
            if self._sb_mode == 0:  # digital
                self._sb_input_device._put(sb_input)
            elif self._sb_mode == 1:  # analog
                self._sb_input_device._put(self._calc_io_range(sb_input, 'Sb'))
            sc_input = int(packet[6:8], 16)
            if self._s_pulse_detect:  # pulse detect
                sc_count = sc_input & 0x7f
                sc_state = (sc_input >> 7) & 0x01
                if sc_count != self._event_sc_pulse_input_detect_id:
                    self._sc_pulse_input_detect_device._put_empty(self._event_sc_pulse_input_detect_id != -1)
                    self._event_sc_pulse_input_detect_id = sc_count
                self._sc_pulse_input_count_device._put(sc_count)
                self._sc_pulse_input_state_device._put(sc_state)
            elif self._sc_mode == 0:  # digital
                self._sc_input_device._put(sc_input)
            elif self._sc_mode == 1:  # analog
                self._sc_input_device._put(self._calc_io_range(sc_input, 'Sc'))

            la_input = int(packet[2:4], 16)
            if self._la_mode == 0:  # digital
                self._la_input_device._put(la_input)
            elif self._la_mode == 1:  # analog
                self._la_input_device._put(self._calc_io_range(la_input, 'La'))
            lb_input = int(packet[4:6], 16)
            if self._lb_mode == 0:  # digital
                self._lb_input_device._put(lb_input)
            elif self._lb_mode == 1:  # analog
                self._lb_input_device._put(self._calc_io_range(lb_input, 'Lb'))
            lc_input = int(packet[6:8], 16)
            if self._s_pulse_detect:  # pulse detect
                lc_count = lc_input & 0x7f
                lc_state = (lc_input >> 7) & 0x01
                if lc_count != self._event_lc_pulse_input_detect_id:
                    self._lc_pulse_input_detect_device._put_empty(self._event_lc_pulse_input_detect_id != -1)
                    self._event_lc_pulse_input_detect_id = lc_count
                self._lc_pulse_input_count_device._put(lc_count)
                self._lc_pulse_input_state_device._put(lc_state)
            elif self._lc_mode == 0:  # digital
                self._lc_input_device._put(lc_input)
            elif self._lc_mode == 1:  # analog
                self._lc_input_device._put(self._calc_io_range(lc_input, 'Lc'))

            # Acceleration
            acc_x = self._to_int16(int(packet[14:18], 16))
            acc_y = self._to_int16(int(packet[18:22], 16))
            acc_z = self._to_int16(int(packet[22:26], 16))
            self._acceleration_x_device._put(acc_x)
            self._acceleration_y_device._put(acc_y)
            self._acceleration_z_device._put(acc_z)

            # STEP counter
            m_step_move = abs(int(packet[26:30], 16))
            self._m_step_move_device._put(m_step_move)

            # ── 상태/이벤트 바이트 ──
            states = int(packet[30:32], 16)

            # FREE FALL 이벤트
            fall_id = (states >> 6) & 0x03
            if fall_id != self._event_fall_id and self._event_fall_id != -1:
                self._fall_state_device._put_empty()
            self._event_fall_id = fall_id

            # TAP 이벤트
            tap_id = (states >> 4) & 0x03
            if tap_id != self._event_tap_id and self._event_tap_id != -1:
                self._tap_state_device._put_empty()
            self._event_tap_id = tap_id
            
            # ── 상태/이벤트 바이트 ──
            states = int(packet[32:34], 16)
            
            # STEP MOVE 완료 이벤트
            m_step_id = (states >> 6) & 0x03
            if m_step_id != self._event_m_step_id and self._event_m_step_id != -1:
                self._m_step_state_device._put_empty()
            self._event_m_step_id = m_step_id
            
            # SOUND CLIP 완료 이벤트
            clip_id = (states >> 4) & 0x03
            if clip_id != self._event_clip_id and self._event_clip_id != -1:
                self._sound_state_device._put_empty()
            self._event_clip_id = clip_id

            temperature = self._to_int8(int(packet[34:36], 16))
            temperature = Utils.round(temperature / 2.0 + 23.0)
            self._temperature_device._put(temperature)

            signal_strength = self._to_int8(int(packet[36:38], 16))
            self._signal_strength_device._put(signal_strength)

            battery = int(packet[38:40], 16)
            battery = round(2.0 + battery / 100, 2)
            self._battery_device._put(battery)
        elif mode == 2 and product_id != 0:     # HAT
            hat = self._hat
            if hat == 10:  # HAT010 5*5 RGB Matrix
                button_a = int(packet[4:6],16)
                button_b = int(packet[6:8],16)

                if self._hat010_button_a_device.read() == 1 and button_a == 0:
                    self._hat010_button_a_state_device._put_empty()
                self._hat010_button_a_device._put(button_a)
                if self._hat010_button_b_device.read() == 1 and button_b == 0:
                    self._hat010_button_b_state_device._put_empty()
                self._hat010_button_b_device._put(button_b)
            elif hat == 22:  # HAT022 Touch Piano 12
                self._hat022_c_device._put(int(packet[4:6], 16))
                self._hat022_c_sharp_device._put(int(packet[6:8], 16))
                self._hat022_d_device._put(int(packet[8:10], 16))
                self._hat022_d_sharp_device._put(int(packet[10:12], 16))
                self._hat022_e_device._put(int(packet[12:14], 16))
                self._hat022_f_device._put(int(packet[14:16], 16))
                self._hat022_f_sharp_device._put(int(packet[16:18], 16))
                self._hat022_g_device._put(int(packet[18:20], 16))
                self._hat022_g_sharp_device._put(int(packet[20:22], 16))
                self._hat022_a_device._put(int(packet[22:24], 16))
                self._hat022_a_sharp_device._put(int(packet[24:26], 16))
                self._hat022_b_device._put(int(packet[26:28], 16))
                self._hat022_left_device._put(int(packet[28:30], 16))
                self._hat022_right_device._put(int(packet[30:32], 16))
                self._hat022_fn_device._put(int(packet[32:34], 16))
        elif mode == 3 and product_id != 0:     # PID           
            pid = self._pid

            # if pid == 10:   # PID10 Ultrasonic
            #     self._pid10_distance_device._put(int(packet[2:6], 16))
            #     self._pid10_echotime_device._put(int(packet[6:10], 16))
            if pid == 13: # PID13 Joystick & Button
                x = int(packet[2:4], 16) - 0x80
                y = int(packet[4:6], 16) - 0x80
                button_a = int(packet[6:8], 16)
                button_b = int(packet[8:10], 16)

                self._pid13_x_device._put(x)
                self._pid13_y_device._put(y)
                if self._pid13_button_a_device.read() == 1 and button_a == 0:
                    self._pid13_button_a_state_device._put_empty()
                self._pid13_button_a_device._put(button_a)
                if self._pid13_button_b_device.read() == 1 and button_b == 0:
                    self._pid13_button_b_state_device._put_empty()
                self._pid13_button_b_device._put(button_b)
            elif pid == 26: # PID26 Environment
                pressure = int(packet[2:4], 16) + (int(packet[4:6], 16) << 8) + (int(packet[6:8], 16) << 16) + (int(packet[8:10], 16) << 8)
                pressure = round(self._to_int32(pressure) * 0.01, 2)
                self._pid26_pressure_device._put(pressure)

                temperature = int(packet[10:12], 16) + (int(packet[12:14], 16) << 8) + (int(packet[14:16], 16) << 16) + (int(packet[16:18], 16) << 8)
                temperature = round(self._to_int32(temperature) * 0.01, 2)
                self._pid26_temperature_device._put(temperature)

                humidity = int(packet[18:20], 16) + (int(packet[20:22], 16) << 8) + (int(packet[22:24], 16) << 16) + (int(packet[24:26], 16) << 8)
                humidity = round(self._to_int32(humidity) / 1024, 2)
                self._pid26_humidity_device._put(humidity)
        elif mode == 4:     # NeoPixel (No Sensor)
            pass
        elif mode == 5:     # SUT (Not Implemented)
            pass
        
        self._packet_received = mode
        return True

    def _encode_motoring_packet(self, address):
        result = ""
        with self._thread_lock:
            mode = self._packet_received
            if mode == 1:
                result += "10"
                
                # config S port
                s_config = (self._s_neo << 7) | (self._s_pulse_detect << 6) | (self._sc_mode << 4) | (self._sb_mode << 2) | self._sa_mode
                result += self._to_hex(s_config)

                # config L port
                l_config = (self._l_pulse_detect << 6) | (self._lc_mode << 4) | (self._lb_mode << 2) | self._la_mode
                result += self._to_hex(l_config)

                # config M port
                m_config = (self._m_mode << 6) | (self._m_driver << 4) | (self._mab_mode << 2) | self._mcd_mode
                result += self._to_hex(m_config)

                # config PID
                result += self._to_hex(self._pid)

                # config Acceleration
                acceleration = (self._accel_bandwidth << 4) | self._accel_g_range
                result += self._to_hex(acceleration)

                # output/motor Sa
                mode = self._sa_mode
                if mode == 0 or mode == 1:  # digital_input or analog_input
                    value = (self._sa_bgv << 4) | self._sa_pull
                elif mode == 2:  # digital_output or pwm_output
                    value = 100 if self._sa_output_digital == 1 else self._sa_output_pwm
                elif mode == 3:  # servo motor
                    value = self._sa_output_servo or 1
                result += self._to_hex(value)       

                # output/motor Sb
                mode = self._sb_mode
                if mode == 0 or mode == 1:  # digital_input or analog_input
                    value = (self._sb_bgv << 4) | self._sb_pull
                elif mode == 2:  # digital_output or pwm_output
                    value = 100 if self._sb_output_digital == 1 else self._sb_output_pwm
                elif mode == 3:  # servo motor
                    value = self._sb_output_servo or 1
                result += self._to_hex(value)       

                # output/motor Sc
                mode = self._sc_mode
                if mode == 0 or mode == 1:  # digital_input or analog_input
                    value = (self._sc_bgv << 4) | self._sc_pull
                elif mode == 2:  # digital_output or pwm_output
                    value = 100 if self._sc_output_digital == 1 else self._sc_output_pwm
                elif mode == 3:  # servo motor
                    value = self._sc_output_servo or 1
                result += self._to_hex(value)  

                # output/motor La
                mode = self._la_mode
                if mode == 0 or mode == 1:  # digital_input or analog_input
                    value = (self._la_bgv << 4) | self._la_pull
                elif mode == 2:  # digital_output or pwm_output
                    if self._la_output_servo: value = self._calc_io_range(self._la_output_servo, 'La')
                    elif self._la_output_digital == 1: value = 100
                    else: value = self._la_output_pwm
                elif mode == 3:  # servo motor
                    value = self._la_output_servo or 1
                result += self._to_hex(value)   

                # output/motor Lb
                mode = self._lb_mode
                if mode == 0 or mode == 1:  # digital_input or analog_input
                    value = (self._lb_bgv << 4) | self._lb_pull
                elif mode == 2:  # digital_output or pwm_output
                    if self._lb_output_servo: value = self._calc_io_range(self._lb_output_servo, 'Lb')
                    elif self._lb_output_digital == 1: value = 100
                    else: value = self._lb_output_pwm
                elif mode == 3:  # servo motor
                    value = self._lb_output_servo or 1
                result += self._to_hex(value) 

                # output/motor Lc
                mode = self._lc_mode
                if mode == 0 or mode == 1:  # digital_input or analog_input
                    value = (self._lc_bgv << 4) | self._lc_pull
                elif mode == 2:  # digital_output or pwm_output
                    if self._lc_output_servo: value = self._calc_io_range(self._lc_output_servo, 'Lc')
                    elif self._lc_output_digital == 1: value = 100
                    else: value = self._lc_output_pwm
                elif mode == 3:  # servo motor
                    value = self._lc_output_servo or 1
                result += self._to_hex(value)                   
                
                m_mode = self._m_mode
                if m_mode == 2 or m_mode == 3:  # PPS - step motor control
                    pps = self._m_step_pps
                    result += self._to_hex2(pps)
                else:                           # normal mode
                    # output/motor Mab
                    mab_mode = self._mab_mode
                    if mab_mode == 0:  # digital_output
                        value = (self._mab_motor_b << 1) | self._mab_motor_a
                    elif mab_mode == 1:  # dc_motor
                        value = self._mab_output_pwm
                    elif mab_mode == 2:  # analog_servo
                        value = self._mab_output_servo or 1
                    result += self._to_hex(value)
                    
                    if m_mode == 0:  # control with Mab and Mcd
                        # output/motor Mcd
                        mode_mcd = self._mcd_mode
                        if mode_mcd == 0:  # digital_output
                            value = (self._mcd_motor_d << 1) | self._mcd_motor_d
                        elif mode_mcd == 1:  # dc_motor
                            value = self._mcd_output_pwm
                        elif mode_mcd == 2:  # analog_servo
                            value = self._mcd_output_servo or 1
                        result += self._to_hex(value)
                    elif m_mode == 1:  # control only with Mab
                        result += "00"

                # LED & step counter clear
                value = (self._m_step_id & 0x03) # | (self._led_red_remote << 7) | (self._led_red_power << 6) | (self._led_blue_remote << 5) | (self._led_blue_power << 4)
                result += self._to_hex(value)

                m_step_move = self._m_step_move
                if self._m_step_move_written:
                    if m_step_move != 0 or self._m_step_move_prev != 0:
                        self._m_step_id = (self._m_step_id % 255) + 1
                    self._m_step_count = 0
                    self._m_step_event = 1 if m_step_move > 0 else 0
                    self._m_step_move_prev = m_step_move
                    self._m_step_move_written = False
                result += self._to_hex2(m_step_move)

                # ── Sound : clip > note > buzz 우선순위, 완료 이벤트용 flag latch ──
                if self._sound_written:
                    if self._sound_clip > 0:
                        self._sound_id = (self._sound_id % 255) + 1
                        self._sound_event = 1
                    else:
                        self._sound_event = 0
                    self._sound_written = False
                if self._sound_clip > 0:
                    result += "00"
                    result += self._to_hex(((self._sound_id & 0x01) << 7) | self._sound_clip)
                elif self._sound_note > 0:
                    result += "01"
                    result += self._to_hex(self._sound_note)
                else:
                    buzz = self._hz_to_buzz(self._sound_buzz)
                    result += self._to_hex(min((buzz >> 8) + 2, 0xff))
                    result += self._to_hex(buzz & 0xff)
                result += "00"  # self._sut
            elif mode == 2:  # HAT
                hat = self._hat
                if hat == 10:  # HAT010 5*5 RGB Matrix
                    result += "2A"
                    result += "00"

                    if self._hat010_led_written:
                        self._hat010_matrix.set(
                            self._hat010_origin_x + self._hat010_x,
                            self._hat010_origin_y + self._hat010_y,
                            self._hat010_led,
                        )
                        self._hat010_led_state_device._put_empty()
                        self._hat010_led_written = False
                    if self._hat010_draw_written:
                        for y in range(5):
                            for x in range(5):
                                self._hat010_matrix.set(
                                    self._hat010_origin_x + self._hat010_x + x,
                                    self._hat010_origin_y + self._hat010_y + y,
                                    self._hat010_draw[y*5+x],
                                )
                        self._hat010_draw_state_device._put_empty()
                        self._hat010_draw_written = False
                    if self._hat010_clear_written:
                        self._hat010_matrix.clear()
                        self._hat010_clear_written = False

                    matrix = []
                    for y in range(5):
                        for x in range(5):
                            matrix.append(
                                self._hat010_matrix.get(
                                    self._hat010_origin_x + x,
                                    self._hat010_origin_y + y,
                                )
                            )
                    for i in range(0, 25, 2):
                        high = matrix[i] & 0x0f
                        low = matrix[i+1] & 0x0f if i+1 < 25 else 0
                        result += self._to_hex((high << 4) | low)
                    
                    result += "00" * 4
                    result += self._to_hex(self._hat010_brightness)
                elif hat == 22:  # HAT022 Touch Piano 12
                    result += "26"
                    result += "01"
                    result += "00" * 18
                else:
                    result += "20"
                    result += "00" * 19
            elif mode == 3:  # PID
                result += "30"
                result += "00" * 19
            elif mode == 4:  # NeoPixel
                result += "4"
                result += str((self._neo_mode << 3) + 0x05)

                command = self._neo_command
                if self._neo_command_written:
                    self._neo_command_id = (self._neo_command_id % 15) + 1
                    self._neo_command_written = False
                result += self._to_hex((command << 4) | self._neo_command_id)
                result += self._to_hex(self._neo_from - 1)
                result += self._to_hex(self._neo_to - 1)

                if command == 0 or command == 2:  # cmd_fill or cmd_brightness
                    result += self._to_hex(self._neo_increment)
                    result += self._to_hex(self._neo_white)
                    result += self._to_hex(self._neo_red)
                    result += self._to_hex(self._neo_green)
                    result += self._to_hex(self._neo_blue)
                    result += self._to_hex(self._neo_brightness)
                    result += "00" * 10
                elif command == 1:  # cmd_change
                    result += self._to_hex(self._neo_increment)
                    result += self._to_hex(self._neo_white)
                    result += self._to_hex(self._neo_red)
                    result += self._to_hex(self._neo_green)
                    result += self._to_hex(self._neo_blue)
                    result += self._to_hex(self._neo_brightness)
                    change = (self._neo_white_change << 3) | (self._neo_red_change << 2) | (self._neo_green_change << 1) | self._neo_blue_change
                    result += self._to_hex(change)
                    result += "00" * 9
                elif command == 3:  # cmd_pattern
                    result += self._to_hex(self._neo_pattern_mode)
                    result += self._to_hex(self._neo_brightness)
                    result += self._to_hex(self._neo_pattern_block)

                    skip = self._neo_pattern_skip
                    clear = self._neo_pattern_clear if skip == 0 else 1
                    result += self._to_hex((clear << 7) | skip)
                    result += "00" * 12
                elif command == 4:  # cmd_shift
                    result += self._to_hex(self._neo_shift_pixel)
                    result += self._to_hex(self._neo_shift_mode)
                    result += self._to_hex(self._neo_shift_direction)
                    result += self._to_hex(self._neo_brightness)
                elif command == 5:  # cmd_clear
                    result += "00" * 19
            elif mode == 5:  # SUT (Not Implemented)
                result += "50"
                result += "00" * 19
            
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
