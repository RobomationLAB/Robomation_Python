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

import sys
import threading
import time
from timeit import default_timer as timer

import serial.tools.list_ports

from robomation.core.error import _err

BAUD_RATE = 115200
VALID_PACKET_LENGTH = 54
RETRY = 10
TIMEOUT = 0.25
RECONNECT_DELAY = 0.5       # 연결이 끊긴 뒤 재연결을 시도하기까지의 대기 (초)
RECONNECT_INTERVAL = 1.0    # 재연결이 실패했을 때 다시 시도하는 주기 (초)

State = type("Enum", (), {"CONNECTING": 1, "CONNECTED": 2, "CONNECTION_LOST": 3, "DISCONNECTED": 4, "DISPOSED": 5})
Result = type("Enum", (), {"FOUND": 1, "NOT_CONNECTED": 2, "NOT_AVAILABLE": 3})

ADDRESS_LENGTH = 12         # 저장/전송 형식: 구분자 없는 16진수 12자


# ── 제품 고유 주소(Bluetooth Address) ────────────────────────────────────────
# 두 가지 표기가 있고 바이트 순서가 반대다.
#   저장/전송 형식  info[4], 모터링 패킷에 그대로 실린다   예) 67E1A48B4BD9
#   사용자 표기     연결 로그와 Scanner.scan() 이 보여준다  예) D9:4B:8B:A4:E1:67
# 사용자가 눈으로 볼 수 있는 것은 사용자 표기뿐이므로 그것을 입력 정본으로 삼는다.
# 표기 규칙을 바꿔야 하면 아래 두 함수만 고치면 된다.

def format_address(address):
    """저장 형식 → 사용자 표기. 생성자에 그대로 붙여넣을 수 있는 형태로 돌려준다."""
    address = str(address or "")
    if len(address) < ADDRESS_LENGTH:
        return ""
    return ":".join(address[i:i + 2] for i in range(ADDRESS_LENGTH - 2, -1, -2)).upper()


def normalize_address(text):
    """사용자 표기 → 저장 형식. info[4] 와 바로 비교할 수 있다.

    구분자(:, -, 공백)는 있어도 없어도 되고 대소문자를 구분하지 않는다.
    형식이 틀리면 None 을 돌려준다.
    """
    if text is None:
        return None
    cleaned = "".join(c for c in str(text) if c not in ":-. _").upper()
    if len(cleaned) != ADDRESS_LENGTH:
        return None
    try:
        int(cleaned, 16)
    except ValueError:
        return None
    # 사용자 표기는 바이트 역순이므로 뒤집어 저장 형식으로 만든다.
    return "".join(cleaned[i:i + 2] for i in range(ADDRESS_LENGTH - 2, -1, -2))


def looks_like_address(text):
    """문자열이 주소 표기인지 판별한다. 포트 이름(COM3, /dev/cu.*)과 구분하는 데 쓴다."""
    return isinstance(text, str) and normalize_address(text) is not None


def parse_connect_args(index=0, port_name=None, address=None):
    """로봇 생성자의 인자를 (index, port_name, address) 로 정리한다.

    기존 관례(단일 문자열을 port_name 으로 해석)를 유지하면서, 주소 표기로 보이는 문자열은 주소로 인식한다.
    COM3 나 /dev/cu.* 는 16진수 12자가 될 수 없어 겹치지 않는다.
    """
    if isinstance(index, str):
        if address is None and looks_like_address(index):
            address = index
        elif port_name is None:
            port_name = index
        index = 0
    if isinstance(port_name, str) and address is None and looks_like_address(port_name):
        address = port_name
        port_name = None
    return index, port_name, address


class SerialConnector(object):
    _claimed_ports = set()
    _claim_lock = threading.Lock()

    def __init__(self, tag, connection_checker, loader=None, address=None):
        self._tag = tag
        self._connection_checker = connection_checker
        self._loader = loader
        self._serial = None
        self._serial_line = None
        self._address = "0" * ADDRESS_LENGTH
        self._port_name = ""
        self._found = False
        self._timestamp = 0
        self._connected = False

        # 지정한 제품 고유 주소와 일치하는 로봇만 연결한다. None 이면 기존처럼 기종만 본다.
        self._address_filter = normalize_address(address)
        self._address_invalid = address is not None and self._address_filter is None
        if self._address_invalid:
            # 조용히 무시하면 사용자는 특정 로봇을 지정했다고 믿는데 아무 로봇에나 붙는다.
            _err(type(self), "address", "address", address, "제품 고유 주소 12자리 16진수 (예: 'D9:4B:8B:A4:E1:67' 또는 'D94B8BA4E167')")
    
    def open(self, port_name=None):
        if self._address_invalid:
            return Result.NOT_AVAILABLE  # 주소 형식이 틀렸다 → 어떤 로봇에도 붙지 않는다
        if port_name:
            result = self._open_port(port_name)
            if result != Result.NOT_AVAILABLE:
                return result
        else:
            # 로봇이 붙지 않은 BLE 브리지(NOT_CONNECTED)를 만나도 스캔을 멈추지 않는다.
            # 그런 포트는 후보로 한 개만 붙잡아 두고, 뒤쪽 포트까지 모두 확인한다.
            pending_name = None
            pending_serial = None
            for port in serial.tools.list_ports.comports():
                name = port[0]
                with SerialConnector._claim_lock:
                    if name in SerialConnector._claimed_ports:
                        continue  # 이미 다른 로봇이 점유 → 건너뜀
                    SerialConnector._claimed_ports.add(name)  # 낙관적 선점
                result = self._open_port(name)
                if result == Result.FOUND:
                    self._port_name = name
                    self._release_port(pending_name, pending_serial)  # 후보 포트 반납
                    return result  # 점유 확정
                if result == Result.NOT_CONNECTED and pending_name is None:
                    # 첫 후보만 점유를 유지한 채 보관하고 스캔을 계속한다.
                    pending_name = name
                    pending_serial = self._serial
                    self._serial = None
                    self._port_name = ""
                    continue
                self._release_port(name, self._serial)  # 내 것 아님 → 닫고 점유 해제
                self._serial = None
                self._port_name = ""
            if pending_name:
                # 끝까지 로봇을 못 찾았다. 브리지만 열린 포트를 그대로 사용한다.
                self._port_name = pending_name
                self._serial = pending_serial
                return Result.NOT_CONNECTED
        if self._address_filter:
            # 주소는 로봇이 연결될 때 "Connected: <포트> <주소>" 로 출력된다.
            self._print_error("No robot with address {} (the address is printed on connect)"
                              .format(format_address(self._address_filter)))
        else:
            self._print_error("No available USB to BLE bridge")
        return Result.NOT_AVAILABLE

    def _release_port(self, port_name, s):
        if s:
            try:
                s.close()
            except:
                pass
        if port_name:
            with SerialConnector._claim_lock:
                SerialConnector._claimed_ports.discard(port_name)

    def _open_port(self, port_name):
        if port_name:
            s = None
            try:
                s = serial.Serial(port_name, BAUD_RATE, rtscts=True, timeout=0.1)
                s.reset_input_buffer()
                s.reset_output_buffer()
                self._port_name = port_name  # 연결 성공 로그가 포트명을 참조한다
                result = self._check_port(s)
                if result != Result.NOT_AVAILABLE:
                    self._serial = s
                    return result
            except:
                pass
            # 실패한 프로브에서는 _port_name 을 남기지 않는다.
            # (남기면 나중에 close() 가 다른 로봇의 포트 점유를 해제해 버린다.)
            self._port_name = ""
            if s:
                try:
                    s.close()
                except:
                    pass
        return Result.NOT_AVAILABLE

    def close(self):
        self._connected = False
        serial_port = self._serial
        self._serial = None
        if serial_port:
            try:
                serial_port.close()
            except:
                pass
        port_name = self._port_name
        self._port_name = ""
        if port_name:
            with SerialConnector._claim_lock:
                SerialConnector._claimed_ports.discard(port_name)
        # 커넥터를 다시 사용할 수 있도록 수신 상태를 초기화한다.
        self._found = False
        self._timestamp = 0
        self._serial_line = None
        if port_name:  # 포트를 한 번도 잡지 못했다면 알릴 것이 없다
            self._print_message("Disposed")

    def is_connected(self):
        return self._connected

    def get_address(self):
        return self._address

    def _set_address(self, address):
        self._address = address

    def _set_connection_state(self, state):
        self._connected = (state == State.CONNECTED)
        if self._found == False and self._connected:
            self._found = True
        if self._found:
            if state == State.CONNECTED:
                address = format_address(self._address)
                if address:
                    self._print_message("Connected: {} {}".format(self._port_name, address))
                else:
                    self._print_message("Connected: {}".format(self._port_name))
            elif state == State.CONNECTION_LOST:
                self._print_error("Connection lost")

    def _read_line(self, serial):
        try:
            line = bytearray()
            terminator = ord("\r")
            while True:
                c = serial.read()[0]
                line.append(c)
                if c == terminator: break
            return line.decode("utf-8")
        except:
            return ""

    def _read_bytes(self, serial):
        try:
            if serial.in_waiting > 0:
                arr = serial.read_all()
                sz = len(arr)
                if sz > 0:
                    if self._serial_line is None:
                        self._serial_line = bytearray()
                    terminator = ord("\r")
                    bufs = []
                    index = 0
                    while index < sz:
                        c = arr[index]
                        index += 1
                        self._serial_line.append(c)
                        if c == terminator:
                            try:
                                tmp = self._serial_line.decode("utf-8")
                                if len(tmp) == VALID_PACKET_LENGTH:
                                    bufs.append(tmp)
                            except:
                                pass
                            self._serial_line = bytearray()
                    if len(bufs) > 0:
                        return bufs
        except:
            pass
        return None

    def _read_packet(self, serial, start_byte=None):
        try:
            packet = self._read_line(serial)
            if start_byte is None:
                return packet
            if packet[:2] == start_byte:
                return packet
            return None
        except:
            return None

    def write(self, packet):
        if self._serial:
            try:
                self._serial.write(packet.encode())
            except:
                pass

    def read(self):
        if self._serial:
            try:
                packets = self._read_bytes(self._serial)
                if packets is not None:
                    if self._found == False:
                        self._check_connection(self._serial)
                    elif self._connected == False:
                        if len(packets) > 0:
                            self._set_address(packets[0][41:53])
                        self._set_connection_state(State.CONNECTED)
                    self._timestamp = 0
                    return packets
                elif self._connected:
                    t = timer()
                    if self._timestamp == 0:
                        self._timestamp = t
                    elif t - self._timestamp > TIMEOUT:
                        self._set_connection_state(State.CONNECTION_LOST)
            except:
                if self._connected:
                    self._set_connection_state(State.CONNECTION_LOST)
        return None

    def _check_port(self, serial):
        self._read_packet(serial)
        packet1 = self._read_packet(serial)
        packet2 = self._read_packet(serial)
        if packet2:
            if len(packet2) == VALID_PACKET_LENGTH:
                return self._check_connection(serial)
            elif packet1 and len(packet2) == 2:
                # 포트마다 메시지를 찍지 않는다.
                # 스캔이 모두 끝난 뒤 Runner.connect_roboid() 가 최종 결과를 한 번만 보고한다.
                return Result.NOT_CONNECTED
        return Result.NOT_AVAILABLE

    def _check_connection(self, serial):
        for i in range(RETRY):
            serial.write("FF\r".encode())
            packet = self._read_packet(serial, "FF")
            if packet:
                packet = packet.strip()
                info = packet.split(",")
                if info and len(info) >= 5:
                    if self._connection_checker.check(info):
                        if self._address_filter and info[4].upper() != self._address_filter:
                            # 기종은 맞지만 지정한 개체가 아니다 → 이 포트는 넘기고 계속 찾는다
                            return Result.NOT_AVAILABLE
                        self._set_address(info[4])
                        if self._loader is not None:
                            self._loader.load(serial, info[4])
                        self._set_connection_state(State.CONNECTED)
                        return Result.FOUND
                return Result.NOT_AVAILABLE
        return Result.NOT_AVAILABLE

    def _print_message(self, message):
        sys.stdout.write("{} {}\n".format(self._tag, message))

    def _print_error(self, message):
        sys.stderr.write("{} {}\n".format(self._tag, message))


class NusSerialConnector(SerialConnector):
    def __init__(self, tag, connection_checker, loader=None, address=None):
        super(NusSerialConnector, self).__init__(tag, connection_checker, loader, address)
        self._suma = 0
        self._sumb = 0
        self._header = 0
        self._length = 0
        self._state = 0
        self._reconnecting = False
        self._closing = False
        self._reconnect_at = 0

    def _read_bytes(self, serial):
        try:
            if serial.in_waiting > 0:
                arr = serial.read_all()
                sz = len(arr)
                if sz > 0:
                    bufs = []
                    index = 0
                    while index < sz:
                        c = arr[index]
                        index += 1
                        if self._state == 0: # idle
                            if c == 0x52 or c == 0x5A: # header1
                                self._header = c
                                self._state = 1
                        elif self._state == 1: # header
                            if c == 0x4F: # header2
                                self._state = 2
                            else:
                                self._state = 0
                        elif self._state == 2: # length
                            if c > 244:
                                self._state = 0
                                #self._print_error("length error: " + c)
                            else:
                                self._length = c
                                self._serial_line = []
                                self._state = 3
                        elif self._state == 3: # packet
                            self._serial_line.append(c)
                            if len(self._serial_line) == self._length:
                                # compute checksum over header1, header2, length, data at once
                                suma = self._header
                                sumb = self._header
                                for b in (0x4F, self._length, *self._serial_line):
                                    suma = (suma + b) & 0xff
                                    sumb = (sumb + suma) & 0xff
                                self._suma = suma
                                self._sumb = sumb
                                self._state = 4
                        elif self._state == 4: # checksum1
                            if c == self._suma:
                                self._state = 5
                            else:
                                self._suma = 0
                                self._sumb = 0
                                self._state = 0
                                #self._print_error("checksum A failed.")
                        elif self._state == 5: # checksum2
                            if c == self._sumb:
                                bufs.append(self._serial_line)
                            #else:
                                #self._print_error("checksum B failed.")
                            self._suma = 0
                            self._sumb = 0
                            self._state = 0
                    if len(bufs) > 0:
                        return bufs
        except:
            self._suma = 0
            self._sumb = 0
            self._state = 0
        return None

    def write(self, packet):
        if self._serial:
            try:
                self._serial.write(packet)
            except:
                pass

    def close(self):
        self._closing = True
        super(NusSerialConnector, self).close()
        self._suma = 0
        self._sumb = 0
        self._header = 0
        self._length = 0
        self._state = 0

    def _reconnect(self):
        self._connected = False
        if self._serial:
            try:
                self._serial.close()
            except:
                pass
            self._serial = None
        # 0.5초 대기. close() 가 호출되면 즉시 중단해 dispose 가 지연되지 않게 한다.
        deadline = timer() + RECONNECT_DELAY
        while timer() < deadline:
            if self._closing:
                return
            time.sleep(0.01)
        self._reconnecting = True
        port_name = self._port_name
        self._open_port(port_name)
        self._reconnecting = False
        if self._serial is None:
            # 재연결에 실패해도 포트명을 유지해야 read() 가 다시 시도할 수 있다.
            self._port_name = port_name
            self._reconnect_at = timer() + RECONNECT_INTERVAL

    def _retry_reconnect(self):
        # 재연결 실패 후에도 주기적으로 다시 시도한다. (예전에는 여기서 영구 사망했다)
        if self._closing or self._reconnecting:
            return
        # 한 번이라도 연결된 적이 있어야 재시도한다.
        # (그러지 않으면 최초 open() 의 포트 탐색과 동시에 같은 포트를 열려고 경쟁한다)
        if self._found == False or not self._port_name:
            return
        if timer() < self._reconnect_at:
            return
        self._reconnecting = True
        port_name = self._port_name
        self._open_port(port_name)
        self._reconnecting = False
        if self._serial is None:
            self._port_name = port_name
            self._reconnect_at = timer() + RECONNECT_INTERVAL

    def read(self):
        if self._serial is None:
            self._retry_reconnect()
        if self._serial:
            try:
                packets = self._read_bytes(self._serial)
                if packets is not None:
                    if self._found == False:
                        self._check_connection(self._serial)
                    elif self._connected == False:
                        self._set_connection_state(State.CONNECTED)
                    self._timestamp = 0
                    return packets
                elif self._connected:
                    t = timer()
                    if self._timestamp == 0:
                        self._timestamp = t
                    elif t - self._timestamp > TIMEOUT:
                        self._set_connection_state(State.CONNECTION_LOST)
                        self._reconnect()
            except:
                if self._connected:
                    self._set_connection_state(State.CONNECTION_LOST)
                    self._reconnect()
        return None

    def _check_port(self, serial):
        serial.read_all()
        packet1 = serial.read()
        packet2 = serial.read()
        if packet1 is not None and packet2 is not None:
            if len(packet1) == 1 and len(packet2) == 1:
                if packet1 == b'\r' or packet2 == b'\r':
                    return Result.NOT_CONNECTED
                else:
                    return self._check_connection(serial)
        return Result.NOT_AVAILABLE
