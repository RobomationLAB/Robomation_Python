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

import contextlib
import io
import sys

import serial.tools.list_ports

from robomation.core.serial_connector import (SerialConnector, NusSerialConnector,
                                              Result, format_address)

# 로봇 종류 → (roboid 모듈, 연결 검사 클래스, 커넥터 클래스, 표시 이름)
# 커넥터는 프로토콜이 갈린다: Beagle 계열만 NUS, 나머지는 텍스트 패킷.
_ROBOTS = (
    ("hamster_s_roboid",   "HamsterSConnectionChecker",    SerialConnector,    "HamsterS"),
    ("hamster_roboid",     "HamsterConnectionChecker",     SerialConnector,    "Hamster"),
    ("pio_roboid",         "PioConnectionChecker",         SerialConnector,    "Pio"),
    ("turtle_roboid",      "TurtleConnectionChecker",      SerialConnector,    "Turtle"),
    ("raccoonbot_roboid",  "RaccoonBotConnectionChecker",  SerialConnector,    "RaccoonBot"),
    ("cheesestick_roboid", "CheeseStickConnectionChecker", SerialConnector,    "CheeseStick"),
    ("beagle_roboid",      "BeagleConnectionChecker",      NusSerialConnector, "Beagle"),
)

_checker_cache = None


def _checkers():
    """SDK 에 이미 있는 연결 검사 클래스를 그대로 재사용한다.

    roboid 모듈은 함수 안에서 불러온다 
    — robomation/__init__.py 가 scanner 를 roboids 보다 먼저 읽으므로 모듈 레벨 import 는 순환이 된다.
    """
    global _checker_cache
    if _checker_cache is None:
        import importlib
        _checker_cache = []
        for module_name, checker_name, connector_class, display in _ROBOTS:
            module = importlib.import_module("robomation.roboids." + module_name)
            checker = getattr(module, checker_name)(None)
            _checker_cache.append((display, checker, connector_class))
    return _checker_cache


class _Sniffer(object):
    """조사 전용 연결 검사기.

    check() 에서 핸드셰이크 정보만 챙기고 False 를 돌려준다. 
    그러면 커넥터가 NOT_AVAILABLE 로 끝나므로, 실제 연결을 만들지 않으면서 기종과 주소만 얻는다.
    """

    def __init__(self):
        self.info = None

    def check(self, info):
        self.info = info
        return False


def _identify(info):
    """핸드셰이크 정보로 기종 이름을 정한다."""
    for display, checker, _ in _checkers():
        try:
            if checker.check(info):
                return display
        except:
            pass
    return "Unknown (code={})".format(info[2] if len(info) > 2 else "?")


def _probe(port_name):
    """포트 하나를 조사해 (기종, 주소) 를 돌려준다.

    로봇이 없으면 (None, None), 이 프로토콜의 브리지가 아니면 (False, None).
    두 프로토콜을 모두 시도한다 (텍스트 패킷 → NUS).
    """
    bridge = False
    for connector_class in (SerialConnector, NusSerialConnector):
        sniffer = _Sniffer()
        probe = connector_class("scan", sniffer)
        noise = io.StringIO()  # 커넥터 자체 로그는 삼키고 목록만 남긴다
        result = Result.NOT_AVAILABLE
        with contextlib.redirect_stdout(noise), contextlib.redirect_stderr(noise):
            try:
                result = probe._open_port(port_name)
            finally:
                probe.close()
        if sniffer.info is not None:
            return _identify(sniffer.info), format_address(sniffer.info[4])
        if result == Result.NOT_CONNECTED:
            bridge = True
    return (None, None) if bridge else (False, None)


class Scanner(object):
    @staticmethod
    def scan():
        """시리얼 포트를 나열하고, 각 포트에 연결된 로봇의 기종과 제품 고유 주소를 보여준다.

        출력되는 주소는 로봇 생성자에 그대로 넣을 수 있다.
        HamsterS(address="D9:4B:8B:A4:E1:67")
        """
        sys.stdout.write("Serial ports:\n")
        rows = []
        for port in serial.tools.list_ports.comports():
            name = port[0]
            model, address = _probe(name)
            if model:
                rows.append((name, model, address))
            elif model is None:
                rows.append((name, "(no robot connected)", ""))
            else:
                rows.append((name, "", ""))
        if not rows:
            sys.stdout.write("No available serial port\n")
            return

        name_width = max(len(name) for name, _, _ in rows)
        model_width = max(len(model) for _, model, _ in rows)
        for name, model, address in rows:
            line = "  {}".format(name.ljust(name_width))
            if model:
                line += "  {}".format(model.ljust(model_width) if address else model)
            if address:
                line += "  {}".format(address)
            sys.stdout.write(line.rstrip() + "\n")
