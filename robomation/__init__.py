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


import signal

from robomation.core.scanner import Scanner
# from robomation.core.keyboard import Keyboard
from robomation.core.utils import Utils
from robomation.core.runner import Runner
# from robomation.core.model import DeviceType, DataType
from robomation.roboids import HamsterS, Hamster, Pio, Turtle, Beagle, RaccoonBot, CheeseStick
from robomation.ai import ASR, FaceDetection, DetailedFaceDetection, FaceExpression, HandDetection, BodyDetection, ObjectDetection, ColorDetection, ArucoMarker, SelfDriving

__version__ = "0.1.0"
# __author__ = 
# __email__ =

__all__ = [
    "Utils",
    "HamsterS", "Hamster", "Pio", "Turtle", "Beagle", "RaccoonBot", "CheeseStick",
    "ASR", "FaceDetection", "DetailedFaceDetection", "FaceExpression", "HandDetection", "BodyDetection", "ObjectDetection", "ColorDetection", "ArucoMarker", "SelfDriving",
    # "scan", "dispose", "set_executable", "when_do", "while_do", 
    # "wait", "wait_until_ready", "wait_until", "parallel",  
    # "DeviceType", "DataType", 
]

def _handle_signal(signal, frame):
    Runner.shutdown()
    raise SystemExit

signal.signal(signal.SIGINT, _handle_signal)

# def scan():
#     Scanner.scan()

# def dispose():
#     Runner.dispose_all()

# def set_executable(execute):
#     Runner.set_executable(execute)

# def wait(milliseconds):
#     Runner.wait(milliseconds)

# def wait_until_ready():
#     Runner.wait_until_ready()

# def wait_until(condition, args=None):
#     Runner.wait_until(condition, args)

# def when_do(condition, do, args=None):
#     Runner.when_do(condition, do, args)

# def while_do(condition, do, args=None):
#     Runner.while_do(condition, do, args)

# def parallel(*functions):
#     Runner.parallel(functions)
