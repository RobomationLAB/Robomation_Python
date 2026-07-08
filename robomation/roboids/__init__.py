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

# robomation roboid internals

from robomation.roboids.hamster import Hamster
from robomation.roboids.hamster_s import HamsterS
from robomation.roboids.pio import Pio
from robomation.roboids.turtle import Turtle
from robomation.roboids.beagle import Beagle
from robomation.roboids.raccoonbot import RaccoonBot
from robomation.roboids.cheesestick import CheeseStick

__all__ = [
    "Hamster",
    "HamsterS",
    "Pio",
    "Turtle",
    "Beagle",
    "RaccoonBot",
    "CheeseStick",
]
