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


DeviceType = type("Enum", (), {"SENSOR": 0, "EFFECTOR": 1, "EVENT": 2, "COMMAND": 3})
DataType = type("Enum", (), {"INTEGER": 4, "FLOAT": 5, "STRING": 6})


class NamedElement(object):
    def __init__(self, name):
        self.set_name(name)

    def get_name(self):
        return self._name

    def set_name(self, name):
        self._name = str(name)


class Device(NamedElement):
    def __init__(self, id, name, device_type, data_type, data_size, min_value, max_value, initial_value):
        super(Device, self).__init__(name)
        self._id = id & 0xfff00fff
        self._device_type = device_type
        self._data_type = data_type
        self._data_size = data_size
        self._min_value = min_value
        self._max_value = max_value
        self._initial_value = initial_value
        if data_size > 0:
            self._data = [initial_value] * data_size
        else:
            self._data = []
        self._event = False
        self._fired = False
        self._written = False
        self._can_notify = False
        self._device_data_changed_listeners = []

    def get_id(self):
        return self._id

    def get_device_type(self):
        return self._device_type

    def get_data_type(self):
        return self._data_type

    def get_data_size(self):
        return self._data_size

    def _is_written(self):
        return self._written

    def _clear_written(self):
        self._written = False

    def _check_data_type(self, value):
        if self._data_type == DataType.INTEGER or self._data_type == DataType.FLOAT:
            return isinstance(value, (int, float))
        elif self._data_type == DataType.STRING:
            return isinstance(value, str)
        return False

    def _check_range(self, value):
        if isinstance(value, (int, float)):
            if value < self._min_value:
                value = self._min_value
            elif value > self._max_value:
                value = self._max_value
            if self._data_type == DataType.INTEGER:
                return int(value)
            else:
                return float(value)
        return str(value)

    def e(self):
        return self._event

    def read(self, arg=None):
        this_data = self._data
        this_len = len(this_data)
        if isinstance(arg, (int, float)):
            index = int(arg)
            if index >= 0 and index < this_len:
                return this_data[index]
        elif isinstance(arg, list):
            length = min(this_len, len(arg))
            arg[:length] = this_data[:length]
            return length
        elif arg is None:
            if this_len == 1:
                return this_data[0]
            elif this_len > 1:
                return this_data
        if self._data_type == DataType.STRING:
            return ""
        else:
            return 0

    def write(self, arg1=None, arg2=None):
        this_data = self._data
        if arg1 is None:
            self._fired = True
            self._written = True
            self._can_notify = True
            return True
        elif isinstance(arg1, (int, float)) and isinstance(arg2, (int, float, str)):
            if self._device_type == DeviceType.SENSOR or self._device_type == DeviceType.EVENT:
                return False
            index = int(arg1)
            if index < 0:
                return False
            if self._check_data_type(arg2) == False:
                return False
            this_len = len(this_data)
            if index >= this_len:
                if self._data_size < 0:
                    new_data = [self._initial_value] * (index + 1)
                    new_data[:this_len] = this_data
                    this_data = self._data = new_data
                else:
                    return False
            this_data[index] = self._check_range(arg2)
            self._fired = True
            self._written = True
            self._can_notify = True
            return True
        elif isinstance(arg1, (int, float, str)):
            if self._device_type == DeviceType.SENSOR or self._device_type == DeviceType.EVENT:
                return False
            if self._check_data_type(arg1) == False:
                return False
            if len(this_data) <= 0:
                if self._data_size < 0:
                    this_data = self._data = [0]
                else:
                    return False
            this_data[0] = self._check_range(arg1)
            self._fired = True
            self._written = True
            self._can_notify = True
            return True
        elif isinstance(arg1, (list, tuple)):
            if self._device_type == DeviceType.SENSOR or self._device_type == DeviceType.EVENT:
                return 0
            data_len = len(arg1)
            if data_len <= 0:
                return 0
            if self._data_size < 0:
                if len(this_data) != data_len:
                    this_data = self._data = [self._initial_value] * data_len
            this_len = len(this_data)
            if this_len <= 0:
                return 0
            length = min(this_len, data_len)
            written = False
            for i in range(length):
                if self._check_data_type(arg1[i]):
                    this_data[i] = self._check_range(arg1[i])
                    self._fired = True
                    self._written = True
                    self._can_notify = True
                    written = True
            if written:
                return length
        return 0

    def _put(self, value, fired=True):
        self._data[0] = value
        self._fired = fired
        self._can_notify = True

    def _put_at(self, index, value, fired=True):
        self._data[index] = value
        self._fired = fired
        self._can_notify = True

    def _put_array(self, values, fired=True):
        self._data[:] = values[:]
        self._fired = fired
        self._can_notify = True

    def _put_empty(self, fired=True):
        self._fired = fired
        self._can_notify = True

    def _reset(self):
        self._event = False
        self._fired = False
        self._written = False
        if self._data_size > 0:
            self._data = [self._initial_value] * self._data_size
        else:
            self._data = []

    def add_device_data_changed_listener(self, listener):
        if listener:
            self._device_data_changed_listeners.append(listener)

    def remove_device_data_changed_listener(self, listener):
        if listener:
            self._device_data_changed_listeners.remove(listener)

    def clear_device_data_changed_listeners(self):
        self._device_data_changed_listeners = []

    def _update_device_state(self):
        self._event = self._fired
        self._fired = False

    def _notify_device_data_changed(self):
        if len(self._device_data_changed_listeners) > 0:
            can_notify = True
            if self._device_type == DeviceType.COMMAND or self._device_type == DeviceType.EVENT:
                can_notify = self._can_notify
            if can_notify:
                data = tuple(self._data)
                for listener in self._device_data_changed_listeners:
                    try:
                        listener.__func__(self, data)
                    except:
                        try:
                            listener(self, data)
                        except:
                            pass
        self._can_notify = False


class Roboid(NamedElement):
    def __init__(self, id, name, uid):
        super(Roboid, self).__init__(name)
        self._id = id
        self._uid = uid
        self._roboids = []
        self._devices = []
        self._sensory_devices = []
        self._motoring_devices = []
        self._disposed = False

    def _is_disposed(self):
        return self._disposed

    def _dispose(self):
        self._disposed = True
        self._reset()

    def _reset(self):
        for device in self._devices:
            device._reset()
        for roboid in self._roboids:
            roboid._reset()

    def _add_device(self, id, name, device_type, data_type, data_size, min_value, max_value, initial_value):
        dev = Device(id, name, device_type, data_type, data_size, min_value, max_value, initial_value)
        self._devices.append(dev)
        if device_type == DeviceType.SENSOR or device_type == DeviceType.EVENT:
            self._sensory_devices.append(dev)
        else:
            self._motoring_devices.append(dev)
        return dev

    def _add_roboid(self, roboid):
        if isinstance(roboid, Roboid):
            self._roboids.append(roboid)

    def get_id(self):
        return self._id

    def find_roboid_by_name(self, name):
        name = str(name).lower()
        dot = name.find(".")
        if dot < 0:
            for roboid in self._roboids:
                if name == roboid.get_name().lower():
                    return roboid
        else:
            roboid_name = name[:dot]
            sub_name = name[dot+1:]
            for roboid in self._roboids:
                if roboid_name == roboid.get_name().lower():
                    return roboid.find_roboid_by_name(sub_name)
        return None

    def find_device_by_name(self, name):
        name = str(name).lower()
        dot = name.find(".")
        if dot < 0:
            for device in self._devices:
                if name == device.get_name().lower():
                    return device
        else:
            roboid_name = name[:dot]
            sub_name = name[dot+1:]
            for roboid in self._roboids:
                if roboid_name == roboid.get_name().lower():
                    return roboid.find_device_by_name(sub_name)
        return None

    def find_device_by_id(self, device_id):
        uid = device_id & 0xfffff000
        if uid == self._uid:
            for device in self._devices:
                if device.get_id() == device_id:
                    return device
            return None

        for roboid in self._roboids:
            device = roboid.find_device_by_id(device_id)
            if device is not None:
                return device
        return None

    def add_device_data_changed_listener(self, listener):
        if listener:
            for device in self._devices:
                device.add_device_data_changed_listener(listener)
            for roboid in self._roboids:
                roboid.add_device_data_changed_listener(listener)

    def remove_device_data_changed_listener(self, listener):
        if listener:
            for device in self._devices:
                device.remove_device_data_changed_listener(listener)
            for roboid in self._roboids:
                roboid.remove_device_data_changed_listener(listener)

    def clear_device_data_changed_listeners(self):
        for device in self._devices:
            device.clear_device_data_changed_listeners()
        for roboid in self._roboids:
            roboid.clear_device_data_changed_listeners()

    def _to_int32(self, hex):
        return hex if hex < 0x80000000 else hex - 0x100000000

    def _to_int16(self, hex):
        return hex if hex < 0x8000 else hex - 0x10000
    
    def _to_int8(self, hex):
        return hex if hex < 0x80 else hex - 0x100

    def _to_hex(self, number):
        value = int(number)
        if value < 0: value += 0x100
        value = hex(value).upper()[2:]
        if len(value) > 1:
            return value
        return "0" + value

    def _to_hex2(self, number):
        value = int(number)
        if value < 0: value += 0x10000
        value = hex(value).upper()[2:]
        length = len(value)
        if length < 4:
            result = ""
            for i in range(length, 4):
                result += "0"
            return result + value
        return value

    def _to_hex3(self, number):
        value = int(number)
        if value < 0: value += 0x1000000
        value = hex(value).upper()[2:]
        length = len(value)
        if length < 6:
            result = ""
            for i in range(length, 6):
                result += "0"
            return result + value
        return value
    
    def _to_hex4(self, number):
        value = int(number)
        if value < 0: value += 0x100000000
        value = hex(value).upper()[2:]
        length = len(value)
        if length < 8:
            result = ""
            for i in range(length, 8):
                result += "0"
            return result + value
        return value

    def _request_motoring_data(self):
        pass

    def _update_sensory_device_state(self):
        for device in self._sensory_devices:
            device._update_device_state()
        for roboid in self._roboids:
            roboid._update_sensory_device_state()

    def _update_motoring_device_state(self):
        for device in self._motoring_devices:
            device._update_device_state()
        for roboid in self._roboids:
            roboid._update_motoring_device_state()

    def _notify_sensory_device_data_changed(self):
        for device in self._sensory_devices:
            device._notify_device_data_changed()
        for roboid in self._roboids:
            roboid._notify_sensory_device_data_changed()

    def _notify_motoring_device_data_changed(self):
        for device in self._motoring_devices:
            device._notify_device_data_changed()
        for roboid in self._roboids:
            roboid._notify_motoring_device_data_changed()

    def _clear_written(self):
        for device in self._devices:
            device._clear_written()
        for roboid in self._roboids:
            roboid._clear_written()

    def is_connected(self):
        if self._connector:
            return self._connector.is_connected()
        return False


class Robot(NamedElement):
    def __init__(self, id, name, index):
        super(Robot, self).__init__(name)
        self._id = id
        self._index = index
        self._roboids = []

    def dispose(self):
        for roboid in self._roboids:
            roboid._dispose()

    def get_id(self):
        return self._id

    def get_index(self):
        return self._index

    def reset(self):
        for roboid in self._roboids:
            roboid._reset()

    def _add_roboid(self, roboid):
        if isinstance(roboid, Roboid):
            self._roboids.append(roboid)

    def find_roboid_by_name(self, name):
        name = str(name).lower()
        dot = name.find(".")
        if dot < 0:
            for roboid in self._roboids:
                if name == roboid.get_name().lower():
                    return roboid
        else:
            roboid_name = name[:dot]
            sub_name = name[dot+1:]
            for roboid in self._roboids:
                if roboid_name == roboid.get_name().lower():
                    return roboid.find_roboid_by_name(sub_name)
        return None

    def find_device_by_name(self, name):
        name = str(name).lower()
        dot = name.find(".")
        if dot >= 0:
            roboid_name = name[:dot]
            sub_name = name[dot+1:]
            for roboid in self._roboids:
                if roboid_name == roboid.get_name().lower():
                    return roboid.find_device_by_name(sub_name)
        return None

    def find_device_by_id(self, device_id):
        for roboid in self._roboids:
            device = roboid.find_device_by_id(device_id)
            if device is not None:
                return device
        return None

    def e(self, device_id):
        device = self.find_device_by_id(device_id)
        if device is None: return False
        return device.e()

    def read(self, device_id, arg=None):
        device = self.find_device_by_id(device_id)
        if device is None: return 0
        return device.read(arg)

    def write(self, device_id, arg1=None, arg2=None):
        device = self.find_device_by_id(device_id)
        if device is None:
            if isinstance(arg1, (int, float)) and isinstance(arg2, (int, float, str)):
                return False
            elif isinstance(arg1, (int, float, str)):
                return False
            return 0
        return device.write(arg1, arg2)

    def add_device_data_changed_listener(self, listener):
        if listener:
            for roboid in self._roboids:
                roboid.add_device_data_changed_listener(listener)

    def remove_device_data_changed_listener(self, listener):
        if listener:
            for roboid in self._roboids:
                roboid.remove_device_data_changed_listener(listener)

    def clear_device_data_changed_listeners(self):
        for roboid in self._roboids:
            roboid.clear_device_data_changed_listeners()

    def _request_motoring_data(self):
        for roboid in self._roboids:
            roboid._request_motoring_data()

    def _update_sensory_device_state(self):
        for roboid in self._roboids:
            roboid._update_sensory_device_state()

    def _update_motoring_device_state(self):
        for roboid in self._roboids:
            roboid._update_motoring_device_state()

    def _notify_sensory_device_data_changed(self):
        for roboid in self._roboids:
            roboid._notify_sensory_device_data_changed()

    def _notify_motoring_device_data_changed(self):
        for roboid in self._roboids:
            roboid._notify_motoring_device_data_changed()

    def is_connected(self):
        if self._roboid:
            return self._roboid.is_connected()
        return False
