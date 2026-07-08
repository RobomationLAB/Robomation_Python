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

import threading
import time
from timeit import default_timer as timer


class Evaluation(object):
    def __init__(self, evaluate, callback=None, event=False):
        self._evaluate = evaluate
        self._callback = callback
        self._arg = None
        self._event = event
        self._result = False
        self._result_prev = False
        self._done = False
        self._can_remove = callback is None

    def _set_arg(self, arg):
        self._arg = arg

    def _cancel(self):
        self._done = True

    def _check(self):
        if self._evaluate:
            result = False
            try:
                if self._arg is not None:
                    result = self._evaluate.__func__(self._arg)
                else:
                    result = self._evaluate.__func__()
            except:
                try:
                    if self._arg is not None:
                        result = self._evaluate(self._arg)
                    else:
                        result = self._evaluate()
                except:
                    self._evaluate = None
            if self._event:
                if result and self._result_prev == False:
                    self._result = True
                    self._done = True
                else:
                    self._result = False
                self._result_prev = result
            else:
                self._result = result
                if result:
                    self._done = True

    def _run(self):
        Runner._evaluator._add(self)
        while True:
            while self._result == False:
                time.sleep(0.01)
            if self._callback:
                try:
                    if self._arg is not None:
                        self._callback.__func__(self._arg)
                    else:
                        self._callback.__func__()
                except:
                    try:
                        if self._arg is not None:
                            self._callback(self._arg)
                        else:
                            self._callback()
                    except:
                        self._callback = None
            self._result = False
            self._done = False

    def _start(self):
        thread = threading.Thread(target=self._run)
        thread.daemon = True
        thread.start()


class Evaluator(object):
    _added = []
    _removed = []
    _evaluations = []

    @staticmethod
    def _add(evaluation):
        Evaluator._added.append(evaluation)

    @staticmethod
    def _evaluate():
        added = Evaluator._added
        removed = Evaluator._removed
        evaluations = Evaluator._evaluations

        if len(added) > 0:
            for evaluation in added:
                evaluations.append(evaluation)
            Evaluator._added = []
        for evaluation in evaluations:
            if evaluation._done:
                if evaluation._can_remove:
                    removed.append(evaluation)
            else:
                evaluation._check()
                if evaluation._result and evaluation._can_remove:
                    removed.append(evaluation)
        if len(removed) > 0:
            for evaluation in removed:
                if evaluation in evaluations:
                    evaluations.remove(evaluation)
            Evaluator._removed = []


class Runner(object):
    _added = []
    _removed = []
    _robots = []
    _components = []
    _thread = None
    _required = 0
    _checked = 0
    _start_flag = False
    _evaluator = Evaluator()
    _execute = None
    _wait_callbacks = []

    @staticmethod
    def dispose_all():
        robots = Runner._robots
        Runner._robots = []
        for robot in robots:
            robot.dispose()
        components = Runner._components
        Runner._components = []
        for component in components:
            component.dispose()

    @staticmethod
    def shutdown():
        Runner.dispose_all()

        Runner._running = False
        thread = Runner._thread
        Runner._thread = None
        if thread:
            thread.join()

    @staticmethod
    def register_robot(robot):
        Runner._added.append(robot)

    @staticmethod
    def unregister_robot(robot):
        Runner._removed.append(robot)

    @staticmethod
    def register_component(component):
        Runner._components.append(component)

    @staticmethod
    def unregister_component(component):
        components = Runner._components
        if component in components:
            components.remove(component)

    @staticmethod
    def register_required():
        Runner._required += 1

    @staticmethod
    def register_checked():
        Runner._checked += 1

    @staticmethod
    def set_executable(execute):
        Runner._execute = execute

    @staticmethod
    def register_wait_callback(fn):
        # wait()/wait_forever() 루프가 매 반복마다 (호출 스레드에서) 호출할 콜백.
        # 카메라 표시 펌프 등 "메인 스레드에서 돌아야 하는 작업"을 여기에 건다.
        if fn not in Runner._wait_callbacks:
            Runner._wait_callbacks.append(fn)

    @staticmethod
    def unregister_wait_callback(fn):
        if fn in Runner._wait_callbacks:
            Runner._wait_callbacks.remove(fn)

    @staticmethod
    def _pump_wait_callbacks():
        for fn in list(Runner._wait_callbacks):
            try:
                fn()
            except Exception:
                pass

    @staticmethod
    def wait(sec):
        current = timer()
        if isinstance(sec, (int, float)):
            if sec > 0:
                timeout = current + sec
                while timer() < timeout:
                    Runner._pump_wait_callbacks()
                    time.sleep(0.001)
            elif sec < 0:
                while True:
                    Runner._pump_wait_callbacks()
                    time.sleep(0.01)

    @staticmethod
    def wait_until_ready():
        while Runner._checked < Runner._required:
            time.sleep(0.01)

    @staticmethod
    def wait_until(condition, arg=None):
        evaluation = Evaluation(condition)
        evaluation._set_arg(arg)
        Runner._evaluator._add(evaluation)
        Runner.start()
        while evaluation._result == False:
            time.sleep(0.01)

    @staticmethod
    def when_do(condition, do, arg=None):
        Runner.start()
        evaluation = Evaluation(condition, do, True)
        evaluation._set_arg(arg)
        evaluation._start()

    @staticmethod
    def while_do(condition, do, arg=None):
        Runner.start()
        evaluation = Evaluation(condition, do)
        evaluation._set_arg(arg)
        evaluation._start()

    @staticmethod
    def parallel(functions):
        for fn in functions:
            if isinstance(fn, (list, tuple)):
                if len(fn) > 0:
                    thread = threading.Thread(target=fn[0], args=fn[1:])
            else:
                thread = threading.Thread(target=fn)
            thread.daemon = True
            thread.start()

    @staticmethod
    def dispatch(fn, wait):
        """
        wait=True  : 호출 스레드에서 sync 실행. 내부의 Runner.wait* 가 블로킹 담당.
        wait=False : daemon thread 에서 백그라운드 실행, 즉시 반환.
        """
        if wait:
            fn()
        else:
            threading.Thread(target=fn, daemon=True).start()

    @staticmethod
    def _run():
        try:
            target_time = timer()
            while Runner._running:
                if timer() > target_time:
                    added = Runner._added
                    removed = Runner._removed
                    robots = Runner._robots

                    if len(added) > 0:
                        for robot in added:
                            robots.append(robot)
                        Runner._added = []
                    if len(removed) > 0:
                        for robot in removed:
                            if robot in robots:
                                robots.remove(robot)
                        Runner._removed = []

                    for robot in robots:
                        robot._update_sensory_device_state()

                    Runner._evaluator._evaluate()

                    if Runner._execute:
                        try:
                            Runner._execute.__func__()
                        except:
                            try:
                                Runner._execute()
                            except:
                                Runner._execute = None

                    for robot in robots:
                        robot._request_motoring_data()
                    for robot in robots:
                        robot._update_motoring_device_state()
                    for robot in robots:
                        robot._notify_motoring_device_data_changed()

                    target_time += 0.02
                    time.sleep(0.01)
                time.sleep(0.001)
        except:
            pass

    @staticmethod
    def start():
        if Runner._start_flag == False:
            Runner._start_flag = True
            Runner._running = True
            thread = threading.Thread(target=Runner._run)
            Runner._thread = thread
            thread.daemon = True
            thread.start()
