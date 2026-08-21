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

# 핸드셰이크(FOUND) 성공 후 첫 센서 패킷을 기다리는 한계 시간.
# 정상 로봇은 20ms 주기로 패킷을 보내므로 이보다 오래 걸리면 응답 불능으로 본다.
READY_TIMEOUT = 3.0
# _release() 에서 종료 패킷을 전송할 수 있게 주는 유예 시간.
RELEASE_TIMEOUT = 1.0


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
        Runner._added = []
        Runner._removed = []
        # 남아 있는 카운터를 초기화해야 다음 연결의 wait_until_ready() 가 정상 동작한다.
        Runner._required = 0
        Runner._checked = 0

    @staticmethod
    def shutdown():
        Runner.dispose_all()

        Runner._running = False
        thread = Runner._thread
        Runner._thread = None
        if thread:
            thread.join(RELEASE_TIMEOUT)
        # 같은 프로세스에서 다시 start() 할 수 있도록 플래그를 내린다.
        Runner._start_flag = False

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
    def _register_checked(roboid):
        # roboid 당 정확히 1회만 세야 required/checked 균형이 유지된다.
        # (연결 성공/실패/타임아웃 중 어느 경로로 끝나도 1회)
        if roboid._checked == False:
            roboid._checked = True
            Runner.register_checked()

    @staticmethod
    def _print_roboid_error(roboid, message):
        try:
            tag = "{}[{}]".format(roboid.get_name(), getattr(roboid, "_index", 0))
        except:
            tag = "Roboid"
        sys.stderr.write("{} {}\n".format(tag, message))

    @staticmethod
    def connect_roboid(roboid, connector, port_name=None):
        # roboid 의 통신 스레드를 띄우고 연결이 확립될 때까지(또는 실패가 확정될 때까지) 기다린다.
        # 어떤 경우에도 유한 시간에 반환하며, 연결에 실패하면 COM 포트를 반납한다.
        from robomation.core.serial_connector import Result

        Runner.register_required()
        roboid._checked = False
        roboid._ready = False
        roboid._running = True
        roboid._releasing = 0
        roboid._release_deadline = 0
        roboid._connector = connector

        thread = threading.Thread(target=roboid._run)
        thread.daemon = True
        roboid._thread = thread
        thread.start()

        result = connector.open(port_name)
        if result != Result.NOT_AVAILABLE:
            # 핸드셰이크는 통과했다. 첫 센서 패킷이 해석될 때까지만 기다린다.
            # 통신 스레드가 죽었거나 시간이 초과되면 즉시 빠져나온다.
            # NOT_CONNECTED 도 이 경로를 타므로, 이 사이에 브리지가 로봇을 붙이면 그대로 연결된다.
            deadline = timer() + READY_TIMEOUT
            while (roboid._ready == False and roboid._is_disposed() == False
                   and thread.is_alive() and timer() < deadline):
                time.sleep(0.01)

        if roboid._ready == False:
            if roboid._is_disposed() == False:
                if thread.is_alive() == False:
                    Runner._print_roboid_error(roboid, "Communication stopped by packet error")
                elif result == Result.NOT_CONNECTED:
                    Runner._print_roboid_error(roboid, "No robot connected to the USB to BLE bridge")
                elif result != Result.NOT_AVAILABLE:
                    Runner._print_roboid_error(roboid, "No response from robot")
            # 실패가 확정됐으므로 통신 스레드를 정리하고 COM 포트를 반납한다.
            # (반납하지 않으면 같은 프로세스에서 재시도해도 그 포트를 다시 열 수 없다.)
            roboid._release()

        Runner._register_checked(roboid)
        return result

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
        # 조건이 로봇의 바운드 메서드면(모든 완료 대기가 그렇다), 그 로봇이 한 번도
        # 연결되지 않았을 때는 완료 신호가 올 수 없으므로 기다리지 않는다.
        # 연결됐다가 통신이 끊긴 경우는 _ready 가 유지되므로 기존대로 계속 기다린다.
        roboid = getattr(getattr(condition, "__self__", None), "_roboid", None)

        evaluation = Evaluation(condition)
        evaluation._set_arg(arg)
        Runner._evaluator._add(evaluation)
        Runner.start()
        while evaluation._result == False:
            if roboid is not None and roboid._ready == False:
                evaluation._cancel()  # 평가 목록에 남지 않도록 완료로 표시
                break
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
