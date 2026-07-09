# Copyright 2025 SCHUNK SE & Co. KG
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# this program. If not, see <https://www.gnu.org/licenses/>.
# --------------------------------------------------------------------------------
import threading
import pytest
import os
import signal
import socket
from socket import socket as Socket


def _test_timeout_sec():
    return float(os.getenv("SCHUNK_FTS_TEST_TIMEOUT_SEC", "60"))


@pytest.hookimpl(hookwrapper=True)
def pytest_runtest_call(item):
    timeout_sec = _test_timeout_sec()
    if (
        timeout_sec <= 0
        or not hasattr(signal, "SIGALRM")
        or threading.current_thread() is not threading.main_thread()
    ):
        yield
        return

    def timeout_handler(signum, frame):
        raise TimeoutError(f"{item.nodeid} exceeded {timeout_sec:.1f}s")

    previous_handler = signal.getsignal(signal.SIGALRM)
    signal.signal(signal.SIGALRM, timeout_handler)
    previous_timer = signal.setitimer(signal.ITIMER_REAL, timeout_sec)
    try:
        yield
    finally:
        signal.setitimer(signal.ITIMER_REAL, previous_timer[0], previous_timer[1])
        signal.signal(signal.SIGALRM, previous_handler)


def _unused_udp_port():
    with Socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture
def unused_udp_port():
    return _unused_udp_port()


@pytest.fixture
def unused_udp_port_factory():
    return _unused_udp_port


@pytest.fixture
def send_messages():
    def _send_to(port, messages):
        def sender():
            with Socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                for msg in messages:
                    s.sendto(msg, ("127.0.0.1", port))

        thread = threading.Thread(target=sender)
        thread.start()
        return thread

    return _send_to


def pytest_terminal_summary(terminalreporter, exitstatus, config):
    if hasattr(config, "sensor_ip"):
        print("\n=== Sensor Summary ===")
        print(f"Sensor kind: {config.sensor_kind}")
        print(f"Sensor used: {config.sensor_ip}")
        print(f"Port used: {config.sensor_port}")
        print("======================\n")
    if hasattr(config, "sensor_pair"):
        sensor_1, sensor_2 = config.sensor_pair
        print("\n=== Sensor Pair Summary ===")
        print(f"Sensor pair kind: {config.sensor_pair_kind}")
        print(f"Sensor 1: {sensor_1[0]}:{sensor_1[1]}")
        print(f"Sensor 2: {sensor_2[0]}:{sensor_2[1]}")
        print("===========================\n")
