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

import pytest
import os
import rclpy
import signal
import threading
import time
from types import ModuleType
from launch import LaunchDescription  # type: ignore [attr-defined]
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node

launch_pytest_module: ModuleType | None = None
try:
    import launch_pytest as _launch_pytest
except ModuleNotFoundError:
    pass
else:
    launch_pytest_module = _launch_pytest


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


def service_is_ready(client, timeout_sec=None):
    timeout_sec = float(
        timeout_sec or os.getenv("SCHUNK_FTS_SERVICE_TIMEOUT_SEC", "10")
    )
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        if client.wait_for_service(timeout_sec=min(0.25, max(0.0, remaining))):
            return True
    return False


def service_timeout_sec():
    return float(os.getenv("SCHUNK_FTS_SERVICE_TIMEOUT_SEC", "10"))


@pytest.fixture(scope="module")
def ros2():
    rclpy.init()
    yield
    rclpy.shutdown()


if launch_pytest_module is not None:

    @pytest.fixture(scope="function")
    def output_rate():
        return "1000"

    @launch_pytest_module.fixture(scope="function")
    def driver(request, ros2, sensor, output_rate):
        host, port = sensor

        setup = IncludeLaunchDescription(
            PathJoinSubstitution(
                [
                    FindPackageShare("schunk_fts_driver"),
                    "launch",
                    "driver.launch.py",
                ]
            ),
            launch_arguments={
                "host": str(host),
                "port": str(port),
                "output_rate": str(output_rate),
            }.items(),
        )
        return LaunchDescription([setup, launch_pytest_module.actions.ReadyToTest()])

else:

    @pytest.fixture(scope="function")
    def driver():
        pytest.skip(
            "launch_pytest is required for launch-based driver tests. "
            "Install the ROS launch_pytest package for this ROS distribution."
        )


class LifecycleInterface(object):
    def __init__(self):
        self.node = Node(f"lifecycle_interface_{time.time_ns()}")
        self.change_state_client = self.node.create_client(
            ChangeState, "/schunk/fts/change_state"
        )
        self.get_state_client = self.node.create_client(
            GetState, "/schunk/fts/get_state"
        )

        if not service_is_ready(self.change_state_client):
            raise TimeoutError("/schunk/fts/change_state service is not available")
        if not service_is_ready(self.get_state_client):
            raise TimeoutError("/schunk/fts/get_state service is not available")

    def change_state(self, transition_id):
        timeout_sec = service_timeout_sec()
        last_error = None
        for _ in range(2):
            if not service_is_ready(self.change_state_client):
                last_error = "service is not available"
                continue
            req = ChangeState.Request()
            req.transition.id = transition_id
            future = self.change_state_client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
            if future.done():
                return future.result()
            last_error = f"no response within {timeout_sec:.1f}s"
        raise TimeoutError(
            f"Timed out waiting for lifecycle transition {transition_id}: {last_error}"
        )

    def check_state(self, state_id):
        timeout_sec = service_timeout_sec()
        last_error = None
        for _ in range(2):
            if not service_is_ready(self.get_state_client):
                last_error = "service is not available"
                continue
            req = GetState.Request()
            future = self.get_state_client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
            if future.done():
                return future.result().current_state.id == state_id
            last_error = f"no response within {timeout_sec:.1f}s"
        raise TimeoutError(
            f"Timed out waiting for lifecycle state {state_id}: {last_error}"
        )

    def shutdown(self):
        """Properly shutdown the driver to inactive state."""
        from lifecycle_msgs.msg import State, Transition

        try:
            if self.check_state(State.PRIMARY_STATE_ACTIVE):
                self.change_state(Transition.TRANSITION_DEACTIVATE)
        except Exception:
            pass

        try:
            if self.check_state(State.PRIMARY_STATE_INACTIVE):
                self.change_state(Transition.TRANSITION_CLEANUP)
        except Exception:
            pass


@pytest.fixture(scope="function")
def lifecycle_interface(driver):
    interface = LifecycleInterface()
    yield interface
    interface.shutdown()
    interface.node.destroy_node()


class MessageSubscriber(Node):
    def __init__(self, msg_type, topic, node_name_suffix):
        super().__init__(f"message_subscriber_{node_name_suffix}")
        self.messages = []
        self.subscription = self.create_subscription(
            msg_type, topic, self.listener_callback, 10
        )

    def listener_callback(self, msg):
        self.messages.append(msg)


@pytest.fixture
def message_subscriber_factory(ros2):
    nodes = []

    def _factory(msg_type, topic):
        # Use a unique name to avoid conflicts if used multiple times in a test
        node = MessageSubscriber(msg_type, topic, f"{len(nodes)}")
        nodes.append(node)
        return node

    yield _factory

    for node in nodes:
        node.destroy_node()


def pytest_terminal_summary(terminalreporter, exitstatus, config):
    if hasattr(config, "sensor_ip"):
        print("\n=== Sensor Summary ===")
        print(f"Sensor kind: {config.sensor_kind}")
        print(f"Sensor used: {config.sensor_ip}")
        print(f"Port used: {config.sensor_port}")
        print("======================\n")
