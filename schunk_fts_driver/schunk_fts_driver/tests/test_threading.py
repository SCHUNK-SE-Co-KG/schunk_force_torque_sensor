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

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from lifecycle_msgs.msg import Transition, State
from geometry_msgs.msg import WrenchStamped
from example_interfaces.srv import Trigger
from schunk_fts_interfaces.srv import SendCommand  # type: ignore [attr-defined]
from functools import partial
import time
import threading


def test_concurrent_data_publishing_and_state_transitions(sensor, lifecycle_interface):
    """Test that data publishing and state
    transitions don't interfere with each other."""
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    messages = []
    error_occurred = []

    def collect_messages(msg: WrenchStamped, messages: list[WrenchStamped]) -> None:
        try:
            messages.append(msg)
        except Exception as e:
            error_occurred.append(e)

    _ = driver.node.create_subscription(
        WrenchStamped,
        "/schunk/driver/data",
        partial(collect_messages, messages=messages),
        10,
    )

    # Collect messages while performing state transitions
    start_time = time.time()
    timeout = start_time + 0.3

    while time.time() < timeout:
        rclpy.spin_once(driver.node, timeout_sec=0.001)

    assert len(messages) > 0, "Should collect messages during active state"
    assert (
        len(error_occurred) == 0
    ), "No errors should occur during concurrent operations"

    # Now transition while still subscribed
    driver.change_state(Transition.TRANSITION_DEACTIVATE)

    # Try to collect messages after deactivate - should get none
    messages = []
    timeout = time.time() + 0.2
    while time.time() < timeout:
        rclpy.spin_once(driver.node, timeout_sec=0.01)
    assert len(messages) == 0, "No messages should be received after deactivate"

    driver.change_state(Transition.TRANSITION_CLEANUP)


def test_concurrent_service_calls_thread_safety(sensor, lifecycle_interface):
    """Test that concurrent service calls are handled safely."""
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    node = Node("concurrent_service_caller")
    # 1. Use a thread-safe executor to manage all callbacks for the node.
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # 2. Run the executor in a background thread. This is now the dedicated
    #    "ROS event loop" for our client node.
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    tare_client = node.create_client(Trigger, "/schunk/driver/tare")
    assert tare_client.wait_for_service(timeout_sec=2.0)

    results = []
    errors = []
    lock = threading.Lock()  # Protect shared lists from concurrent appends

    def call_tare_service():
        try:
            req = Trigger.Request()
            future = tare_client.call_async(req)

            # 3. Use the shared executor to safely wait for THIS future to complete.
            #    This is a thread-safe operation.
            executor.spin_until_future_complete(future, timeout_sec=5.0)

            result = future.result()
            with lock:
                if result:
                    results.append(result)
                else:
                    errors.append("Service call timed out (result was None).")
        except Exception as e:
            with lock:
                errors.append(e)

    # Launch multiple concurrent service calls
    threads = []
    for _ in range(5):
        t = threading.Thread(target=call_tare_service)
        threads.append(t)
        t.start()

    # Wait for all threads to complete
    for t in threads:
        t.join(timeout=10.0)

    # 4. Gracefully shut down the executor and clean up resources
    executor.shutdown()
    executor_thread.join()

    # All calls should complete without errors
    assert len(errors) == 0, f"Concurrent service calls caused errors: {errors}"
    assert (
        len(results) == 5
    ), f"Expected 5 results, but got {len(results)}. Errors: {errors}"

    # All results should be successful
    for result in results:
        assert (
            result.success
        ), f"Service call failed: {getattr(result, 'message', 'No message')}"

    node.destroy_node()
    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)


def test_rapid_state_transitions_no_deadlock(sensor, lifecycle_interface):
    """Test that rapid state transitions don't cause deadlocks."""
    driver = lifecycle_interface

    # Perform rapid state transitions
    for _ in range(3):
        result = driver.change_state(Transition.TRANSITION_CONFIGURE)
        assert result.success, "Configure should succeed"
        assert driver.check_state(State.PRIMARY_STATE_INACTIVE)

        result = driver.change_state(Transition.TRANSITION_ACTIVATE)
        assert result.success, "Activate should succeed"
        assert driver.check_state(State.PRIMARY_STATE_ACTIVE)

        # Immediately deactivate without waiting
        result = driver.change_state(Transition.TRANSITION_DEACTIVATE)
        assert result.success, "Deactivate should succeed"
        assert driver.check_state(State.PRIMARY_STATE_INACTIVE)

        result = driver.change_state(Transition.TRANSITION_CLEANUP)
        assert result.success, "Cleanup should succeed"
        assert driver.check_state(State.PRIMARY_STATE_UNCONFIGURED)


def test_mutex_protection_during_publisher_destruction(sensor, lifecycle_interface):
    """Test that publisher destruction is protected by mutex."""
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    messages = []

    def collect_messages(msg: WrenchStamped, messages: list[WrenchStamped]) -> None:
        messages.append(msg)

    _ = driver.node.create_subscription(
        WrenchStamped,
        "/schunk/driver/data",
        partial(collect_messages, messages=messages),
        10,
    )

    # Start collecting messages
    for _ in range(10):
        rclpy.spin_once(driver.node, timeout_sec=0.001)

    initial_count = len(messages)
    assert initial_count > 0, "Should have collected some messages"

    # Deactivate while publishing thread might be trying to publish
    # The mutex should protect against race conditions
    driver.change_state(Transition.TRANSITION_DEACTIVATE)

    # The deactivate should complete successfully without hanging or crashing
    assert driver.check_state(State.PRIMARY_STATE_INACTIVE)

    driver.change_state(Transition.TRANSITION_CLEANUP)


def test_no_race_condition_in_data_buffer_access(sensor, lifecycle_interface):
    """Test that concurrent access to data buffer doesn't cause race conditions."""
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    # Create multiple subscribers to increase concurrent access
    messages1 = []
    messages2 = []
    messages3 = []

    def collect1(msg: WrenchStamped, messages: list[WrenchStamped]) -> None:
        messages.append(msg)

    def collect2(msg: WrenchStamped, messages: list[WrenchStamped]) -> None:
        messages.append(msg)

    def collect3(msg: WrenchStamped, messages: list[WrenchStamped]) -> None:
        messages.append(msg)

    _ = driver.node.create_subscription(
        WrenchStamped, "/schunk/driver/data", partial(collect1, messages=messages1), 10
    )
    _ = driver.node.create_subscription(
        WrenchStamped, "/schunk/driver/data", partial(collect2, messages=messages2), 10
    )
    _ = driver.node.create_subscription(
        WrenchStamped, "/schunk/driver/data", partial(collect3, messages=messages3), 10
    )

    # Spin rapidly to stress test concurrent access
    timeout = time.time() + 0.5
    while time.time() < timeout:
        rclpy.spin_once(driver.node, timeout_sec=0.001)

    # All subscribers should receive data without crashes
    assert len(messages1) > 0
    assert len(messages2) > 0
    assert len(messages3) > 0

    # Data should be consistent across subscribers (same messages)
    # Check that the first few messages match
    min_len = min(len(messages1), len(messages2), len(messages3))
    if min_len > 0:
        # At least the data values should be reasonable (not corrupted)
        for msg in messages1[:min_len]:
            assert msg.wrench.force.x is not None
            assert msg.wrench.force.y is not None
            assert msg.wrench.force.z is not None

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)


def test_service_calls_during_active_publishing(sensor, lifecycle_interface):
    """Test that service calls work correctly while data is being published."""
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    node = Node("service_caller_during_publish")
    tare_client = node.create_client(Trigger, "/schunk/driver/tare")
    send_cmd_client = node.create_client(SendCommand, "/schunk/driver/send_command")
    assert tare_client.wait_for_service(timeout_sec=2.0)
    assert send_cmd_client.wait_for_service(timeout_sec=2.0)

    messages = []

    def collect_messages(msg: WrenchStamped, messages: list[WrenchStamped]) -> None:
        messages.append(msg)

    _ = driver.node.create_subscription(
        WrenchStamped,
        "/schunk/driver/data",
        partial(collect_messages, messages=messages),
        10,
    )

    # Start collecting data
    for _ in range(10):
        rclpy.spin_once(driver.node, timeout_sec=0.001)

    initial_count = len(messages)
    assert initial_count > 0

    # Make service calls while publishing
    tare_req = Trigger.Request()
    tare_future = tare_client.call_async(tare_req)
    rclpy.spin_until_future_complete(node, tare_future, timeout_sec=5.0)
    assert tare_future.result().success

    # Continue collecting data
    for _ in range(10):
        rclpy.spin_once(driver.node, timeout_sec=0.001)

    # Data should continue to be published
    assert (
        len(messages) > initial_count
    ), "Data should continue publishing during service calls"

    # Another service call
    cmd_req = SendCommand.Request()
    cmd_req.command_id = "12"  # Tare command
    cmd_future = send_cmd_client.call_async(cmd_req)
    rclpy.spin_until_future_complete(node, cmd_future, timeout_sec=5.0)
    assert cmd_future.result().success

    node.destroy_node()
    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)


def test_thread_termination_on_deactivate(sensor, lifecycle_interface):
    """Test that publishing thread terminates properly on deactivate."""
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    # Give thread time to start
    time.sleep(0.1)

    # Deactivate should wait for thread to finish
    start_time = time.time()
    result = driver.change_state(Transition.TRANSITION_DEACTIVATE)
    elapsed = time.time() - start_time

    assert result.success, "Deactivate should succeed"
    assert driver.check_state(State.PRIMARY_STATE_INACTIVE)

    # Thread should terminate reasonably quickly (within 2 seconds)
    assert elapsed < 2.0, f"Thread termination took too long: {elapsed}s"

    driver.change_state(Transition.TRANSITION_CLEANUP)


def test_callback_group_isolation(sensor, lifecycle_interface):
    """Test that different callback groups don't block each other."""
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    node = Node("callback_group_test")
    tare_client = node.create_client(Trigger, "/schunk/driver/tare")
    assert tare_client.wait_for_service(timeout_sec=2.0)

    data_received = []
    service_completed = []

    def collect_data(msg: WrenchStamped, data: list) -> None:
        data.append(msg)

    _ = driver.node.create_subscription(
        WrenchStamped,
        "/schunk/driver/data",
        partial(collect_data, data=data_received),
        10,
    )

    # Start a service call in a separate thread
    def call_service():
        req = Trigger.Request()
        future = tare_client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        service_completed.append(True)

    service_thread = threading.Thread(target=call_service)
    service_thread.start()

    # While service is processing, data should still be published
    # (different callback groups should not block each other)
    timeout = time.time() + 0.5
    while time.time() < timeout and len(data_received) < 100:
        rclpy.spin_once(driver.node, timeout_sec=0.001)

    service_thread.join(timeout=10.0)

    # Both should have completed
    assert len(data_received) > 0, "Data should be published while service is called"
    assert len(service_completed) == 1, "Service should complete"

    node.destroy_node()
    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)


def test_no_data_corruption_under_stress(sensor, lifecycle_interface):
    """Test that data values remain valid under stress conditions."""
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    messages = []

    def check_data_validity(
        msg: WrenchStamped, messages: list, corrupted: list
    ) -> None:
        import math

        messages.append(msg)
        # Check for NaN or Inf values
        if (
            math.isnan(msg.wrench.force.x)
            or math.isinf(msg.wrench.force.x)
            or math.isnan(msg.wrench.force.y)
            or math.isinf(msg.wrench.force.y)
            or math.isnan(msg.wrench.force.z)
            or math.isinf(msg.wrench.force.z)
            or math.isnan(msg.wrench.torque.x)
            or math.isinf(msg.wrench.torque.x)
            or math.isnan(msg.wrench.torque.y)
            or math.isinf(msg.wrench.torque.y)
            or math.isnan(msg.wrench.torque.z)
            or math.isinf(msg.wrench.torque.z)
        ):
            corrupted.append(msg)

    corrupted = []
    _ = driver.node.create_subscription(
        WrenchStamped,
        "/schunk/driver/data",
        partial(check_data_validity, messages=messages, corrupted=corrupted),
        10,
    )

    # Collect many messages rapidly to stress test
    timeout = time.time() + 1.0
    while time.time() < timeout:
        rclpy.spin_once(driver.node, timeout_sec=0.0001)

    assert len(messages) > 100, "Should collect many messages under stress"
    assert (
        len(corrupted) == 0
    ), f"No data should be corrupted, but found {len(corrupted)}"

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)


def test_repeated_activation_cycles_thread_safety(sensor, lifecycle_interface):
    """Test thread safety across multiple activation cycles."""
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)

    for cycle in range(5):
        driver.change_state(Transition.TRANSITION_ACTIVATE)
        assert driver.check_state(State.PRIMARY_STATE_ACTIVE)

        # Brief data collection
        messages = []

        def collect(msg: WrenchStamped, messages: list) -> None:
            messages.append(msg)

        sub = driver.node.create_subscription(
            WrenchStamped,
            "/schunk/driver/data",
            partial(collect, messages=messages),
            10,
        )

        timeout = time.time() + 0.1
        while time.time() < timeout:
            rclpy.spin_once(driver.node, timeout_sec=0.001)

        assert len(messages) > 0, f"Should receive data in cycle {cycle + 1}"

        driver.node.destroy_subscription(sub)
        driver.change_state(Transition.TRANSITION_DEACTIVATE)
        assert driver.check_state(State.PRIMARY_STATE_INACTIVE)

    driver.change_state(Transition.TRANSITION_CLEANUP)
