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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from lifecycle_msgs.msg import Transition, State
from geometry_msgs.msg import WrenchStamped
from diagnostic_msgs.msg import DiagnosticStatus
from example_interfaces.srv import Trigger
from functools import partial
import time
import threading


def test_complete_startup_to_shutdown_cycle(sensor, lifecycle_interface):
    """Test complete lifecycle from startup to shutdown."""
    driver = lifecycle_interface

    # Initial state
    assert driver.check_state(State.PRIMARY_STATE_UNCONFIGURED)

    # Configure
    result = driver.change_state(Transition.TRANSITION_CONFIGURE)
    assert result.success
    assert driver.check_state(State.PRIMARY_STATE_INACTIVE)

    # Activate
    result = driver.change_state(Transition.TRANSITION_ACTIVATE)
    assert result.success
    assert driver.check_state(State.PRIMARY_STATE_ACTIVE)

    # Verify data flow
    messages = []

    def collect(msg: WrenchStamped, messages: list) -> None:
        messages.append(msg)

    _ = driver.node.create_subscription(
        WrenchStamped, "/schunk/driver/data", partial(collect, messages=messages), 10
    )

    timeout = time.time() + 0.5
    while time.time() < timeout and len(messages) < 10:
        rclpy.spin_once(driver.node, timeout_sec=0.01)

    assert len(messages) >= 10, "Should receive data flow"

    # Deactivate
    result = driver.change_state(Transition.TRANSITION_DEACTIVATE)
    assert result.success
    assert driver.check_state(State.PRIMARY_STATE_INACTIVE)

    # Cleanup
    result = driver.change_state(Transition.TRANSITION_CLEANUP)
    assert result.success
    assert driver.check_state(State.PRIMARY_STATE_UNCONFIGURED)


def test_end_to_end_data_flow(sensor, lifecycle_interface):
    """Test end-to-end data flow from sensor to ROS topic."""
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    received_data = []
    timestamps = []

    def analyze_data(msg: WrenchStamped, data: list, timestamps: list) -> None:
        data.append(
            {
                "fx": msg.wrench.force.x,
                "fy": msg.wrench.force.y,
                "fz": msg.wrench.force.z,
                "tx": msg.wrench.torque.x,
                "ty": msg.wrench.torque.y,
                "tz": msg.wrench.torque.z,
            }
        )
        timestamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        timestamps.append(timestamp_ns)

    _ = driver.node.create_subscription(
        WrenchStamped,
        "/schunk/driver/data",
        partial(analyze_data, data=received_data, timestamps=timestamps),
        10,
    )

    # Collect data for 1 second
    timeout = time.time() + 1.0
    while time.time() < timeout:
        rclpy.spin_once(driver.node, timeout_sec=0.001)

    # Verify data quality
    assert len(received_data) > 100, "Should receive substantial amount of data"

    # Check data completeness
    for data in received_data:
        assert "fx" in data
        assert "fy" in data
        assert "fz" in data
        assert "tx" in data
        assert "ty" in data
        assert "tz" in data

    # Check timestamps are monotonic
    for i in range(1, len(timestamps)):
        assert timestamps[i] >= timestamps[i - 1], "Timestamps should be monotonic"

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)


def test_system_behavior_under_load(sensor, lifecycle_interface):
    """Test system behavior under high load conditions."""
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    # Create multiple subscribers to increase load
    messages_lists = [[] for _ in range(5)]

    def collect_factory(index: int, msg: WrenchStamped) -> None:
        messages_lists[index].append(msg)

    # Create 5 subscribers
    for i in range(5):
        _ = driver.node.create_subscription(
            WrenchStamped,
            "/schunk/driver/data",
            partial(collect_factory, i),
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
            ),
        )

    # Also make periodic service calls to add load
    load_node = Node("load_test_service_caller")
    tare_cli = load_node.create_client(Trigger, "/schunk/driver/tare")
    assert tare_cli.wait_for_service(timeout_sec=5.0)

    service_results = []

    # Create an executor for the load_node
    executor = MultiThreadedExecutor()
    executor.add_node(load_node)

    # Thread to spin the executor
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    def periodic_service_calls():
        for _ in range(5):
            req = Trigger.Request()
            future = tare_cli.call_async(req)
            rclpy.spin_until_future_complete(
                load_node, future, executor=executor, timeout_sec=5.0
            )

            if future.result():
                service_results.append(future.result())
            time.sleep(0.2)
        print("Periodic service calls finished.")

    service_thread = threading.Thread(target=periodic_service_calls)
    service_thread.start()

    # Spin rapidly for 2 seconds
    timeout = time.time() + 2.0
    while time.time() < timeout:
        rclpy.spin_once(driver.node, timeout_sec=0.0001)

    service_thread.join(timeout=10.0)
    executor.shutdown()
    load_node.destroy_node()

    # Verify system handled the load
    for i, msgs in enumerate(messages_lists):
        assert (
            len(msgs) > 500
        ), f"Subscriber {i} should have received many messages under load"

    # All service calls should have completed
    assert len(service_results) == 5, "All service calls should complete under load"
    for result in service_results:
        assert result.success, "Service calls should succeed under load"

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)


def test_multiple_complete_cycles(sensor, lifecycle_interface):
    """Test multiple complete startup-to-shutdown cycles."""
    driver = lifecycle_interface

    for cycle in range(3):
        # Configure
        result = driver.change_state(Transition.TRANSITION_CONFIGURE)
        assert result.success, f"Configure failed in cycle {cycle + 1}"

        # Activate
        result = driver.change_state(Transition.TRANSITION_ACTIVATE)
        assert result.success, f"Activate failed in cycle {cycle + 1}"

        # Verify data publishing
        messages = []

        def collect(msg: WrenchStamped, messages: list) -> None:
            messages.append(msg)

        sub = driver.node.create_subscription(
            WrenchStamped,
            "/schunk/driver/data",
            partial(collect, messages=messages),
            10,
        )

        timeout = time.time() + 0.3
        while time.time() < timeout and len(messages) < 50:
            rclpy.spin_once(driver.node, timeout_sec=0.001)

        assert len(messages) >= 50, f"Should receive data in cycle {cycle + 1}"

        # Make a service call
        node = Node(f"service_test_cycle_{cycle}")
        cli = node.create_client(Trigger, "/schunk/driver/tare")
        assert cli.wait_for_service(timeout_sec=2.0)
        req = Trigger.Request()
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        assert future.result().success, f"Service call failed in cycle {cycle + 1}"
        node.destroy_node()

        # Cleanup
        driver.node.destroy_subscription(sub)
        result = driver.change_state(Transition.TRANSITION_DEACTIVATE)
        assert result.success, f"Deactivate failed in cycle {cycle + 1}"

        result = driver.change_state(Transition.TRANSITION_CLEANUP)
        assert result.success, f"Cleanup failed in cycle {cycle + 1}"


def test_data_and_diagnostics_correlation(sensor, lifecycle_interface):
    """Test that data publishing and diagnostic updates are correlated properly."""
    driver = lifecycle_interface

    data_messages = []
    diagnostic_messages = []

    def collect_data(msg: WrenchStamped, messages: list) -> None:
        messages.append(msg)

    def collect_diagnostics(msg: DiagnosticStatus, messages: list) -> None:
        messages.append(msg)

    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

    latching_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

    _ = driver.node.create_subscription(
        WrenchStamped,
        "/schunk/driver/data",
        partial(collect_data, messages=data_messages),
        10,
    )

    _ = driver.node.create_subscription(
        DiagnosticStatus,
        "/schunk/driver/state",
        partial(collect_diagnostics, messages=diagnostic_messages),
        latching_qos,
    )

    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    # Collect for some time
    timeout = time.time() + 1.0
    while time.time() < timeout:
        rclpy.spin_once(driver.node, timeout_sec=0.01)

    # Should have both data and diagnostics
    assert len(data_messages) > 0, "Should receive data messages"
    assert len(diagnostic_messages) > 0, "Should receive diagnostic messages"

    # When sensor is working, diagnostics should show OK status
    ok_diagnostics = [d for d in diagnostic_messages if d.level == DiagnosticStatus.OK]
    assert (
        len(ok_diagnostics) > 0
    ), "Should have OK diagnostic status when sensor is healthy"

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)


def test_service_and_data_interleaving(sensor, lifecycle_interface):
    """Test that services and data publishing can interleave correctly."""
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    data_messages = []
    service_call_times = []

    def collect_data(msg: WrenchStamped, messages: list) -> None:
        messages.append(msg)

    _ = driver.node.create_subscription(
        WrenchStamped,
        "/schunk/driver/data",
        partial(collect_data, messages=data_messages),
        10,
    )

    node = Node("interleave_test")
    tare_cli = node.create_client(Trigger, "/schunk/driver/tare")
    assert tare_cli.wait_for_service(timeout_sec=2.0)

    # Interleave data collection with service calls
    for i in range(3):
        # Collect some data
        for _ in range(20):
            rclpy.spin_once(driver.node, timeout_sec=0.001)

        count_before_service = len(data_messages)

        # Make a service call
        start = time.time()
        req = Trigger.Request()
        future = tare_cli.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        elapsed = time.time() - start
        service_call_times.append(elapsed)

        assert future.result().success, f"Service call {i+1} should succeed"

        # Continue collecting data
        for _ in range(20):
            rclpy.spin_once(driver.node, timeout_sec=0.001)

        # Data should have continued during and after service call
        assert (
            len(data_messages) > count_before_service
        ), f"Data should continue during service call {i+1}"

    # All service calls should be reasonably fast
    for i, elapsed in enumerate(service_call_times):
        assert elapsed < 2.0, f"Service call {i+1} took too long: {elapsed}s"

    node.destroy_node()
    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)


def test_long_running_stability(sensor, lifecycle_interface):
    """Test system stability over extended operation."""
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    messages = []
    errors = []

    def collect_with_validation(
        msg: WrenchStamped, messages: list, errors: list
    ) -> None:
        try:
            import math

            # Validate data
            if math.isnan(msg.wrench.force.x) or math.isinf(msg.wrench.force.x):
                errors.append("Invalid force.x")
            messages.append(msg)
        except Exception as e:
            errors.append(str(e))

    _ = driver.node.create_subscription(
        WrenchStamped,
        "/schunk/driver/data",
        partial(collect_with_validation, messages=messages, errors=errors),
        10,
    )

    # Run for 3 seconds
    timeout = time.time() + 3.0
    while time.time() < timeout:
        rclpy.spin_once(driver.node, timeout_sec=0.001)

    # Should have collected many messages without errors
    assert len(messages) > 1000, f"Should collect many messages, got {len(messages)}"
    assert len(errors) == 0, f"Should have no errors during long run, got: {errors}"

    # Verify timestamps remain monotonic throughout
    prev_ns = 0
    for msg in messages:
        curr_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        assert curr_ns >= prev_ns, "Timestamps should remain monotonic"
        prev_ns = curr_ns

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)


def test_resource_cleanup_verification(sensor, lifecycle_interface):
    """Test that resources are properly cleaned up after shutdown."""
    driver = lifecycle_interface

    # Go through a complete cycle
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    # Collect some data
    messages = []

    def collect(msg: WrenchStamped, messages: list) -> None:
        messages.append(msg)

    sub = driver.node.create_subscription(
        WrenchStamped, "/schunk/driver/data", partial(collect, messages=messages), 10
    )

    timeout = time.time() + 0.3
    while time.time() < timeout:
        rclpy.spin_once(driver.node, timeout_sec=0.001)

    assert len(messages) > 0

    # Shutdown
    driver.node.destroy_subscription(sub)
    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)

    # Verify we can start again (resources were cleaned up)
    result = driver.change_state(Transition.TRANSITION_CONFIGURE)
    assert result.success, "Should be able to configure again after cleanup"

    result = driver.change_state(Transition.TRANSITION_ACTIVATE)
    assert result.success, "Should be able to activate again after cleanup"

    # Verify data flows again
    messages2 = []

    def collect2(msg: WrenchStamped, messages: list) -> None:
        messages.append(msg)

    _ = driver.node.create_subscription(
        WrenchStamped, "/schunk/driver/data", partial(collect2, messages=messages2), 10
    )

    timeout = time.time() + 0.3
    while time.time() < timeout and len(messages2) < 10:
        rclpy.spin_once(driver.node, timeout_sec=0.001)

    assert len(messages2) >= 10, "Data should flow after restart"

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)


def test_concurrent_operations_integration(sensor, lifecycle_interface):
    """Test concurrent data publishing, service calls, and state monitoring."""
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    data_messages = []
    diagnostic_messages = []
    service_results = []

    def collect_data(msg: WrenchStamped, messages: list) -> None:
        messages.append(msg)

    def collect_diagnostics(msg: DiagnosticStatus, messages: list) -> None:
        messages.append(msg)

    latching_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

    _ = driver.node.create_subscription(
        WrenchStamped,
        "/schunk/driver/data",
        partial(collect_data, messages=data_messages),
        10,
    )

    _ = driver.node.create_subscription(
        DiagnosticStatus,
        "/schunk/driver/state",
        partial(collect_diagnostics, messages=diagnostic_messages),
        latching_qos,
    )

    # Use a single MultiThreadedExecutor to manage all nodes in this test.
    executor = MultiThreadedExecutor()

    # Create the node for making service calls
    service_node = Node("concurrent_ops_test")
    cli = service_node.create_client(Trigger, "/schunk/driver/tare")

    # Add ALL nodes to the executor
    executor.add_node(driver.node)
    executor.add_node(service_node)

    # Run the executor in its own thread. It will handle all callbacks.
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Now that the executor is running, we can safely wait for the service.
    assert cli.wait_for_service(timeout_sec=5.0)

    def make_service_calls():
        for _ in range(3):
            req = Trigger.Request()
            future = cli.call_async(req)
            executor.spin_until_future_complete(future, timeout_sec=5.0)

            if future.result():
                service_results.append(future.result())
            time.sleep(0.3)

    service_thread = threading.Thread(target=make_service_calls)
    service_thread.start()
    service_thread.join(timeout=10.0)

    # Gracefully shut down the executor and clean up resources
    executor.shutdown()
    executor_thread.join(timeout=5.0)  # Ensure the executor thread has finished
    service_node.destroy_node()

    # Verify all operations completed successfully
    assert len(data_messages) > 100, "Should collect substantial data"
    assert len(diagnostic_messages) > 0, "Should receive diagnostics"
    assert len(service_results) == 3, "All service calls should complete"

    for result in service_results:
        assert result.success, "Service calls should succeed"

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)
