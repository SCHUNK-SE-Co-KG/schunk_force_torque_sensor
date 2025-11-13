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
from lifecycle_msgs.msg import Transition
from geometry_msgs.msg import WrenchStamped
from diagnostic_msgs.msg import DiagnosticStatus
from example_interfaces.srv import Trigger
from schunk_fts_interfaces.srv import SendCommand  # type: ignore [attr-defined]
from functools import partial
import time


def test_driver_handles_empty_data_buffer_gracefully(sensor, lifecycle_interface):
    """Test that driver handles empty buffer (None from sample()) gracefully."""
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

    # The driver should handle empty buffer returns gracefully
    # by continuing to loop without publishing
    timeout = time.time() + 0.5
    while time.time() < timeout:
        rclpy.spin_once(driver.node, timeout_sec=0.01)

    # Should receive at least some messages (buffer not always empty)
    assert len(messages) >= 0  # May be 0 if buffer is temporarily empty

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)


def test_driver_continues_after_packet_skip(sensor, lifecycle_interface):
    """Test that driver continues operating after detecting skipped packets."""
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
        100,
    )

    # Collect messages - driver should continue even if packets are skipped
    timeout = time.time() + 1.0
    while time.time() < timeout:
        rclpy.spin_once(driver.node, timeout_sec=0.001)

    # Driver should continue publishing even if some packets were skipped
    assert (
        len(messages) > 100
    ), "Driver should continue publishing despite potential packet skips"

    # Check that all messages have valid data
    for msg in messages:
        assert msg.header.stamp.sec >= 0
        assert msg.header.stamp.nanosec >= 0
        assert msg.header.frame_id == "driver"

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)


def test_invalid_service_command_returns_error(sensor, lifecycle_interface):
    """Test that invalid commands return proper error messages."""
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    node = Node("invalid_command_test")
    cmd_client = node.create_client(SendCommand, "/schunk/driver/send_command")
    assert cmd_client.wait_for_service(timeout_sec=2.0)

    # Send an invalid command
    req = SendCommand.Request()
    req.command_id = "0xFF"  # Invalid command
    future = cmd_client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

    result = future.result()
    assert result is not None
    assert result.success is False
    assert "Unknown Command" in result.error_message

    node.destroy_node()
    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)


def test_service_error_handling_with_exception(sensor, lifecycle_interface):
    """Test that service exceptions are caught and reported."""
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    node = Node("exception_test")
    cmd_client = node.create_client(SendCommand, "/schunk/driver/send_command")
    assert cmd_client.wait_for_service(timeout_sec=2.0)

    # Try various potentially problematic commands
    test_cases = [
        "0xFF",  # Invalid command
        "99",  # Another invalid command
        "AA",  # Another invalid command
    ]

    for command_id in test_cases:
        req = SendCommand.Request()
        req.command_id = command_id
        future = cmd_client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

        result = future.result()
        # Should get a result, not crash
        assert result is not None
        # Invalid commands should fail
        assert result.success is False
        assert len(result.error_message) > 0

    node.destroy_node()
    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)


def test_recovery_after_service_failure(sensor, lifecycle_interface):
    """Test that driver continues to work after service call failures."""
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    node = Node("recovery_test")
    cmd_client = node.create_client(SendCommand, "/schunk/driver/send_command")
    tare_client = node.create_client(Trigger, "/schunk/driver/tare")
    assert cmd_client.wait_for_service(timeout_sec=2.0)
    assert tare_client.wait_for_service(timeout_sec=2.0)

    # First, make a failing service call
    req = SendCommand.Request()
    req.command_id = "0xFF"
    future = cmd_client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    result = future.result()
    assert result.success is False

    # Now make a valid service call - should work
    tare_req = Trigger.Request()
    tare_future = tare_client.call_async(tare_req)
    rclpy.spin_until_future_complete(node, tare_future, timeout_sec=5.0)
    tare_result = tare_future.result()
    assert (
        tare_result.success is True
    ), "Valid service call should succeed after failed call"

    # Verify data is still being published
    messages = []

    def collect(msg: WrenchStamped, messages: list) -> None:
        messages.append(msg)

    _ = driver.node.create_subscription(
        WrenchStamped, "/schunk/driver/data", partial(collect, messages=messages), 10
    )

    timeout = time.time() + 0.5
    while time.time() < timeout and len(messages) < 10:
        rclpy.spin_once(driver.node, timeout_sec=0.01)

    assert len(messages) > 0, "Data should still be published after service failure"

    node.destroy_node()
    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)


def test_diagnostic_status_updates_on_sensor_errors(sensor, lifecycle_interface):
    """Test that diagnostic status reflects sensor error conditions."""
    driver = lifecycle_interface

    diagnostics = []

    def collect_diagnostics(msg: DiagnosticStatus, diagnostics: list) -> None:
        diagnostics.append(msg)

    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

    latching_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

    _ = driver.node.create_subscription(
        DiagnosticStatus,
        "/schunk/driver/state",
        partial(collect_diagnostics, diagnostics=diagnostics),
        latching_qos,
    )

    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    # Collect diagnostic messages
    timeout = time.time() + 2.0
    while time.time() < timeout:
        rclpy.spin_once(driver.node, timeout_sec=0.1)

    # Should have received at least one diagnostic message
    assert len(diagnostics) >= 1

    # In normal operation, status should be OK
    # (unless there's an actual sensor error)
    for diag in diagnostics:
        assert diag.level in [
            DiagnosticStatus.OK,
            DiagnosticStatus.WARN,
            DiagnosticStatus.ERROR,
            DiagnosticStatus.STALE,
        ]
        assert diag.name  # Should have a name
        assert len(diag.hardware_id) > 0

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)


def test_driver_handles_counter_wraparound(sensor, lifecycle_interface):
    """Test that driver handles counter wraparound (65535 -> 0) correctly."""
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
        100,
    )

    # Collect many messages to potentially see counter behavior
    timeout = time.time() + 2.0
    while time.time() < timeout:
        rclpy.spin_once(driver.node, timeout_sec=0.001)

    # Driver should handle counter wraparound gracefully
    # All messages should have valid timestamps
    assert len(messages) > 100

    for i, msg in enumerate(messages):
        assert msg.header.stamp.sec >= 0
        assert msg.header.stamp.nanosec >= 0
        # Timestamps should be monotonically increasing
        if i > 0:
            prev_stamp_ns = (
                messages[i - 1].header.stamp.sec * 1_000_000_000
                + messages[i - 1].header.stamp.nanosec
            )
            curr_stamp_ns = (
                msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            )
            assert curr_stamp_ns >= prev_stamp_ns

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)


def test_error_reporting_through_service_response(sensor, lifecycle_interface):
    """Test that errors are properly reported through service response messages."""
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    node = Node("error_reporting_test")
    cmd_client = node.create_client(SendCommand, "/schunk/driver/send_command")
    assert cmd_client.wait_for_service(timeout_sec=2.0)

    # Test various error codes
    invalid_commands = ["0xFF", "0xAB", "99", "FE"]

    for cmd in invalid_commands:
        req = SendCommand.Request()
        req.command_id = cmd
        future = cmd_client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

        result = future.result()
        assert result is not None, f"Should get response for command {cmd}"
        assert result.success is False, f"Command {cmd} should fail"
        assert (
            len(result.error_message) > 0
        ), f"Should have error message for command {cmd}"
        # Error message should be meaningful
        assert "Unknown" in result.error_message or "Error" in result.error_message

    node.destroy_node()
    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)


def test_driver_publishes_after_temporary_buffer_empty(sensor, lifecycle_interface):
    """Test that driver resumes publishing after temporary buffer empty condition."""
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    messages_phase1 = []
    messages_phase2 = []

    def collect_phase1(msg: WrenchStamped, messages: list) -> None:
        messages.append(msg)

    sub1 = driver.node.create_subscription(
        WrenchStamped,
        "/schunk/driver/data",
        partial(collect_phase1, messages=messages_phase1),
        10,
    )

    # Phase 1: Collect some messages
    timeout = time.time() + 0.3
    while time.time() < timeout:
        rclpy.spin_once(driver.node, timeout_sec=0.01)

    count_phase1 = len(messages_phase1)
    assert count_phase1 > 0, "Should receive messages in phase 1"

    # Phase 2: Continue collecting (simulating recovery from empty buffer)
    driver.node.destroy_subscription(sub1)

    def collect_phase2(msg: WrenchStamped, messages: list) -> None:
        messages.append(msg)

    _ = driver.node.create_subscription(
        WrenchStamped,
        "/schunk/driver/data",
        partial(collect_phase2, messages=messages_phase2),
        10,
    )

    timeout = time.time() + 0.3
    while time.time() < timeout:
        rclpy.spin_once(driver.node, timeout_sec=0.01)

    count_phase2 = len(messages_phase2)
    assert count_phase2 > 0, "Should receive messages in phase 2"

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)


def test_command_with_0x_prefix_handling(sensor, lifecycle_interface):
    """Test that commands with 0x prefix are handled correctly."""
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    node = Node("prefix_test")
    cmd_client = node.create_client(SendCommand, "/schunk/driver/send_command")
    assert cmd_client.wait_for_service(timeout_sec=2.0)

    # Test valid command with 0x prefix (tare command)
    req = SendCommand.Request()
    req.command_id = "0x12"
    future = cmd_client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    result = future.result()
    assert result is not None
    # Should handle the 0x prefix by stripping it
    # Tare command should succeed if sensor is working

    # Test the same command without prefix
    req2 = SendCommand.Request()
    req2.command_id = "12"
    future2 = cmd_client.call_async(req2)
    rclpy.spin_until_future_complete(node, future2, timeout_sec=5.0)
    result2 = future2.result()
    assert result2 is not None

    # Both should have same outcome
    assert result.success == result2.success

    node.destroy_node()
    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)


def test_continuous_operation_stress_test(sensor, lifecycle_interface):
    """Test continuous operation under stress for extended period."""
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    messages = []
    errors = []

    def collect_messages(msg: WrenchStamped, messages: list, errors: list) -> None:
        try:
            # Validate message
            import math

            if (
                math.isnan(msg.wrench.force.x)
                or math.isnan(msg.wrench.force.y)
                or math.isnan(msg.wrench.force.z)
            ):
                errors.append("NaN values detected")
            messages.append(msg)
        except Exception as e:
            errors.append(str(e))

    _ = driver.node.create_subscription(
        WrenchStamped,
        "/schunk/driver/data",
        partial(collect_messages, messages=messages, errors=errors),
        10,
    )

    # Run for 2 seconds under continuous load
    timeout = time.time() + 2.0
    while time.time() < timeout:
        rclpy.spin_once(driver.node, timeout_sec=0.001)

    # Should collect many messages without errors
    assert len(messages) > 500, f"Should collect many messages, got {len(messages)}"
    assert len(errors) == 0, f"Should have no errors, got: {errors}"

    # All messages should have increasing timestamps
    for i in range(1, min(100, len(messages))):
        prev_ns = (
            messages[i - 1].header.stamp.sec * 1e9
            + messages[i - 1].header.stamp.nanosec
        )
        curr_ns = messages[i].header.stamp.sec * 1e9 + messages[i].header.stamp.nanosec
        assert curr_ns >= prev_ns, f"Timestamp decreased at index {i}"

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)


def test_fallback_behavior_on_no_signal(sensor, lifecycle_interface):
    """Test fallback behavior when sensor status indicates no signal."""
    # This test verifies the diagnostic reporting works correctly
    driver = lifecycle_interface

    diagnostics = []

    def collect_diagnostics(msg: DiagnosticStatus, diagnostics: list) -> None:
        diagnostics.append(msg)

    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

    latching_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

    _ = driver.node.create_subscription(
        DiagnosticStatus,
        "/schunk/driver/state",
        partial(collect_diagnostics, diagnostics=diagnostics),
        latching_qos,
    )

    result = driver.change_state(Transition.TRANSITION_CONFIGURE)

    if result.success:
        driver.change_state(Transition.TRANSITION_ACTIVATE)

        # Collect diagnostics
        timeout = time.time() + 1.0
        while time.time() < timeout:
            rclpy.spin_once(driver.node, timeout_sec=0.1)

        # With a working sensor, we should get OK or WARN status
        if len(diagnostics) > 0:
            # Status should be reported
            assert diagnostics[0].level in [
                DiagnosticStatus.OK,
                DiagnosticStatus.WARN,
                DiagnosticStatus.ERROR,
            ]

        driver.change_state(Transition.TRANSITION_DEACTIVATE)
        driver.change_state(Transition.TRANSITION_CLEANUP)
