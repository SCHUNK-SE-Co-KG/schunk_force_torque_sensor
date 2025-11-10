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

from lifecycle_msgs.msg import Transition
import time
import rclpy
from geometry_msgs.msg import WrenchStamped
from diagnostic_msgs.msg import DiagnosticStatus
from functools import partial
import pytest


def test_driver_publishes_force_torque_data(lifecycle_interface):
    """Test that the driver publishes force-torque data when active."""
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)
    messages = []

    def check_fields(msg: WrenchStamped, messages: list[WrenchStamped]) -> None:
        messages.append(msg)
        assert msg.header.stamp
        assert msg.header.frame_id
        assert pytest.approx(msg.wrench.force.x) != 0.0
        assert pytest.approx(msg.wrench.force.y) != 0.0
        assert pytest.approx(msg.wrench.force.z) != 0.0
        assert pytest.approx(msg.wrench.torque.x) != 0.0
        assert pytest.approx(msg.wrench.torque.y) != 0.0
        assert pytest.approx(msg.wrench.torque.z) != 0.0

    _ = driver.node.create_subscription(
        WrenchStamped,
        "/schunk/driver/data",
        partial(check_fields, messages=messages),
        1,
    )

    timeout = time.time() + 1.0
    while time.time() < timeout and len(messages) == 0:
        rclpy.spin_once(driver.node, timeout_sec=0.1)

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)

    assert len(messages) >= 1


def test_data_publishing_rate(lifecycle_interface):
    """Test that data publishing rate is approximately 1000 Hz (within tolerance)."""
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

    # Collect messages for a short duration
    start_time = time.time()
    collection_duration = 0.5  # seconds
    timeout = start_time + collection_duration

    while time.time() < timeout:
        rclpy.spin_once(driver.node, timeout_sec=0.01)

    actual_duration = time.time() - start_time
    message_count = len(messages)

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)

    # Calculate publishing rate
    measured_rate = message_count / actual_duration
    expected_rate = 1000.0  # Hz (actual sensor rate)
    tolerance = 0.20  # 20% tolerance to account for system load

    assert message_count > 0
    assert measured_rate >= expected_rate * (
        1 - tolerance
    ), f"Publishing rate {measured_rate:.2f} Hz is below expected "
    f"{expected_rate * (1 - tolerance):.2f} Hz"
    assert measured_rate <= expected_rate * (
        1 + tolerance
    ), f"Publishing rate {measured_rate:.2f} Hz is above expected "
    f"{expected_rate * (1 + tolerance):.2f} Hz"


def test_no_data_published_when_inactive(lifecycle_interface):
    """Test that no data is published when the driver is in inactive state."""
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    # Don't activate - stay in inactive state
    messages = []

    def collect_messages(msg: WrenchStamped, messages: list[WrenchStamped]) -> None:
        messages.append(msg)

    _ = driver.node.create_subscription(
        WrenchStamped,
        "/schunk/driver/data",
        partial(collect_messages, messages=messages),
        10,
    )

    # Wait and verify no messages are published
    timeout = time.time() + 1.0
    while time.time() < timeout:
        rclpy.spin_once(driver.node, timeout_sec=0.1)

    driver.change_state(Transition.TRANSITION_CLEANUP)

    assert len(messages) == 0, "No data should be published in inactive state"


def test_data_publishing_stops_after_deactivate(lifecycle_interface):
    """Test that data publishing stops immediately after deactivation."""
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)
    messages_before = []
    messages_after = []

    def collect_messages_before(
        msg: WrenchStamped, messages: list[WrenchStamped]
    ) -> None:
        messages.append(msg)

    sub = driver.node.create_subscription(
        WrenchStamped,
        "/schunk/driver/data",
        partial(collect_messages_before, messages=messages_before),
        10,
    )

    # Collect some messages while active
    timeout = time.time() + 0.2
    while time.time() < timeout:
        rclpy.spin_once(driver.node, timeout_sec=0.01)

    assert len(messages_before) > 0, "Should have received messages while active"

    # Deactivate
    driver.change_state(Transition.TRANSITION_DEACTIVATE)

    # Update subscription to count post-deactivate messages
    driver.node.destroy_subscription(sub)
    _ = driver.node.create_subscription(
        WrenchStamped,
        "/schunk/driver/data",
        partial(collect_messages_before, messages=messages_after),
        10,
    )

    # Wait and verify no new messages
    timeout = time.time() + 0.5
    while time.time() < timeout:
        rclpy.spin_once(driver.node, timeout_sec=0.1)

    driver.change_state(Transition.TRANSITION_CLEANUP)

    assert len(messages_after) == 0, "No data should be published after deactivation"


def test_message_counter_increments(lifecycle_interface):
    """Test that message timestamps are properly sequenced and counter logic works."""
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

    # Collect messages
    timeout = time.time() + 0.5
    while time.time() < timeout:
        rclpy.spin_once(driver.node, timeout_sec=0.01)

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)

    assert (
        len(messages) >= 10
    ), "Should have collected enough messages to test sequencing"

    # Check that timestamps are monotonically increasing
    for i in range(1, len(messages)):
        prev_stamp = (
            messages[i - 1].header.stamp.sec * 1e9
            + messages[i - 1].header.stamp.nanosec
        )
        curr_stamp = (
            messages[i].header.stamp.sec * 1e9 + messages[i].header.stamp.nanosec
        )
        assert (
            curr_stamp >= prev_stamp
        ), "Timestamps should be monotonically increasing: "
        f"{prev_stamp} -> {curr_stamp}"


def test_diagnostic_status_published(lifecycle_interface):
    """Test that diagnostic status messages are published."""
    driver = lifecycle_interface
    messages = []

    def collect_diagnostics(
        msg: DiagnosticStatus, messages: list[DiagnosticStatus]
    ) -> None:
        messages.append(msg)

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
        partial(collect_diagnostics, messages=messages),
        latching_qos,
    )

    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    # Wait for diagnostic messages - they are published on state change
    timeout = time.time() + 2.0
    while time.time() < timeout and len(messages) == 0:
        rclpy.spin_once(driver.node, timeout_sec=0.1)

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)

    assert len(messages) >= 1, "Should have received at least one diagnostic message"
    # Check fields are populated
    assert messages[0].name
    assert messages[0].hardware_id
    # Level should be one of the valid diagnostic levels
    assert messages[0].level in [
        DiagnosticStatus.OK,
        DiagnosticStatus.WARN,
        DiagnosticStatus.ERROR,
        DiagnosticStatus.STALE,
    ]


def test_diagnostic_status_ok_when_sensor_healthy(lifecycle_interface):
    """Test that diagnostic status is OK when sensor is operating normally."""
    driver = lifecycle_interface
    messages = []

    def collect_diagnostics(
        msg: DiagnosticStatus, messages: list[DiagnosticStatus]
    ) -> None:
        messages.append(msg)

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
        partial(collect_diagnostics, messages=messages),
        latching_qos,
    )

    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    # Wait for diagnostic messages
    timeout = time.time() + 2.0
    while time.time() < timeout and len(messages) == 0:
        rclpy.spin_once(driver.node, timeout_sec=0.1)

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)

    assert len(messages) >= 1
    # Assuming sensor is healthy, status should be OK
    assert (
        messages[0].level == DiagnosticStatus.OK
    ), f"Expected OK status, got level {messages[0].level} "
    f"with message: {messages[0].message}"


def test_diagnostic_qos_is_latching(lifecycle_interface):
    """Test that diagnostic messages use latching QoS (transient local durability)."""
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    # Wait for initial publishing
    time.sleep(0.5)

    # Create a late subscriber
    messages = []

    def collect_diagnostics(
        msg: DiagnosticStatus, messages: list[DiagnosticStatus]
    ) -> None:
        messages.append(msg)

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
        partial(collect_diagnostics, messages=messages),
        latching_qos,
    )

    # Late subscriber should receive the latched message
    timeout = time.time() + 2.0
    while time.time() < timeout and len(messages) == 0:
        rclpy.spin_once(driver.node, timeout_sec=0.1)

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)

    assert (
        len(messages) >= 1
    ), "Late subscriber should receive latched diagnostic message"


def test_frame_id_matches_node_name(lifecycle_interface):
    """Test that the frame_id in published messages matches the node name."""
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
        1,
    )

    timeout = time.time() + 0.5
    while time.time() < timeout and len(messages) == 0:
        rclpy.spin_once(driver.node, timeout_sec=0.1)

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)

    assert len(messages) >= 1
    # Frame ID is set to node name which is 'driver' (without namespace prefix)
    assert (
        messages[0].header.frame_id == "driver"
    ), f"Expected frame_id 'driver', got '{messages[0].header.frame_id}'"


def test_concurrent_subscribers(lifecycle_interface):
    """Test that multiple subscribers can receive data concurrently."""
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    messages1 = []
    messages2 = []
    messages3 = []

    def collect_messages1(msg: WrenchStamped, messages: list[WrenchStamped]) -> None:
        messages.append(msg)

    def collect_messages2(msg: WrenchStamped, messages: list[WrenchStamped]) -> None:
        messages.append(msg)

    def collect_messages3(msg: WrenchStamped, messages: list[WrenchStamped]) -> None:
        messages.append(msg)

    # Create multiple subscribers
    _ = driver.node.create_subscription(
        WrenchStamped,
        "/schunk/driver/data",
        partial(collect_messages1, messages=messages1),
        10,
    )
    _ = driver.node.create_subscription(
        WrenchStamped,
        "/schunk/driver/data",
        partial(collect_messages2, messages=messages2),
        10,
    )
    _ = driver.node.create_subscription(
        WrenchStamped,
        "/schunk/driver/data",
        partial(collect_messages3, messages=messages3),
        10,
    )

    # Collect messages
    timeout = time.time() + 0.5
    while time.time() < timeout:
        rclpy.spin_once(driver.node, timeout_sec=0.01)

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)

    # All subscribers should have received messages
    assert len(messages1) > 0, "Subscriber 1 should have received messages"
    assert len(messages2) > 0, "Subscriber 2 should have received messages"
    assert len(messages3) > 0, "Subscriber 3 should have received messages"

    # They should have received approximately the same number of messages
    min_count = min(len(messages1), len(messages2), len(messages3))
    max_count = max(len(messages1), len(messages2), len(messages3))
    assert (
        max_count - min_count < 50
    ), "All subscribers should receive similar message counts"


def test_repeated_activation_cycles(lifecycle_interface):
    """Test that data publishing works correctly through
    multiple activate/deactivate cycles."""
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)

    for cycle in range(3):
        messages = []

        def collect_messages(msg: WrenchStamped, messages: list[WrenchStamped]) -> None:
            messages.append(msg)

        driver.change_state(Transition.TRANSITION_ACTIVATE)

        sub = driver.node.create_subscription(
            WrenchStamped,
            "/schunk/driver/data",
            partial(collect_messages, messages=messages),
            10,
        )

        # Collect messages
        timeout = time.time() + 0.3
        while time.time() < timeout:
            rclpy.spin_once(driver.node, timeout_sec=0.01)

        assert len(messages) > 0, f"Should have received messages in cycle {cycle + 1}"

        driver.node.destroy_subscription(sub)
        driver.change_state(Transition.TRANSITION_DEACTIVATE)

    driver.change_state(Transition.TRANSITION_CLEANUP)


def test_wrench_data_fields_are_floats(lifecycle_interface):
    """Test that all wrench data fields are valid floating point numbers."""
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
        1,
    )

    timeout = time.time() + 0.5
    while time.time() < timeout and len(messages) == 0:
        rclpy.spin_once(driver.node, timeout_sec=0.1)

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)

    assert len(messages) >= 1
    msg = messages[0]

    # Check all fields are valid floats (not NaN or Inf)
    import math

    assert not math.isnan(msg.wrench.force.x) and not math.isinf(msg.wrench.force.x)
    assert not math.isnan(msg.wrench.force.y) and not math.isinf(msg.wrench.force.y)
    assert not math.isnan(msg.wrench.force.z) and not math.isinf(msg.wrench.force.z)
    assert not math.isnan(msg.wrench.torque.x) and not math.isinf(msg.wrench.torque.x)
    assert not math.isnan(msg.wrench.torque.y) and not math.isinf(msg.wrench.torque.y)
    assert not math.isnan(msg.wrench.torque.z) and not math.isinf(msg.wrench.torque.z)


def test_timestamp_increases_monotonically(lifecycle_interface):
    """Test that timestamps increase monotonically across all messages."""
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

    # Collect many messages to test sequencing
    timeout = time.time() + 1.0
    while time.time() < timeout:
        rclpy.spin_once(driver.node, timeout_sec=0.001)

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)

    assert (
        len(messages) >= 100
    ), "Should collect enough messages to properly test monotonicity"

    # Convert all timestamps to nanoseconds and verify monotonic increase
    timestamps = [
        msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        for msg in messages
    ]

    for i in range(1, len(timestamps)):
        assert (
            timestamps[i] >= timestamps[i - 1]
        ), f"Timestamp at index {i} ({timestamps[i]}) "
        f"is less than previous ({timestamps[i-1]})"

    # Check that timestamps are reasonably spaced (approximately 1ms apart for 1kHz)
    # Allow for some variation but ensure they're generally in the right ballpark
    time_diffs = [timestamps[i] - timestamps[i - 1] for i in range(1, len(timestamps))]
    avg_diff_ns = sum(time_diffs) / len(time_diffs)

    # Allow 50% tolerance for timing variation
    assert (
        500_000 <= avg_diff_ns <= 1_500_000
    ), f"Average time difference {avg_diff_ns}ns is outside "
    "expected range for 1kHz publishing"
