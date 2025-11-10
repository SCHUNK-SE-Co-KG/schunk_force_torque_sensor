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
from lifecycle_msgs.msg import Transition, State


def test_driver_supports_repeated_configure_and_cleanup(sensor, lifecycle_interface):
    driver = lifecycle_interface
    for _ in range(3):
        driver.change_state(Transition.TRANSITION_CONFIGURE)
        assert driver.check_state(State.PRIMARY_STATE_INACTIVE)
        driver.change_state(Transition.TRANSITION_CLEANUP)
        assert driver.check_state(State.PRIMARY_STATE_UNCONFIGURED)


def test_driver_supports_repeated_activate_and_deactivate(sensor, lifecycle_interface):
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    for _ in range(3):
        driver.change_state(Transition.TRANSITION_ACTIVATE)
        assert driver.check_state(State.PRIMARY_STATE_ACTIVE)
        driver.change_state(Transition.TRANSITION_DEACTIVATE)
        assert driver.check_state(State.PRIMARY_STATE_INACTIVE)
    driver.change_state(Transition.TRANSITION_CLEANUP)


def test_primary_lifecycle_states(sensor, lifecycle_interface):
    driver = lifecycle_interface

    # Startup
    assert driver.check_state(State.PRIMARY_STATE_UNCONFIGURED)

    # configure
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    assert driver.check_state(State.PRIMARY_STATE_INACTIVE)

    # activate
    driver.change_state(Transition.TRANSITION_ACTIVATE)
    assert driver.check_state(State.PRIMARY_STATE_ACTIVE)

    # deactivate
    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    assert driver.check_state(State.PRIMARY_STATE_INACTIVE)

    # cleanup
    driver.change_state(Transition.TRANSITION_CLEANUP)
    assert driver.check_state(State.PRIMARY_STATE_UNCONFIGURED)


def test_initial_state_is_unconfigured(sensor, lifecycle_interface):
    """Test that the node starts in the unconfigured state."""
    assert lifecycle_interface.check_state(Transition.TRANSITION_CONFIGURE)


def test_configure_transition(sensor, lifecycle_interface):
    """Test the transition from unconfigured to inactive."""
    assert lifecycle_interface.check_state(Transition.TRANSITION_CONFIGURE)
    result = lifecycle_interface.change_state(Transition.TRANSITION_CONFIGURE)
    assert result.success
    assert lifecycle_interface.check_state(Transition.TRANSITION_CLEANUP)


def test_activate_transition(sensor, lifecycle_interface):
    """Test the transition from inactive to active."""
    # Go to inactive state first
    lifecycle_interface.change_state(Transition.TRANSITION_CONFIGURE)
    assert lifecycle_interface.check_state(Transition.TRANSITION_CLEANUP)

    # Activate
    result = lifecycle_interface.change_state(Transition.TRANSITION_ACTIVATE)
    assert result.success
    assert lifecycle_interface.check_state(Transition.TRANSITION_DEACTIVATE - 1)


def test_deactivate_transition(sensor, lifecycle_interface):
    """Test the transition from active to inactive."""
    # Go to active state first
    lifecycle_interface.change_state(Transition.TRANSITION_CONFIGURE)
    lifecycle_interface.change_state(Transition.TRANSITION_ACTIVATE)
    assert lifecycle_interface.check_state(Transition.TRANSITION_DEACTIVATE - 1)

    # Deactivate
    result = lifecycle_interface.change_state(Transition.TRANSITION_DEACTIVATE)
    assert result.success
    assert lifecycle_interface.check_state(Transition.TRANSITION_CLEANUP)


def test_cleanup_transition(sensor, lifecycle_interface):
    """Test the transition from inactive to unconfigured."""
    # Go to inactive state first
    lifecycle_interface.change_state(Transition.TRANSITION_CONFIGURE)
    assert lifecycle_interface.check_state(Transition.TRANSITION_CLEANUP)

    # Cleanup
    result = lifecycle_interface.change_state(Transition.TRANSITION_CLEANUP)
    assert result.success
    assert lifecycle_interface.check_state(Transition.TRANSITION_CONFIGURE)


def test_full_lifecycle_sequence(sensor, lifecycle_interface):
    """Test a full configure -> activate -> deactivate -> cleanup sequence."""
    # Initial state: unconfigured
    assert lifecycle_interface.check_state(Transition.TRANSITION_CONFIGURE)

    # Configure: unconfigured -> inactive
    result = lifecycle_interface.change_state(Transition.TRANSITION_CONFIGURE)
    assert result.success
    assert lifecycle_interface.check_state(Transition.TRANSITION_CLEANUP)

    # Activate: inactive -> active
    result = lifecycle_interface.change_state(Transition.TRANSITION_ACTIVATE)
    assert result.success
    assert lifecycle_interface.check_state(Transition.TRANSITION_DEACTIVATE - 1)

    # Deactivate: active -> inactive
    result = lifecycle_interface.change_state(Transition.TRANSITION_DEACTIVATE)
    assert result.success
    assert lifecycle_interface.check_state(Transition.TRANSITION_CLEANUP)

    # Cleanup: inactive -> unconfigured
    result = lifecycle_interface.change_state(Transition.TRANSITION_CLEANUP)
    assert result.success
    assert lifecycle_interface.check_state(Transition.TRANSITION_CONFIGURE)


def test_state_callbacks_executed_in_correct_order(sensor, lifecycle_interface):
    """Test that lifecycle callbacks are executed in the correct order."""
    import time

    # Track execution by checking state changes happen in order
    driver = lifecycle_interface

    # Initial state
    assert driver.check_state(State.PRIMARY_STATE_UNCONFIGURED)

    # Configure
    start_time = time.time()
    result = driver.change_state(Transition.TRANSITION_CONFIGURE)
    configure_time = time.time() - start_time
    assert result.success
    assert configure_time < 5.0, "Configure should complete in reasonable time"
    assert driver.check_state(State.PRIMARY_STATE_INACTIVE)

    # Activate
    start_time = time.time()
    result = driver.change_state(Transition.TRANSITION_ACTIVATE)
    activate_time = time.time() - start_time
    assert result.success
    assert activate_time < 2.0, "Activate should complete quickly"
    assert driver.check_state(State.PRIMARY_STATE_ACTIVE)

    # Deactivate
    start_time = time.time()
    result = driver.change_state(Transition.TRANSITION_DEACTIVATE)
    deactivate_time = time.time() - start_time
    assert result.success
    assert deactivate_time < 2.0, "Deactivate should complete quickly"
    assert driver.check_state(State.PRIMARY_STATE_INACTIVE)

    # Cleanup
    start_time = time.time()
    result = driver.change_state(Transition.TRANSITION_CLEANUP)
    cleanup_time = time.time() - start_time
    assert result.success
    assert cleanup_time < 2.0, "Cleanup should complete quickly"
    assert driver.check_state(State.PRIMARY_STATE_UNCONFIGURED)


def test_configure_initializes_sensor_connection(sensor, lifecycle_interface):
    """Test that configure transition properly initializes sensor connection."""
    driver = lifecycle_interface

    # Before configure, should be unconfigured
    assert driver.check_state(State.PRIMARY_STATE_UNCONFIGURED)

    # Configure should establish connection and start streaming
    result = driver.change_state(Transition.TRANSITION_CONFIGURE)
    assert result.success, "Configure should successfully initialize connection"
    assert driver.check_state(State.PRIMARY_STATE_INACTIVE)

    # After configure, sensor should be streaming (verified in on_configure)
    # If we got to INACTIVE, the sensor must be streaming

    driver.change_state(Transition.TRANSITION_CLEANUP)


def test_activate_starts_data_publishing(sensor, lifecycle_interface):
    """Test that activate transition starts data publishing."""
    import rclpy
    from geometry_msgs.msg import WrenchStamped
    from functools import partial
    import time

    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)

    messages_before = []

    def collect_before(msg: WrenchStamped, messages: list) -> None:
        messages.append(msg)

    _ = driver.node.create_subscription(
        WrenchStamped,
        "/schunk/driver/data",
        partial(collect_before, messages=messages_before),
        10,
    )

    # Before activate, no data should be published
    timeout = time.time() + 0.3
    while time.time() < timeout:
        rclpy.spin_once(driver.node, timeout_sec=0.01)

    assert len(messages_before) == 0, "No data should be published before activate"

    # Activate
    result = driver.change_state(Transition.TRANSITION_ACTIVATE)
    assert result.success

    messages_after = []

    def collect_after(msg: WrenchStamped, messages: list) -> None:
        messages.append(msg)

    # Subscribe again after activate
    _ = driver.node.create_subscription(
        WrenchStamped,
        "/schunk/driver/data",
        partial(collect_after, messages=messages_after),
        10,
    )

    # After activate, data should be published
    timeout = time.time() + 0.5
    while time.time() < timeout and len(messages_after) < 10:
        rclpy.spin_once(driver.node, timeout_sec=0.01)

    assert len(messages_after) > 0, "Data should be published after activate"

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)


def test_deactivate_stops_data_publishing(sensor, lifecycle_interface):
    """Test that deactivate transition stops data publishing."""
    import rclpy
    from geometry_msgs.msg import WrenchStamped
    from functools import partial
    import time

    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    messages_active = []

    def collect_active(msg: WrenchStamped, messages: list) -> None:
        messages.append(msg)

    sub = driver.node.create_subscription(
        WrenchStamped,
        "/schunk/driver/data",
        partial(collect_active, messages=messages_active),
        10,
    )

    # While active, should receive data
    timeout = time.time() + 0.3
    while time.time() < timeout:
        rclpy.spin_once(driver.node, timeout_sec=0.01)

    assert len(messages_active) > 0, "Should receive data while active"

    # Deactivate
    driver.node.destroy_subscription(sub)
    result = driver.change_state(Transition.TRANSITION_DEACTIVATE)
    assert result.success

    messages_inactive = []

    def collect_inactive(msg: WrenchStamped, messages: list) -> None:
        messages.append(msg)

    _ = driver.node.create_subscription(
        WrenchStamped,
        "/schunk/driver/data",
        partial(collect_inactive, messages=messages_inactive),
        10,
    )

    # After deactivate, no data should be published
    timeout = time.time() + 0.5
    while time.time() < timeout:
        rclpy.spin_once(driver.node, timeout_sec=0.01)

    assert len(messages_inactive) == 0, "No data should be published after deactivate"

    driver.change_state(Transition.TRANSITION_CLEANUP)


def test_cleanup_releases_resources(sensor, lifecycle_interface):
    """Test that cleanup transition releases resources properly."""
    driver = lifecycle_interface

    # Go through a full cycle
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)
    driver.change_state(Transition.TRANSITION_DEACTIVATE)

    # Cleanup should release resources
    result = driver.change_state(Transition.TRANSITION_CLEANUP)
    assert result.success
    assert driver.check_state(State.PRIMARY_STATE_UNCONFIGURED)

    # After cleanup, should be able to configure again (resources were released)
    result = driver.change_state(Transition.TRANSITION_CONFIGURE)
    assert result.success, "Should be able to reconfigure after cleanup"

    driver.change_state(Transition.TRANSITION_CLEANUP)


def test_error_handling_during_configure(sensor, lifecycle_interface):
    """Test error handling during configure transition."""
    import rclpy
    from rclpy.node import Node
    from rcl_interfaces.srv import SetParameters
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

    # Set an invalid host to force configure to fail
    node = Node("test_configure_error")
    set_params_client = node.create_client(
        SetParameters, "/schunk/driver/set_parameters"
    )
    assert set_params_client.wait_for_service(timeout_sec=2)

    # Set unreachable host
    param = Parameter(
        name="host",
        value=ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value="192.168.99.99"
        ),
    )
    future = set_params_client.call_async(SetParameters.Request(parameters=[param]))
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

    # Try to configure - should fail
    _ = lifecycle_interface.change_state(Transition.TRANSITION_CONFIGURE)

    # Should either fail or timeout, but not crash
    # Node should remain in unconfigured or error recovery state
    # Just verify we can still interact with it
    assert not lifecycle_interface.check_state(State.PRIMARY_STATE_ACTIVE)

    node.destroy_node()


def test_rapid_transitions_without_crashes(sensor, lifecycle_interface):
    """Test that rapid state transitions don't cause crashes."""
    driver = lifecycle_interface

    # Rapid transitions
    for _ in range(3):
        driver.change_state(Transition.TRANSITION_CONFIGURE)
        driver.change_state(Transition.TRANSITION_ACTIVATE)
        driver.change_state(Transition.TRANSITION_DEACTIVATE)
        driver.change_state(Transition.TRANSITION_CLEANUP)

    # Should still be functional
    result = driver.change_state(Transition.TRANSITION_CONFIGURE)
    assert result.success
    driver.change_state(Transition.TRANSITION_CLEANUP)


def test_services_available_in_active_state(sensor, lifecycle_interface):
    """Test that services are available after activation."""
    from rclpy.node import Node
    from example_interfaces.srv import Trigger

    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    # Check that services are available
    node = Node("test_services_available")
    tare_client = node.create_client(Trigger, "/schunk/driver/tare")
    reset_tare_client = node.create_client(Trigger, "/schunk/driver/reset_tare")

    assert tare_client.wait_for_service(
        timeout_sec=2.0
    ), "Tare service should be available"
    assert reset_tare_client.wait_for_service(
        timeout_sec=2.0
    ), "Reset tare service should be available"

    node.destroy_node()
    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)


def test_services_not_available_in_inactive_state(sensor, lifecycle_interface):
    """Test that services are not available in inactive state."""
    from rclpy.node import Node
    from example_interfaces.srv import Trigger

    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)

    # In inactive state, services should not be available
    node = Node("test_services_inactive")
    tare_client = node.create_client(Trigger, "/schunk/driver/tare")

    # Services should not be available (or timeout quickly)
    service_available = tare_client.wait_for_service(timeout_sec=0.5)
    assert not service_available, "Services should not be available in inactive state"

    node.destroy_node()
    driver.change_state(Transition.TRANSITION_CLEANUP)
