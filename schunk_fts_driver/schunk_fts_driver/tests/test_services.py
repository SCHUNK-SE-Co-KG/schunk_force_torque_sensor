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
import rclpy
from rclpy.executors import MultiThreadedExecutor
from lifecycle_msgs.msg import Transition
from example_interfaces.srv import Trigger
from schunk_fts_interfaces.srv import SendCommand  # type: ignore [attr-defined]
import threading


@pytest.fixture(scope="function")
def service_client_node(ros2, lifecycle_interface):
    """Fixture to provide a node for service clients and ensure driver is active."""
    lifecycle_interface.change_state(Transition.TRANSITION_CONFIGURE)
    lifecycle_interface.change_state(Transition.TRANSITION_ACTIVATE)
    node = rclpy.create_node("service_client_node")
    yield node
    node.destroy_node()


def call_service(node, client, request):
    """Helper to call a service and wait for the result."""
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    return future.result()


def test_tare_service(service_client_node):
    """Test the successful execution of the tare service."""
    cli = service_client_node.create_client(Trigger, "/schunk/driver/tare")
    assert cli.wait_for_service(timeout_sec=2.0)
    req = Trigger.Request()
    result = call_service(service_client_node, cli, req)
    assert result is not None
    assert result.success is True
    assert result.message == ""


def test_reset_tare_service(service_client_node):
    """Test the successful execution of the reset_tare service."""
    cli = service_client_node.create_client(Trigger, "/schunk/driver/reset_tare")
    assert cli.wait_for_service(timeout_sec=2.0)
    req = Trigger.Request()
    result = call_service(service_client_node, cli, req)
    assert result is not None
    assert result.success is True
    assert result.message == ""


def test_send_command_service_success(service_client_node):
    """Test the send_command service with a valid command (tare)."""
    cli = service_client_node.create_client(SendCommand, "/schunk/driver/send_command")
    assert cli.wait_for_service(timeout_sec=2.0)
    req = SendCommand.Request()
    req.command_id = "12"
    result = call_service(service_client_node, cli, req)
    assert result is not None
    assert result.success is True


def test_send_command_service_fail(service_client_node):
    """Test the send_command service with an invalid command."""
    cli = service_client_node.create_client(SendCommand, "/schunk/driver/send_command")
    assert cli.wait_for_service(timeout_sec=2.0)
    req = SendCommand.Request()
    req.command_id = "0xFF"  # Invalid command
    result = call_service(service_client_node, cli, req)
    assert result is not None
    assert result.success is False
    assert "Unknown Command" in result.error_message


def test_concurrent_tare_service_calls(service_client_node):
    """Test multiple concurrent tare service calls."""
    # Use a thread-safe executor to manage all callbacks for the node.
    executor = MultiThreadedExecutor()
    executor.add_node(service_client_node)

    # Run the executor in a background thread. This thread will now handle all
    # service responses for the service_client_node.
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    cli = service_client_node.create_client(Trigger, "/schunk/driver/tare")
    assert cli.wait_for_service(timeout_sec=5.0)

    results = []
    errors = []
    # Using a lock is good practice when multiple threads modify a shared list.
    lock = threading.Lock()

    def call_tare():
        """This function will be run by each thread."""
        try:
            req = Trigger.Request()
            future = cli.call_async(req)

            executor.spin_until_future_complete(future, timeout_sec=5.0)

            result = future.result()
            with lock:
                if result:
                    results.append(result)
                else:
                    errors.append("Service call timed out and returned None.")
        except Exception as e:
            with lock:
                errors.append(e)

    # Launch 3 concurrent calls
    threads = []
    for _ in range(3):
        t = threading.Thread(target=call_tare)
        threads.append(t)
        t.start()

    # Wait for all threads to complete
    for t in threads:
        t.join(timeout=10.0)

    # Clean up the executor
    executor.shutdown()
    executor_thread.join()

    assert len(errors) == 0, f"Concurrent calls should not cause errors: {errors}"
    assert (
        len(results) == 3
    ), f"All concurrent calls should complete, but only {len(results)} did."

    # All should succeed
    for result in results:
        assert (
            result is not None
        ), "A service call result was None, likely due to a timeout."
        assert result.success is True, "Service call did not report success."


def test_service_response_time(service_client_node):
    """Test that services respond within reasonable time."""
    import time

    cli = service_client_node.create_client(Trigger, "/schunk/driver/tare")
    assert cli.wait_for_service(timeout_sec=2.0)

    # Measure response time
    start_time = time.time()
    req = Trigger.Request()
    result = call_service(service_client_node, cli, req)
    elapsed_time = time.time() - start_time

    assert result is not None
    assert result.success is True
    assert elapsed_time < 2.0, f"Service should respond quickly, took {elapsed_time}s"


def test_service_call_during_data_publishing(service_client_node, lifecycle_interface):
    """Test that service calls work while data is being published."""
    import rclpy
    from geometry_msgs.msg import WrenchStamped
    from functools import partial

    messages = []

    def collect(msg: WrenchStamped, messages: list) -> None:
        messages.append(msg)

    _ = lifecycle_interface.node.create_subscription(
        WrenchStamped,
        "/schunk/driver/data",
        partial(collect, messages=messages),
        10,
    )

    # Start collecting data
    for _ in range(10):
        rclpy.spin_once(lifecycle_interface.node, timeout_sec=0.001)

    initial_count = len(messages)
    assert initial_count > 0, "Should be receiving data"

    # Make a service call
    cli = service_client_node.create_client(Trigger, "/schunk/driver/tare")
    assert cli.wait_for_service(timeout_sec=2.0)
    req = Trigger.Request()
    result = call_service(service_client_node, cli, req)
    assert result.success

    # Data should continue after service call
    for _ in range(10):
        rclpy.spin_once(lifecycle_interface.node, timeout_sec=0.001)

    assert len(messages) > initial_count, "Data should continue after service call"


def test_multiple_service_types_concurrently(service_client_node):
    """Test calling different service types concurrently."""
    # 1. Use a thread-safe executor to manage all callbacks for the node.
    executor = MultiThreadedExecutor()
    executor.add_node(service_client_node)

    # 2. Run the executor in a background thread.
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # 3. Create clients and wait for services (safer to do this after executor starts).
    tare_cli = service_client_node.create_client(Trigger, "/schunk/driver/tare")
    reset_cli = service_client_node.create_client(Trigger, "/schunk/driver/reset_tare")
    cmd_cli = service_client_node.create_client(
        SendCommand, "/schunk/driver/send_command"
    )

    assert tare_cli.wait_for_service(timeout_sec=2.0)
    assert reset_cli.wait_for_service(timeout_sec=2.0)
    assert cmd_cli.wait_for_service(timeout_sec=2.0)

    results = {"tare": [], "reset": [], "command": []}
    errors = []
    lock = threading.Lock()  # Use a lock for thread-safe access to shared lists

    def call_tare():
        try:
            req = Trigger.Request()
            future = tare_cli.call_async(req)
            # Use the shared executor to safely wait for the future to complete
            executor.spin_until_future_complete(future, timeout_sec=5.0)
            with lock:
                results["tare"].append(future.result())
        except Exception as e:
            with lock:
                errors.append(("tare", e))

    def call_reset():
        try:
            req = Trigger.Request()
            future = reset_cli.call_async(req)
            executor.spin_until_future_complete(future, timeout_sec=5.0)
            with lock:
                results["reset"].append(future.result())
        except Exception as e:
            with lock:
                errors.append(("reset", e))

    def call_command():
        try:
            req = SendCommand.Request()
            req.command_id = "12"  # Tare command
            future = cmd_cli.call_async(req)
            executor.spin_until_future_complete(future, timeout_sec=5.0)
            with lock:
                results["command"].append(future.result())
        except Exception as e:
            with lock:
                errors.append(("command", e))

    # Launch concurrent calls of different types
    threads = [
        threading.Thread(target=call_tare),
        threading.Thread(target=call_reset),
        threading.Thread(target=call_command),
    ]

    for t in threads:
        t.start()

    for t in threads:
        t.join(timeout=10.0)

    # 4. Gracefully shut down the executor
    executor.shutdown()
    executor_thread.join()

    assert len(errors) == 0, f"Concurrent service calls should not error: {errors}"
    assert (
        len(results["tare"]) == 1 and results["tare"][0] is not None
    ), "Tare service call failed or timed out."
    assert (
        len(results["reset"]) == 1 and results["reset"][0] is not None
    ), "Reset service call failed or timed out."
    assert (
        len(results["command"]) == 1 and results["command"][0] is not None
    ), "Command service call failed or timed out."

    # All should succeed
    assert results["tare"][0].success
    assert results["reset"][0].success
    assert results["command"][0].success


def test_send_command_with_various_formats(service_client_node):
    """Test send_command service with various command ID formats."""
    cli = service_client_node.create_client(SendCommand, "/schunk/driver/send_command")
    assert cli.wait_for_service(timeout_sec=2.0)

    # Test valid command with different formats
    test_cases = [
        ("12", True),  # Plain hex
        ("0x12", True),  # With 0x prefix
        ("0X12", True),  # With 0X prefix
    ]

    for command_id, expected_success in test_cases:
        req = SendCommand.Request()
        req.command_id = command_id
        result = call_service(service_client_node, cli, req)
        assert result is not None, f"Should get response for {command_id}"
        if expected_success:
            # Tare command (0x12) should succeed
            assert result.success, f"Command {command_id} should succeed"


def test_service_error_messages_are_descriptive(service_client_node):
    """Test that service error messages are descriptive and helpful."""
    cli = service_client_node.create_client(SendCommand, "/schunk/driver/send_command")
    assert cli.wait_for_service(timeout_sec=2.0)

    # Test with various invalid commands
    invalid_commands = ["0xFF", "0xAB", "99"]

    for cmd in invalid_commands:
        req = SendCommand.Request()
        req.command_id = cmd
        result = call_service(service_client_node, cli, req)

        assert result is not None
        assert result.success is False
        assert len(result.error_message) > 0, "Error message should not be empty"
        # Error message should be meaningful
        assert any(
            word in result.error_message
            for word in ["Unknown", "Command", "Error", "Invalid"]
        ), f"Error message should be descriptive: {result.error_message}"


def test_repeated_service_calls(service_client_node):
    """Test that services handle repeated calls correctly."""
    cli = service_client_node.create_client(Trigger, "/schunk/driver/tare")
    assert cli.wait_for_service(timeout_sec=2.0)

    # Make multiple sequential calls
    for i in range(5):
        req = Trigger.Request()
        result = call_service(service_client_node, cli, req)
        assert result is not None, f"Call {i + 1} should return result"
        assert result.success is True, f"Call {i + 1} should succeed"
        assert result.message == "", f"Call {i + 1} should have no error message"


def test_service_timeout_behavior(service_client_node):
    """Test service call timeout behavior."""
    import time

    cli = service_client_node.create_client(Trigger, "/schunk/driver/tare")
    assert cli.wait_for_service(timeout_sec=2.0)

    # Make a call with timeout
    req = Trigger.Request()
    start = time.time()

    # Use the helper which has a 5 second timeout
    result = call_service(service_client_node, cli, req)
    elapsed = time.time() - start

    # Should complete well within timeout
    assert elapsed < 3.0, f"Service call took too long: {elapsed}s"
    assert result is not None, "Should get result within timeout"


def test_tare_and_reset_tare_sequence(service_client_node):
    """Test sequence of tare followed by reset_tare."""
    tare_cli = service_client_node.create_client(Trigger, "/schunk/driver/tare")
    reset_cli = service_client_node.create_client(Trigger, "/schunk/driver/reset_tare")

    assert tare_cli.wait_for_service(timeout_sec=2.0)
    assert reset_cli.wait_for_service(timeout_sec=2.0)

    # First tare
    tare_req = Trigger.Request()
    tare_result = call_service(service_client_node, tare_cli, tare_req)
    assert tare_result.success, "Tare should succeed"

    # Then reset tare
    reset_req = Trigger.Request()
    reset_result = call_service(service_client_node, reset_cli, reset_req)
    assert reset_result.success, "Reset tare should succeed"

    # Tare again
    tare_result2 = call_service(service_client_node, tare_cli, tare_req)
    assert tare_result2.success, "Second tare should succeed"


def test_service_availability_after_error(service_client_node):
    """Test that services remain available after an error."""
    cli = service_client_node.create_client(SendCommand, "/schunk/driver/send_command")
    assert cli.wait_for_service(timeout_sec=2.0)

    # First, cause an error
    error_req = SendCommand.Request()
    error_req.command_id = "0xFF"
    error_result = call_service(service_client_node, cli, error_req)
    assert error_result.success is False

    # Service should still be available
    assert cli.wait_for_service(timeout_sec=1.0), "Service should still be available"

    # Make a valid call
    valid_req = SendCommand.Request()
    valid_req.command_id = "12"
    valid_result = call_service(service_client_node, cli, valid_req)
    assert valid_result.success is True, "Valid call should succeed after error"
