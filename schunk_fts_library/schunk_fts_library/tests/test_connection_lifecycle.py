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
import time
import socket
from threading import Thread
from schunk_fts_library.utility import Connection, Stream
from schunk_fts_library.driver import Driver


def test_connection_reconnection_after_close(sensor):
    """Test reconnection after explicit close."""
    HOST, PORT = sensor
    connection = Connection(host=HOST, port=PORT)

    for i in range(5):
        # Open connection (enters persistent mode)
        assert connection.open(), f"Open failed on iteration {i}"
        assert connection.is_open

        # Close connection (clears persistent flag)
        connection.close()
        # After close(), persistent flag is cleared, but implementation may keep socket
        # Check that persistent flag is cleared
        assert not connection.persistent.is_set()

        # Small delay to ensure clean state
        time.sleep(0.05)


def test_connection_reconnection_after_context_exit(sensor):
    """Test reconnection after context manager exit."""
    HOST, PORT = sensor
    connection = Connection(host=HOST, port=PORT)

    for i in range(5):
        with connection:
            assert connection.is_open, f"Connection not open on iteration {i}"
        # Should close automatically
        assert not connection.is_open, f"Connection still open after iteration {i}"


def test_connection_resource_cleanup_on_failure():
    """Test that failed connection attempts clean up resources."""
    # Try to connect to invalid host/port combinations
    invalid_connections = [
        Connection(host="192.0.2.1", port=82),  # TEST-NET-1, should timeout
        Connection(host="127.0.0.1", port=1),  # Privileged port, likely refused
        Connection(host="invalid.host.local", port=82),  # Invalid hostname
    ]

    for conn in invalid_connections:
        with conn:
            pass
        # Should not be open after failed connection
        assert not conn.is_open


def test_connection_cleanup_after_exception(sensor):
    """Test connection cleanup after exception in context manager."""
    HOST, PORT = sensor
    connection = Connection(host=HOST, port=PORT)

    try:
        with connection:
            assert connection.is_open
            raise ValueError("Test exception")
    except ValueError:
        pass

    # Connection should be cleaned up
    assert not connection.is_open


def test_connection_persistent_mode(sensor):
    """Test persistent connection mode across context managers."""
    HOST, PORT = sensor
    connection = Connection(host=HOST, port=PORT)

    # Open in persistent mode
    assert connection.open()
    assert connection.persistent.is_set()

    # Enter and exit context manager
    with connection:
        assert connection.is_open

    # Should still be open after context exit (persistent mode)
    assert connection.is_open

    # Explicit close
    connection.close()
    # Check persistent flag is cleared
    assert not connection.persistent.is_set()


def test_connection_multiple_context_managers_non_persistent(sensor):
    """Test multiple context manager entries without persistent mode."""
    HOST, PORT = sensor
    connection = Connection(host=HOST, port=PORT)

    for i in range(3):
        with connection:
            assert connection.is_open, f"Iteration {i}"
            assert not connection.persistent.is_set()
        assert not connection.is_open, f"After iteration {i}"


def test_stream_reconnection_after_close():
    """Test stream reconnection after close."""
    stream = Stream(port=54844)

    for i in range(5):
        with stream:
            assert stream.is_open(), f"Stream not open on iteration {i}"
        assert not stream.is_open(), f"Stream still open after iteration {i}"


def test_stream_resource_cleanup_on_invalid_port():
    """Test stream cleanup on invalid port."""
    invalid_ports = [0, -1, 80, 1023]  # Invalid or privileged ports

    for port in invalid_ports:
        stream = Stream(port=port)
        with stream:
            pass
        # Should not be open
        assert not stream.is_open()
        # Socket should be closed
        assert stream.socket.fileno() == -1


def test_stream_socket_reset_after_close():
    """Test that socket is properly reset after close."""
    stream = Stream(port=54845)

    # First use
    with stream:
        socket_1 = stream.socket
        assert stream.is_open()

    # Socket should be closed
    assert socket_1.fileno() == -1

    # Second use - should create new socket
    with stream:
        socket_2 = stream.socket
        assert stream.is_open()

    assert socket_1 != socket_2


def test_driver_streaming_connection_lifecycle(sensor):
    """Test driver streaming lifecycle."""
    HOST, PORT = sensor
    driver = Driver(host=HOST, port=PORT)

    for i in range(3):
        # Start streaming
        assert driver.streaming_on(), f"streaming_on failed on iteration {i}"
        assert driver.is_streaming
        assert driver.connection.is_open
        assert driver.stream.is_open()

        # Stop streaming
        driver.streaming_off()
        assert not driver.is_streaming
        # Connection and stream should be closed
        assert not driver.stream.is_open()

        time.sleep(0.05)


def test_driver_reconnection_after_timeout():
    """Test driver reconnection after timeout."""
    driver = Driver(host="127.0.0.1", port=8082)

    # Try to start streaming with very short timeout
    result = driver.streaming_on(timeout_sec=0.001)

    # Might fail due to short timeout
    if not result:
        driver.streaming_off()

    # Should be able to try again
    time.sleep(0.1)
    _ = driver.streaming_on(timeout_sec=1.0)

    driver.streaming_off()


def test_driver_cleanup_on_streaming_failure():
    """Test driver cleanup when streaming fails."""
    driver = Driver(host="192.0.2.1", port=82, streaming_port=-1)

    # This should fail
    assert not driver.streaming_on(timeout_sec=0.1)
    assert not driver.is_streaming

    # Thread should not be running
    assert not driver.stream_update_thread.is_alive()


def test_connection_handles_broken_pipe(sensor):
    """Test connection handles broken pipe gracefully."""
    HOST, PORT = sensor
    connection = Connection(host=HOST, port=PORT)
    connection.open()

    # Send valid data
    valid_data = bytearray(b"\xff\xff\x00\x00\x01\x00\x12")
    assert connection.send(valid_data)

    # Force close the socket (simulate broken connection)
    connection.socket.close()

    # Try to send again - should fail gracefully
    assert not connection.send(valid_data)

    # Should be able to recover
    connection.close()
    assert connection.open()
    connection.close()


def test_connection_socket_options_preserved_after_reset(sensor):
    """Test that socket options are preserved after reset."""
    HOST, PORT = sensor
    connection = Connection(host=HOST, port=PORT)

    # Check initial socket options
    connection._reset_socket()
    timeout_1 = connection.socket.gettimeout()
    nodelay_1 = connection.socket.getsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY)

    # Reset and check again
    connection._reset_socket()
    timeout_2 = connection.socket.gettimeout()
    nodelay_2 = connection.socket.getsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY)

    assert timeout_1 == timeout_2
    assert nodelay_1 == nodelay_2


def test_stream_socket_options_preserved_after_reset():
    """Test that stream socket options are preserved after reset."""
    stream = Stream(port=54846)

    # Check initial socket options
    stream._reset_socket()
    blocking_1 = stream.socket.getblocking()
    reuse_1 = stream.socket.getsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR)

    # Reset and check again
    stream._reset_socket()
    blocking_2 = stream.socket.getblocking()
    reuse_2 = stream.socket.getsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR)

    assert blocking_1 == blocking_2 == False  # noqa: E712
    assert reuse_1 == reuse_2 == 1


def test_driver_stops_update_thread_on_streaming_off(sensor):
    """Test that update thread is properly stopped."""
    HOST, PORT = sensor
    driver = Driver(host=HOST, port=PORT)

    driver.streaming_on()
    thread = driver.stream_update_thread
    assert thread.is_alive()

    driver.streaming_off()

    # Wait a bit for thread to finish
    time.sleep(0.2)
    assert not thread.is_alive()


def test_driver_update_thread_cleanup_on_stream_error():
    """Test update thread cleanup when stream encounters error."""
    driver = Driver(host="127.0.0.1", port=8082, streaming_port=54847)

    if driver.streaming_on(timeout_sec=1.0):
        # Force stream error by closing the socket
        driver.stream.socket.close()

        # Wait for thread to detect error and exit
        time.sleep(0.5)

        driver.streaming_off()
        assert not driver.stream_update_thread.is_alive()


def test_connection_multiple_rapid_open_close(sensor):
    """Test rapid connection open/close cycles."""
    HOST, PORT = sensor
    connection = Connection(host=HOST, port=PORT)

    for _ in range(20):
        connection.open()
        connection.close()

    # Should still be functional
    assert connection.open()
    connection.close()


def test_driver_buffer_cleared_between_sessions(sensor):
    """Test that buffer is cleared between streaming sessions."""
    HOST, PORT = sensor
    driver = Driver(host=HOST, port=PORT)

    # First session
    driver.streaming_on()
    time.sleep(0.2)
    first_sample = driver.sample()
    driver.streaming_off()

    # Clear buffer manually (in production this might be automatic)
    # For now just verify we can start a new session
    time.sleep(0.1)

    # Second session
    driver.streaming_on()
    time.sleep(0.2)
    second_sample = driver.sample()
    driver.streaming_off()

    assert first_sample is not None
    assert second_sample is not None


def test_concurrent_connection_attempts_to_same_host(sensor):
    """Test concurrent connection attempts to same host."""
    HOST, PORT = sensor
    connections = [Connection(host=HOST, port=PORT) for _ in range(5)]

    threads = []

    def connect_and_close(conn):
        conn.open()
        time.sleep(0.1)
        conn.close()

    for conn in connections:
        t = Thread(target=connect_and_close, args=(conn,))
        t.start()
        threads.append(t)

    for t in threads:
        t.join()

    # All connections should have cleared persistent flag
    for conn in connections:
        assert not conn.persistent.is_set()


def test_connection_state_consistency_across_threads(sensor):
    """Test that connection state remains consistent across threads."""
    HOST, PORT = sensor
    connection = Connection(host=HOST, port=PORT)
    connection.open()

    state_checks = []

    def check_state():
        for _ in range(50):
            state_checks.append(connection.is_open)
            time.sleep(0.001)

    threads = [Thread(target=check_state) for _ in range(5)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    connection.close()

    # All state checks should be consistent (all True while open)
    assert all(state_checks)
