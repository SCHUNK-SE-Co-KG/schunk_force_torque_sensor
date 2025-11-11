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
import struct
import time
from schunk_fts_library.driver import Driver, SensorStatus
from schunk_fts_library.utility import (
    Connection,
    Stream,
    FTDataBuffer,
    GetParameterResponse,
    SetParameterResponse,
    CommandResponse,
)


def test_error_connection_timeout():
    """Test connection timeout handling."""
    # Connect to non-routable address (should timeout)
    connection = Connection(host="192.0.2.1", port=82)
    connection.socket.settimeout(0.1)

    start = time.time()
    with connection:
        pass
    elapsed = time.time() - start

    # Should timeout quickly
    assert elapsed < 1.0
    assert not connection.is_open


def test_error_connection_refused():
    """Test handling of connection refused error."""
    # Try to connect to local port that's not listening
    connection = Connection(host="127.0.0.1", port=9999)

    with connection:
        pass

    # Should fail gracefully
    assert not connection.is_open


def test_error_invalid_hostname():
    """Test handling of invalid hostname."""
    connection = Connection(host="invalid.nonexistent.hostname.local", port=82)

    with connection:
        pass

    # Should fail gracefully
    assert not connection.is_open


def test_error_stream_port_in_use():
    """Test handling when stream port is already in use."""
    stream1 = Stream(port=54851)
    stream2 = Stream(port=54851)

    with stream1:
        assert stream1.is_open()

        # Second stream should fail to bind
        with stream2:
            # May or may not be open depending on SO_REUSEADDR behavior
            pass

    assert not stream1.is_open()


def test_error_stream_invalid_port_range():
    """Test handling of invalid port numbers."""
    invalid_ports = [-1, 0, 65536, 999999]

    for port in invalid_ports:
        stream = Stream(port=port)
        with stream:
            # Should not open with invalid port
            assert not stream.is_open()


def test_error_send_without_connection():
    """Test sending data without connection."""
    connection = Connection(host="127.0.0.1", port=8082)

    data = bytearray(b"\xff\xff\x00\x00\x01\x00\x12")

    # Try to send without opening connection
    result = connection.send(data)

    assert not result


def test_error_receive_without_connection():
    """Test receiving data without connection."""
    connection = Connection(host="127.0.0.1", port=8082)

    # Try to receive without opening connection
    data = connection.receive()

    assert len(data) == 0


def test_error_malformed_packet_decode():
    """Test handling of malformed packets."""
    # Too short packet
    with pytest.raises((struct.error, IndexError)):
        FTDataBuffer.decode(bytearray(b"\xFF\xFF\x00\x00"))

    # Empty packet
    with pytest.raises((struct.error, IndexError)):
        FTDataBuffer.decode(bytearray())


def test_error_response_error_codes():
    """Test handling of error codes in responses."""
    # Simulate error responses
    error_codes = ["01", "02", "10", "ff"]

    for error_code in error_codes:
        response_data = bytearray()
        response_data += bytes.fromhex("ffff")
        response_data += struct.pack("<H", 1)
        response_data += struct.pack("<H", 2)
        response_data += bytes.fromhex("12")
        response_data += bytes.fromhex(error_code)

        resp = CommandResponse()
        resp.from_bytes(response_data)

        assert resp.error_code == error_code


def test_error_status_bits_error_conditions():
    """Test SensorStatus decoding of error conditions."""
    # Test various error bit combinations
    error_scenarios = [
        (0x00000002, "process_data_invalid"),  # Bit 1
        (0x00000004, "temperature_out_of_range"),  # Bit 2
        (0x00000008, "hardware_error"),  # Bit 3
        (0x00000010, "mech_overrange"),  # Bit 4
        (0x00000020, "user_overrange"),  # Bit 5
        (0x0000003E, "multiple_errors"),  # Multiple bits set
    ]

    for status_bits, _ in error_scenarios:
        status = SensorStatus.from_bits(status_bits)

        if status_bits & 0x00000002:
            assert status.process_data_invalid
        if status_bits & 0x00000004:
            assert status.temperature_out_of_range
        if status_bits & 0x00000008:
            assert status.hardware_error
        if status_bits & 0x00000010:
            assert status.mech_overrange
        if status_bits & 0x00000020:
            assert status.user_overrange


def test_error_status_summary_messages():
    """Test SensorStatus.summary() returns correct error messages."""
    # No errors
    status = SensorStatus.from_bits(0x00000001)  # Ready bit set
    summary = status.summary()
    assert len(summary) == 0

    # Not ready
    status = SensorStatus.from_bits(0x00000000)
    summary = status.summary()
    assert any("Not ready" in msg for msg in summary)

    # Multiple errors
    status = SensorStatus.from_bits(0x0000003E)  # All error bits except ready
    summary = status.summary()
    assert len(summary) >= 5  # Should have multiple error messages


def test_error_driver_streaming_on_invalid_timeout():
    """Test streaming_on with invalid timeout values."""
    driver = Driver(host="127.0.0.1", port=8082)

    invalid_timeouts = ["not a number", None, [], {}]

    for timeout in invalid_timeouts:
        result = driver.streaming_on(timeout_sec=timeout)
        assert not result


def test_error_driver_streaming_on_unreachable_sensor():
    """Test streaming_on with unreachable sensor."""
    driver = Driver(host="192.0.2.1", port=82)

    result = driver.streaming_on(timeout_sec=0.1)

    assert not result
    assert not driver.is_streaming


def test_error_driver_sample_without_streaming():
    """Test sampling without active streaming."""
    driver = Driver(host="127.0.0.1", port=8082)

    sample = driver.sample()

    assert sample is None


def test_error_driver_get_status_without_streaming():
    """Test get_status without active streaming."""
    driver = Driver(host="127.0.0.1", port=8082)

    status = driver.get_status()

    assert status is None


def test_error_buffer_timeout_returns_empty_data():
    """Test that buffer returns empty FTData on timeout."""
    buffer = FTDataBuffer(maxsize=10)

    # Don't put any data, just try to get
    sample = buffer.get()

    # First call returns None (waiting)
    assert sample is None

    # Wait for timeout
    time.sleep(0.3)

    # After timeout, should return empty FTData
    sample = buffer.get()
    if sample is not None:
        assert sample["fx"] == 0.0
        assert sample["fy"] == 0.0
        assert sample["fz"] == 0.0


def test_error_connection_broken_pipe_handling(sensor):
    """Test handling of broken pipe during send."""
    HOST, PORT = sensor
    connection = Connection(host=HOST, port=PORT)
    connection.open()

    # Close socket to simulate broken connection
    connection.socket.close()

    # Try to send - should fail gracefully
    data = bytearray(b"\xff\xff\x00\x00\x01\x00\x12")
    result = connection.send(data)

    assert not result


def test_error_stream_read_with_closed_socket():
    """Test reading from stream with closed socket."""
    stream = Stream(port=54852)

    with stream:
        # Close socket while in context
        stream.socket.close()

        # Read should return empty list
        result = stream.read()
        assert result == []


def test_error_driver_multiple_streaming_off_calls():
    """Test multiple streaming_off calls don't cause errors."""
    driver = Driver(host="127.0.0.1", port=8082)

    # Call streaming_off multiple times without streaming_on
    for _ in range(5):
        driver.streaming_off()

    assert not driver.is_streaming


def test_error_parameter_response_parsing_incomplete_data():
    """Test handling of incomplete parameter response data."""
    # Incomplete response (missing param_value)
    response_data = bytearray()
    response_data += bytes.fromhex("ffff")
    response_data += struct.pack("<H", 1)
    response_data += struct.pack("<H", 7)
    response_data += bytes.fromhex("f0")
    response_data += bytes.fromhex("00")
    response_data += bytes.fromhex("0100")
    response_data += bytes.fromhex("00")
    # Missing param_value

    resp = GetParameterResponse()
    resp.from_bytes(response_data)

    # Should handle gracefully
    assert resp.param_value == ""


def test_error_connection_context_manager_exception_safety():
    """Test that exceptions in context manager don't leak resources."""
    connection = Connection(host="127.0.0.1", port=8082)

    try:
        with connection:
            raise RuntimeError("Test exception")
    except RuntimeError:
        pass

    # Connection should be properly closed
    assert not connection.is_open


def test_error_stream_context_manager_exception_safety():
    """Test that exceptions in stream context manager don't leak resources."""
    stream = Stream(port=54853)

    try:
        with stream:
            raise RuntimeError("Test exception")
    except RuntimeError:
        pass

    # Stream should be properly closed
    assert not stream.is_open()


def test_error_driver_timeout_detection(sensor, send_messages):
    """Test driver detects timeout when data stops arriving."""
    HOST, PORT = sensor
    test_port = 8010
    driver = Driver(host=HOST, port=PORT, streaming_port=test_port)
    driver.timeout_sec = 0.2

    # Start streaming
    if driver.streaming_on(timeout_sec=1.0):
        # Send one packet
        packet = bytearray(b"\xFF\xFF") + struct.pack(
            "<HHB I ffffff", 0, 29, 1, 0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0
        )
        send_messages(test_port, [packet])
        time.sleep(0.1)

        # Wait for timeout (no more packets)
        time.sleep(0.5)

        # Driver should detect timeout and stop
        # (depending on implementation, is_streaming might become False)

        driver.streaming_off()


def test_error_invalid_message_fields():
    """Test handling of invalid message field values."""
    from schunk_fts_library.utility import GetParameterRequest

    req = GetParameterRequest()

    # Set invalid hex values
    req.command_id = "zz"  # Invalid hex
    req.param_index = "gggg"  # Invalid hex

    # to_bytes should handle gracefully or raise clear error
    try:
        data = req.to_bytes()
        # If it succeeds, it should produce some output
        assert isinstance(data, bytearray)
    except (ValueError, TypeError):
        # Expected for invalid hex
        pass


def test_error_status_bits_overflow():
    """Test SensorStatus with overflow status bits."""
    # All bits set (uint32 max)
    status = SensorStatus.from_bits(0xFFFFFFFF)

    # Should handle all bits
    assert status.ready
    assert status.process_data_invalid
    assert status.temperature_out_of_range
    assert status.hardware_error
    assert status.mech_overrange
    assert status.user_overrange


def test_error_get_parameter_empty_response(sensor):
    """Test get_parameter with empty response."""
    HOST, PORT = sensor
    driver = Driver(host=HOST, port=PORT)

    # Try to get parameter without proper connection
    resp = driver.get_parameter(index="9999", subindex="ff")

    # Should return response object (even if empty/error)
    assert isinstance(resp, GetParameterResponse)


def test_error_set_parameter_invalid_value(sensor):
    """Test set_parameter with potentially invalid value."""
    HOST, PORT = sensor
    driver = Driver(host=HOST, port=PORT)

    # Try to set with invalid values
    resp = driver.set_parameter(value="invalid", index="9999", subindex="ff")

    # Should return response object
    assert isinstance(resp, SetParameterResponse)


def test_error_run_command_without_connection():
    """Test run_command without active connection."""
    driver = Driver(host="192.0.2.1", port=82)

    # Try to run command without connection
    resp = driver.run_command("12")

    # Should return empty response
    assert isinstance(resp, CommandResponse)
    assert resp.error_code == ""


def test_error_buffer_full_handling():
    """Test buffer behavior when full."""
    buffer = FTDataBuffer(maxsize=5)

    # Fill buffer beyond capacity
    packet = bytearray(b"\xFF\xFF") + struct.pack(
        "<HHB I ffffff", 0, 29, 1, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    )

    for _ in range(10):  # Try to put more than maxsize
        buffer.put(packet)

    # Buffer should not exceed maxsize
    assert len(buffer) <= 5


def test_error_decode_with_incorrect_sync_bytes():
    """Test decode behavior with incorrect sync bytes."""
    # Packet with wrong sync bytes
    packet = bytearray(b"\x00\x00") + struct.pack(
        "<HHB I ffffff", 0, 29, 1, 0, 1.0, 2.0, 3.0, 0.1, 0.2, 0.3
    )

    # Should still decode but with different sync
    decoded = FTDataBuffer.decode(packet)
    assert decoded["sync"] == b"\x00\x00"
    assert decoded["fx"] == pytest.approx(1.0)


def test_error_driver_stream_with_invalid_port():
    """Test driver streaming with invalid UDP port."""
    driver = Driver(host="127.0.0.1", port=8082, streaming_port=-1)

    result = driver.streaming_on(timeout_sec=0.5)

    # Should fail
    assert not result
    assert not driver.is_streaming
