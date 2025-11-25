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
from schunk_fts_library.utility import (
    FTDataBuffer,
    GetParameterRequest,
    GetParameterResponse,
    SetParameterRequest,
    CommandRequest,
    CommandResponse,
)


def test_protocol_packet_structure_is_correct():
    """Test binary packet structure matches protocol specification."""
    # Test FT data packet structure: 42 bytes total
    data = {
        "sync": b"\xFF\xFF",
        "counter": 12345,
        "length": 29,
        "id": 1,
        "status_bits": 0x00000001,
        "fx": 1.5,
        "fy": -2.3,
        "fz": 3.7,
        "tx": 0.15,
        "ty": -0.23,
        "tz": 0.37,
    }

    packet = bytearray(data["sync"]) + struct.pack(
        "<HHB I ffffff",
        data["counter"],
        data["length"],
        data["id"],
        data["status_bits"],
        data["fx"],
        data["fy"],
        data["fz"],
        data["tx"],
        data["ty"],
        data["tz"],
    )

    # Verify packet length
    assert len(packet) == 35, "FT data packet must be 35 bytes"

    # Verify byte order (little-endian)
    decoded = FTDataBuffer.decode(packet)
    assert decoded["counter"] == data["counter"]
    assert decoded["status_bits"] == data["status_bits"]
    assert pytest.approx(decoded["fx"]) == data["fx"]
    assert pytest.approx(decoded["fy"]) == data["fy"]
    assert pytest.approx(decoded["fz"]) == data["fz"]


def test_protocol_handles_endianness_correctly():
    """Test little-endian byte order handling."""
    # Test counter field (uint16)
    packet = bytearray(b"\xFF\xFF") + struct.pack("<H", 0x1234)
    packet += bytearray(31)  # pad to 35 bytes (2 + 2 + 31 = 35)

    decoded = FTDataBuffer.decode(packet)
    assert decoded["counter"] == 0x1234

    # Verify it's not big-endian
    assert decoded["counter"] != 0x3412


def test_protocol_handles_float_precision():
    """Test floating-point value precision in packets."""
    test_values = [
        (1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        (0.001, -0.001, 0.999, 0.0001, -0.0001, 0.9999),
        (100.5, -200.3, 300.7, 10.15, -20.23, 30.37),
    ]

    for fx, fy, fz, tx, ty, tz in test_values:
        packet = bytearray(b"\xFF\xFF") + struct.pack(
            "<HHB I ffffff", 0, 29, 1, 0, fx, fy, fz, tx, ty, tz
        )

        decoded = FTDataBuffer.decode(packet)
        # Use larger tolerance for float32 precision
        assert pytest.approx(decoded["fx"], abs=1e-4) == fx
        assert pytest.approx(decoded["fy"], abs=1e-4) == fy
        assert pytest.approx(decoded["fz"], abs=1e-4) == fz
        assert pytest.approx(decoded["tx"], abs=1e-4) == tx
        assert pytest.approx(decoded["ty"], abs=1e-4) == ty
        assert pytest.approx(decoded["tz"], abs=1e-4) == tz


def test_protocol_get_parameter_request_format():
    """Test GetParameterRequest message formatting."""
    req = GetParameterRequest()
    req.command_id = "f0"
    req.param_index = "0001"
    req.param_subindex = "00"

    data = req.to_bytes()

    # Verify structure
    assert data[0:2] == bytes.fromhex("ffff")  # sync
    assert data[6:7].hex() == "f0"  # command_id
    # param_index should be little-endian
    assert data[7:9][::-1].hex() == "0001"


def test_protocol_get_parameter_response_parsing():
    """Test GetParameterResponse message parsing."""
    # Simulate a response packet
    response_data = bytearray()
    response_data += bytes.fromhex("ffff")  # sync
    response_data += struct.pack("<H", 1)  # counter
    response_data += struct.pack("<H", 7)  # payload length
    response_data += bytes.fromhex("f0")  # command_id
    response_data += bytes.fromhex("00")  # error_code (success)
    response_data += bytes.fromhex("0100")  # param_index (little-endian)
    response_data += bytes.fromhex("00")  # param_subindex
    response_data += bytes.fromhex("1234")  # param_value

    resp = GetParameterResponse()
    resp.from_bytes(response_data)

    assert resp.command_id == "f0"
    assert resp.error_code == "00"
    assert resp.param_index == "0001"
    assert resp.param_subindex == "00"
    assert resp.param_value == "1234"


def test_protocol_set_parameter_request_format():
    """Test SetParameterRequest message formatting."""
    req = SetParameterRequest()
    req.command_id = "f1"
    req.param_index = "0002"
    req.param_subindex = "01"
    req.param_value = "5678"

    data = req.to_bytes()

    # Verify structure
    assert data[0:2] == bytes.fromhex("ffff")  # sync
    assert data[6:7].hex() == "f1"  # command_id
    # param_index should be little-endian
    assert data[7:9][::-1].hex() == "0002"
    assert data[9:10].hex() == "01"  # param_subindex


def test_protocol_command_request_format():
    """Test CommandRequest message formatting."""
    commands = ["12", "13", "40", "41"]

    for cmd in commands:
        req = CommandRequest()
        req.command_id = cmd

        data = req.to_bytes()

        # Verify structure
        assert data[0:2] == bytes.fromhex("ffff")  # sync
        assert struct.unpack("<H", data[2:4])[0] == 0  # counter
        assert struct.unpack("<H", data[4:6])[0] == 1  # payload length
        assert data[6:7].hex() == cmd  # command_id


def test_protocol_command_response_parsing():
    """Test CommandResponse message parsing."""
    # Simulate a response packet
    response_data = bytearray()
    response_data += bytes.fromhex("ffff")  # sync
    response_data += struct.pack("<H", 5)  # counter
    response_data += struct.pack("<H", 2)  # payload length
    response_data += bytes.fromhex("12")  # command_id (tare)
    response_data += bytes.fromhex("00")  # error_code (success)

    resp = CommandResponse()
    resp.from_bytes(response_data)

    assert resp.command_id == "12"
    assert resp.error_code == "00"
    assert resp.counter == 5


def test_protocol_handles_sync_bytes_correctly():
    """Test sync byte detection in packets."""
    # Valid sync bytes
    packet = bytearray(b"\xFF\xFF") + struct.pack(
        "<HHB I ffffff", 0, 29, 1, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    )
    decoded = FTDataBuffer.decode(packet)
    assert decoded["sync"] == b"\xFF\xFF"

    # Invalid sync bytes should still decode (but sync field differs)
    packet_bad = bytearray(b"\x00\x00") + struct.pack(
        "<HHB I ffffff", 0, 29, 1, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    )
    decoded_bad = FTDataBuffer.decode(packet_bad)
    assert decoded_bad["sync"] == b"\x00\x00"


def test_protocol_handles_counter_rollover():
    """Test counter field rollover (uint16 max = 65535)."""
    counters = [0, 1, 100, 65534, 65535]

    for counter in counters:
        packet = bytearray(b"\xFF\xFF") + struct.pack(
            "<HHB I ffffff", counter, 29, 1, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        )
        decoded = FTDataBuffer.decode(packet)
        assert decoded["counter"] == counter


def test_protocol_status_bits_decoding():
    """Test status bits field decoding."""
    status_values = [
        0x00000000,  # All clear
        0x00000001,  # Bit 0 set (ready)
        0x00000002,  # Bit 1 set (process_data_invalid)
        0x0000003F,  # All 6 bits set
        0xFFFFFFFF,  # All bits set
    ]

    for status in status_values:
        packet = bytearray(b"\xFF\xFF") + struct.pack(
            "<HHB I ffffff", 0, 29, 1, status, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        )
        decoded = FTDataBuffer.decode(packet)
        assert decoded["status_bits"] == status


def test_protocol_message_equality():
    """Test message equality comparison."""
    req1 = CommandRequest()
    req1.command_id = "12"

    req2 = CommandRequest()
    req2.command_id = "12"

    req3 = CommandRequest()
    req3.command_id = "13"

    assert req1 == req2
    assert req1 != req3
    assert req1 != "not a message"


def test_protocol_message_string_representation():
    """Test message string representation."""
    req = CommandRequest()
    req.command_id = "12"

    msg_str = str(req)
    assert "command_id" in msg_str
    assert "12" in msg_str
    assert "sync" in msg_str


def test_protocol_handles_malformed_packets():
    """Test handling of malformed/truncated packets."""
    # Too short packet
    short_packet = bytearray(b"\xFF\xFF\x00\x00")
    with pytest.raises((struct.error, IndexError)):
        FTDataBuffer.decode(short_packet)

    # Empty packet
    with pytest.raises((struct.error, IndexError)):
        FTDataBuffer.decode(bytearray())


def test_protocol_payload_length_field():
    """Test payload length field in messages."""
    req = GetParameterRequest()
    req.command_id = "f0"
    req.param_index = "0001"
    req.param_subindex = "00"

    data = req.to_bytes()
    payload_len = struct.unpack("<H", data[4:6])[0]

    # Payload should be command_id (1) + param_index (2) + param_subindex (1) = 4 bytes
    assert payload_len == 4


def test_protocol_multiple_packets_in_sequence():
    """Test processing multiple packets in sequence."""
    packets = []
    for i in range(10):
        packet = bytearray(b"\xFF\xFF") + struct.pack(
            "<HHB I ffffff",
            i,  # counter
            29,
            1,
            0,
            float(i),
            float(i + 1),
            float(i + 2),
            0.0,
            0.0,
            0.0,
        )
        packets.append(packet)

    # Verify each packet decodes correctly
    for i, packet in enumerate(packets):
        decoded = FTDataBuffer.decode(packet)
        assert decoded["counter"] == i
        assert pytest.approx(decoded["fx"]) == float(i)
        assert pytest.approx(decoded["fy"]) == float(i + 1)
        assert pytest.approx(decoded["fz"]) == float(i + 2)
