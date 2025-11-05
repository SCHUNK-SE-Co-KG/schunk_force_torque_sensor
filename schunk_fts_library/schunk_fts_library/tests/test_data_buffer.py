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
from schunk_fts_library.utility import FTDataBuffer, FTData
from threading import Thread
import pytest
import struct
import time


def test_buffer_offers_putting_and_getting_data():
    buffer = FTDataBuffer()

    # Without data
    data = buffer.get()
    if not data:
        assert data is None
    else:
        assert data["id"] == 0
        assert data["status_bits"] == 0
        assert pytest.approx(data["fx"]) == 0.0
        assert pytest.approx(data["fy"]) == 0.0
        assert pytest.approx(data["fz"]) == 0.0
        assert pytest.approx(data["tx"]) == 0.0
        assert pytest.approx(data["ty"]) == 0.0
        assert pytest.approx(data["tz"]) == 0.0

    # Put + get
    data_bytes = bytearray(
        [
            0xFF,
            0xFF,  # sync (2 bytes, little-endian)
            0x2A,
            0x00,  # counter (42, 2 bytes, little-endian)
            0x1D,
            0x00,  # payload (29, 2 bytes, little-endian)
            0x00,  # id (0, 1 byte)
            0x00,
            0x00,
            0x00,
            0x00,  # status_bits (0, 4 bytes, little-endian)
            0x00,
            0x00,
            0x80,
            0x3F,  # fx (1.0, 4 bytes, IEEE 754 little-endian)
            0x00,
            0x00,
            0x00,
            0x40,  # fy (2.0, 4 bytes, little-endian)
            0x00,
            0x00,
            0x40,
            0x40,  # fz (3.0, 4 bytes, little-endian)
            0x00,
            0x00,
            0x80,
            0x40,  # tx (4.0, 4 bytes, little-endian)
            0x00,
            0x00,
            0xA0,
            0x40,  # ty (5.0, 4 bytes, little-endian)
            0x00,
            0x00,
            0xC0,
            0x40,  # tz (6.0, 4 bytes, little-endian)
        ]
    )
    buffer.put(packet=data_bytes)  # Expects bytearry
    data = FTData(
        sync=bytearray([0xFF, 0xFF]),
        counter=42,
        payload=29,
        id=0,
        status_bits=0,
        fx=1.0,
        fy=2.0,
        fz=3.0,
        tx=4.0,
        ty=5.0,
        tz=6.0,
    )
    assert buffer.get() == data


def test_buffer_knows_expected_length():
    buffer = FTDataBuffer(maxsize=3)

    # Initially, the buffer should be empty
    assert len(buffer) == 0

    # Add one packet
    buffer.put(bytearray(b"\x00" * 32))
    assert len(buffer) == 1

    # Fill it up to maxsize
    buffer.put(bytearray(b"\x00" * 32))
    buffer.put(bytearray(b"\x00" * 32))
    assert len(buffer) == 3

    # Try to overfill — buffer should drop packets, but size stays constant
    buffer.put(bytearray(b"\x00" * 32))
    assert len(buffer) == 3


def test_buffer_supports_concurrent_accesses():
    buffer = FTDataBuffer(maxsize=100)
    nr_iterations = 10_000  # keep runtime short for CI

    # Helper: create valid packet
    def make_packet(counter: int, id_: int, fx: float) -> bytearray:
        payload = struct.pack(
            "<HHB I ffffff",
            counter,  # counter
            29,  # length
            id_,  # id
            0,  # status_bits
            fx,
            fx,
            fx,
            fx,
            fx,
            fx,  # fx..tz
        )
        return bytearray(b"\xFF\xFF") + payload

    packet0 = make_packet(1, 0, 1.1)
    packet1 = make_packet(2, 1, 2.2)

    exception = None

    def writer():
        for n in range(nr_iterations):
            packet = packet0 if n % 2 == 0 else packet1
            buffer.put(packet)
            time.sleep(0.0001)  # simulate 10kHz rate

    def reader():
        nonlocal exception
        for _ in range(nr_iterations):
            data = buffer.get()
            if data is None:
                continue  # skip empty cycles
            try:
                assert isinstance(data, dict)
                assert data["sync"] == b"\xFF\xFF"
                assert data["payload"] == 29
                assert data["id"] in (0, 1)
                assert pytest.approx(data["fx"]) in (1.1, 2.2)
            except AssertionError as e:
                exception = e
                break

    writer_thread = Thread(target=writer, daemon=True)
    reader_thread = Thread(target=reader, daemon=True)

    writer_thread.start()
    reader_thread.start()

    writer_thread.join(timeout=5)
    reader_thread.join(timeout=5)

    assert not writer_thread.is_alive()
    assert not reader_thread.is_alive()

    if exception:
        raise exception
