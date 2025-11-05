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
from schunk_fts_library.driver import Driver
from schunk_fts_library.utility import Connection
import time
import pytest
import struct


def test_driver_initializes_as_expected():

    # Default initialization
    driver = Driver()
    assert isinstance(driver.connection, Connection)
    assert driver.connection.host == "192.168.0.100"
    assert driver.connection.port == 82
    assert not driver.connection.is_open

    # With arguments
    host = "some-arbitrary string $\n#^^"
    port = -12345

    driver = Driver(host=host, port=port)
    assert driver.connection.host == host
    assert driver.connection.port == port


def test_driver_offers_streaming(sensor):
    HOST, PORT = sensor
    driver = Driver(host=HOST, port=PORT)
    assert not driver.is_streaming

    for run in range(3):
        assert not driver.stream.is_open()
        assert driver.streaming_on(), f"run: {run}"
        assert driver.is_streaming
        assert driver.stream.is_open()

        driver.streaming_off()
        assert not driver.is_streaming
        assert not driver.stream.is_open()


def test_driver_uses_same_stream_for_multiple_on_calls(sensor):
    HOST, PORT = sensor
    driver = Driver(host=HOST, port=PORT)
    driver.streaming_on()
    before = driver.stream_update_thread
    for _ in range(3):
        assert driver.streaming_on()
        after = driver.stream_update_thread
    assert after == before


def test_driver_survives_multiple_streaming_off_calls():
    driver = Driver()
    for _ in range(3):
        driver.streaming_off()
    assert not driver.is_streaming


def test_driver_runs_update_thread_when_streaming(sensor):
    HOST, PORT = sensor
    driver = Driver(host=HOST, port=PORT)
    assert not driver.stream_update_thread.is_alive()

    for _ in range(3):
        driver.streaming_on()
        assert driver.stream_update_thread.is_alive()
        driver.streaming_off()
        assert not driver.stream_update_thread.is_alive()
        time.sleep(0.01)


def test_driver_timeouts_when_streaming_fails():
    driver = Driver(streaming_port=-1)
    assert not driver.streaming_on()

    invalid_timeouts = [-1, -500.0, 0, 0.0, "15.0", ""]
    for timeout in invalid_timeouts:
        assert not driver.streaming_on(timeout_sec=timeout)


def test_driver_supports_sampling_force_torque_data(sensor, send_messages):
    HOST, PORT = sensor
    test_port = 8001
    driver = Driver(host=HOST, port=PORT, streaming_port=test_port)

    # Not streaming
    assert driver.sample() is None

    # Stream a specific data point and check
    # that we sample that.
    assert driver.streaming_on()
    data = {
        "sync": b"\xFF\xFF",
        "counter": 42,
        "length": 29,
        "id": 1,
        "status_bits": 0x00000000,
        "fx": 1.0,
        "fy": 2.1,
        "fz": 3.3,
        "tx": 0.04,
        "ty": -17.358,
        "tz": 23.001,
    }

    # Build binary packet manually
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

    send_messages(test_port, [packet])
    time.sleep(0.1)  # allow driver to read from socket

    result = driver.sample()
    assert result is not None
    assert result["id"] == data["id"]
    assert result["status_bits"] == data["status_bits"]
    assert pytest.approx(result["fx"]) == data["fx"]
    assert pytest.approx(result["fy"]) == data["fy"]
    assert pytest.approx(result["fz"]) == data["fz"]
    assert pytest.approx(result["tx"]) == data["tx"]
    assert pytest.approx(result["ty"]) == data["ty"]
    assert pytest.approx(result["tz"]) == data["tz"]


@pytest.mark.skip()
def test_driver_supports_sampling_at_different_rates(sensor):
    HOST, PORT = sensor
    driver = Driver()

    # Test streaming at different rates
    assert driver.streaming_on()
    rates = [10, 100, 1000, 8000]
    duration_sec = 1.0

    for rate in rates:
        start = time.time()
        previous_value = None
        count = 0
        while time.time() < start + duration_sec:
            sample = driver.sample()
            count += 1
            assert sample != previous_value, f"rate: {rate}, count: {count}"
            previous_value = sample
            time.sleep(1.0 / rate)
