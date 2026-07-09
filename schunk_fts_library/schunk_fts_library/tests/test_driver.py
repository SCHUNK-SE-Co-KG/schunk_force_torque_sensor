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
from schunk_fts_library.driver import (
    DEFAULT_STREAMING_PORT,
    Driver,
    OUTPUT_RATE_TO_MODE,
    _udp_destination_port_to_parameter_value,
)
from schunk_fts_library.utility import Connection
import time
import pytest
import struct


OUTPUT_RATES_UNDER_TEST = [1000, 500, 250, 100, "500_16"]
DEFAULT_OUTPUT_RATE = 1000
OUTPUT_RATE_MEASUREMENT_SEC = 1.25
OUTPUT_RATE_WARMUP_SEC = 0.25
OUTPUT_RATE_TOLERANCE = 0.25


@pytest.mark.parametrize(
    ("output_rate", "expected_enum", "expected_sample_rate", "expected_samples"),
    [
        (1000, "00", 1000, 1),
        ("500", "01", 500, 1),
        (250, "02", 250, 1),
        (100, "03", 100, 1),
        ("500_16", "0a", 8000, 16),
    ],
)
def test_driver_accepts_supported_output_rates(
    output_rate, expected_enum, expected_sample_rate, expected_samples
):
    driver = Driver(output_rate=output_rate)

    assert driver.output_rate == str(output_rate)
    assert driver.output_rate_parameter_value == expected_enum
    assert driver.output_rate_mode.sample_rate_hz == expected_sample_rate
    assert driver.output_rate_mode.samples_per_packet == expected_samples


@pytest.mark.parametrize("output_rate", [0, 10, 499, 999, 2000, 8000, "8000"])
def test_driver_rejects_unsupported_output_rates(output_rate):
    with pytest.raises(ValueError, match="Unsupported output_rate"):
        Driver(output_rate=output_rate)


@pytest.mark.parametrize(
    ("port", "expected_value"),
    [
        (DEFAULT_STREAMING_PORT, "d63b"),
        (60000, "ea60"),
    ],
)
def test_udp_destination_port_parameter_encoding(port, expected_value):
    assert _udp_destination_port_to_parameter_value(port) == expected_value


@pytest.mark.parametrize("port", [0, 65535, -1, "54843"])
def test_udp_destination_port_rejects_invalid_values(port):
    with pytest.raises(ValueError, match="UDP destination port"):
        _udp_destination_port_to_parameter_value(port)


def test_driver_configures_output_rate_parameter(monkeypatch):
    driver = Driver(output_rate="500_16")
    calls = []

    class Response:
        error_code = "00"

    def set_parameter(value: str, index: str, subindex: str = "00"):
        calls.append((value, index, subindex))
        return Response()

    monkeypatch.setattr(driver, "set_parameter", set_parameter)

    assert driver._configure_output_rate()
    assert calls == [("0a", "1020", "00")]


def _reset_sensor_to_default_output_rate(host, port):
    driver = Driver(host=host, port=port, output_rate=DEFAULT_OUTPUT_RATE)
    try:
        assert driver._configure_output_rate()
        response = driver.get_parameter(index="1020", subindex="00")
        assert response.error_code == "00"
        assert response.param_value == driver.output_rate_parameter_value
    finally:
        driver.connection.close()


def _measure_output_rate(host, port, output_rate):
    driver = Driver(host=host, port=port, output_rate=output_rate)
    samples = 0
    packets = 0
    samples_per_packet = set()

    try:
        assert driver.streaming_on(timeout_sec=2.0, auto_reconnect=False)
        time.sleep(OUTPUT_RATE_WARMUP_SEC)
        driver.clear_samples()

        start = time.perf_counter()
        deadline = start + OUTPUT_RATE_MEASUREMENT_SEC
        while time.perf_counter() < deadline:
            batch = driver.sample_batch()
            if batch is None:
                continue
            packets += 1
            samples += len(batch)
            samples_per_packet.add(batch[0][3])

        elapsed = time.perf_counter() - start
        return samples / elapsed, packets / elapsed, samples_per_packet
    finally:
        if driver.is_streaming:
            driver.streaming_off()
        else:
            driver.connection.close()
        time.sleep(0.1)


def test_driver_accepts_any_udp_source_by_default():
    driver = Driver(host="10.49.60.117")

    assert driver.streaming_source_host is None
    assert driver.stream.source_ip is None


def test_driver_supports_explicit_udp_source_host():
    driver = Driver(host="10.49.60.117", streaming_source_host="127.0.0.1")

    assert driver.streaming_source_host == "127.0.0.1"
    assert driver.stream.source_ip == "127.0.0.1"


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

    try:
        for run in range(3):
            assert not driver.stream.is_open()
            assert driver.streaming_on(), f"run: {run}"
            assert driver.is_streaming
            assert driver.stream.is_open()

            driver.streaming_off()
            assert not driver.is_streaming
            assert not driver.stream.is_open()
    finally:
        driver.streaming_off()
        time.sleep(0.1)


def test_driver_uses_same_stream_for_multiple_on_calls(sensor):
    HOST, PORT = sensor
    driver = Driver(host=HOST, port=PORT)
    try:
        driver.streaming_on()
        before = driver.stream_update_thread
        for _ in range(3):
            assert driver.streaming_on()
            after = driver.stream_update_thread
        assert after == before
    finally:
        driver.streaming_off()
        time.sleep(0.1)


def test_driver_survives_multiple_streaming_off_calls():
    driver = Driver()
    for _ in range(3):
        driver.streaming_off()
    assert not driver.is_streaming


def test_driver_runs_update_thread_when_streaming(sensor):
    HOST, PORT = sensor
    driver = Driver(host=HOST, port=PORT)
    assert not driver.stream_update_thread.is_alive()

    try:
        for _ in range(3):
            driver.streaming_on()
            assert driver.stream_update_thread.is_alive()
            driver.streaming_off()
            assert not driver.stream_update_thread.is_alive()
            time.sleep(0.01)
    finally:
        driver.streaming_off()
        time.sleep(0.1)


def test_driver_timeouts_when_streaming_fails():
    driver = Driver(streaming_port=-1)
    assert not driver.streaming_on()

    invalid_timeouts = [-1, -500.0, 0, 0.0, "15.0", ""]
    for timeout in invalid_timeouts:
        assert not driver.streaming_on(timeout_sec=timeout)


def test_driver_supports_sampling_force_torque_data(sensor, send_messages):
    HOST, PORT = sensor
    test_port = 8001
    driver = Driver(
        host=HOST,
        port=PORT,
        streaming_port=test_port,
        streaming_source_host="127.0.0.1",
    )

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

    try:
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
    finally:
        driver.streaming_off()
        time.sleep(0.1)


def test_driver_supports_sampling_at_different_rates(sensor):
    HOST, PORT = sensor
    try:
        for rate in OUTPUT_RATES_UNDER_TEST:
            driver = Driver(host=HOST, port=PORT, output_rate=rate)
            try:
                assert driver.streaming_on(timeout_sec=2.0, auto_reconnect=False)
                deadline = time.time() + 1.0
                sample = None
                while time.time() < deadline:
                    sample = driver.sample()
                    if sample is None:
                        continue
                    if rate != "500_16" or sample.get("samples_per_packet") == 16:
                        break

                assert sample is not None, f"No sample received at {rate} Hz"
                if rate == "500_16":
                    assert sample["samples_per_packet"] == 16
                else:
                    assert sample.get("samples_per_packet", 1) == 1
            finally:
                if driver.is_streaming:
                    driver.streaming_off()
                else:
                    driver.connection.close()
                time.sleep(0.1)
    finally:
        _reset_sensor_to_default_output_rate(HOST, PORT)


def test_driver_achieves_requested_output_rates(sensor):
    HOST, PORT = sensor
    try:
        for rate in OUTPUT_RATES_UNDER_TEST:
            mode = OUTPUT_RATE_TO_MODE[str(rate)]
            sample_rate, packet_rate, samples_per_packet = _measure_output_rate(
                HOST, PORT, rate
            )

            min_sample_rate = mode.sample_rate_hz * (1.0 - OUTPUT_RATE_TOLERANCE)
            max_sample_rate = mode.sample_rate_hz * (1.0 + OUTPUT_RATE_TOLERANCE)
            assert min_sample_rate <= sample_rate <= max_sample_rate, (
                f"{rate}: expected {mode.sample_rate_hz} samples/s, "
                f"measured {sample_rate:.1f} samples/s "
                f"({packet_rate:.1f} packets/s)"
            )

            min_packet_rate = mode.packet_rate_hz * (1.0 - OUTPUT_RATE_TOLERANCE)
            max_packet_rate = mode.packet_rate_hz * (1.0 + OUTPUT_RATE_TOLERANCE)
            assert min_packet_rate <= packet_rate <= max_packet_rate, (
                f"{rate}: expected {mode.packet_rate_hz} packets/s, "
                f"measured {packet_rate:.1f} packets/s "
                f"({sample_rate:.1f} samples/s)"
            )

            assert samples_per_packet == {mode.samples_per_packet}
    finally:
        _reset_sensor_to_default_output_rate(HOST, PORT)
