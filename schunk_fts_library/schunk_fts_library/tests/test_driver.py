from schunk_fts_library.driver import Driver
from schunk_fts_library.utility import Connection, FTDataBuffer
import time
import pytest
import os


HOST = os.getenv("FTS_HOST", "192.168.0.100")
PORT = int(os.getenv("FTS_PORT", 82))


def test_driver_initializes_as_expected():

    # Default initialization
    driver = Driver()
    assert isinstance(driver.connection, Connection)
    assert driver.connection.host == "192.168.0.100"
    assert driver.connection.port == 82
    assert not driver.connection.is_connected

    # With arguments
    host = "some-arbitrary string $\n#^^"
    port = -12345

    driver = Driver(host=host, port=port)
    assert driver.connection.host == host
    assert driver.connection.port == port


def test_driver_offers_streaming(sensor):
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
    driver = Driver(host=HOST, port=PORT)
    assert not driver.stream_update_thread.is_alive()

    for _ in range(3):
        driver.streaming_on()
        assert driver.stream_update_thread.is_alive()
        driver.streaming_off()
        assert not driver.stream_update_thread.is_alive()


def test_driver_timeouts_when_streaming_fails():
    driver = Driver(streaming_port=-1)
    assert not driver.streaming_on()

    invalid_timeouts = [-1, -500.0, 0, 0.0, "15.0", ""]
    for timeout in invalid_timeouts:
        assert not driver.streaming_on(timeout_sec=timeout)


def test_driver_supports_sampling_force_torque_data(sensor, send_messages):
    test_port = 8001
    driver = Driver(host=HOST, port=PORT, streaming_port=test_port)

    # Not streaming
    assert driver.sample() is None

    # Stream a specific data point and check
    # that we sample that.
    assert driver.streaming_on()
    data = {
        "sync": 0xFFFF,
        "counter": 42,
        "payload": 29,
        "id": 1,
        "status_bits": 0x00000000,
        "fx": 1.0,
        "fy": 2.1,
        "fz": 3.3,
        "tx": 0.04,
        "ty": -17.358,
        "tz": 23.001,
    }
    buffer = FTDataBuffer()
    msg = buffer.encode(data=data)
    send_messages(test_port, [msg])
    time.sleep(0.1)

    result = driver.sample()
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
