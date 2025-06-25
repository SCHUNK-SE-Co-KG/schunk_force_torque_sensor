from schunk_fts_library.driver import Driver
from schunk_fts_library.utility import Connection
import time


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


def test_driver_offers_streaming():
    driver = Driver()
    assert not driver.is_streaming

    for _ in range(3):
        assert not driver.stream.is_open()
        assert driver.streaming_on()
        assert driver.is_streaming
        assert driver.stream.is_open()

        driver.streaming_off()
        assert not driver.is_streaming
        assert not driver.stream.is_open()


def test_driver_uses_same_stream_for_multiple_on_calls():
    driver = Driver()
    driver.streaming_on()
    before = driver.update_thread
    for _ in range(3):
        assert driver.streaming_on()
        after = driver.update_thread
    assert after == before


def test_driver_survives_multiple_streaming_off_calls():
    driver = Driver()
    for _ in range(3):
        driver.streaming_off()
    assert not driver.is_streaming


def test_driver_runs_update_thread_when_streaming():
    driver = Driver()
    assert not driver.update_thread.is_alive()

    for _ in range(3):
        driver.streaming_on()
        assert driver.update_thread.is_alive()
        driver.streaming_off()
        assert not driver.update_thread.is_alive()


def test_driver_timeouts_when_streaming_fails():
    driver = Driver(streaming_port=-1)
    assert not driver.streaming_on()

    invalid_timeouts = [-1, -500.0, 0, 0.0, "15.0", ""]
    for timeout in invalid_timeouts:
        assert not driver.streaming_on(timeout_sec=timeout)


def test_driver_supports_sampling_force_torque_data(sensor):
    driver = Driver()

    # Not streaming
    assert driver.sample() is None

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
