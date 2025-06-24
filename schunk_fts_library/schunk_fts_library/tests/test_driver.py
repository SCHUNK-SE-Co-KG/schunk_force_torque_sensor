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
        driver.streaming_on()
        time.sleep(0.1)
        assert driver.is_streaming
        assert driver.stream.is_open()

        driver.streaming_off()
        time.sleep(0.1)
        assert not driver.is_streaming
        assert not driver.stream.is_open()


def test_driver_uses_same_stream_for_multiple_on_calls():
    driver = Driver()
    driver.streaming_on()
    before = driver.update_thread
    for _ in range(3):
        driver.streaming_on()
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

        time.sleep(0.1)  # wait to take effect
        assert not driver.update_thread.is_alive()
