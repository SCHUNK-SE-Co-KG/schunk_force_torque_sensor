from schunk_fts_library.driver import Driver


def test_driver_initializes_as_expected():
    driver = Driver()
    assert driver.host == "192.168.0.100"
    assert driver.port == 82
    assert driver.socket is not None
    assert not driver.connected


def test_driver_implements_connect_and_disconnect():
    driver = Driver()

    assert driver.connect(host="192.168.0.100", port=82)

    # Repeated connect to same address
    for _ in range(3):
        assert driver.connect(host="192.168.0.100", port=82)

    # Reconnect to other address without disconnect
    other_host = "0.0.0.0"
    assert not driver.connect(host=other_host, port=82)

    # Regular disconnect
    assert driver.connect(host="192.168.0.100", port=82)
    assert driver.disconnect()

    # Repeated disconnects
    for _ in range(3):
        assert driver.disconnect()

    # Several complete runs
    for _ in range(3):
        assert driver.connect(host="192.168.0.100", port=82)
        assert driver.disconnect()
