from schunk_fts_library.driver import Driver


def test_driver_offers_getting_parameters():
    driver = Driver()
    driver.connect(host="192.168.0.100", port=82)

    param = "1031"
    result = driver.get_parameter(index=param)
    assert result

    driver.disconnect()
