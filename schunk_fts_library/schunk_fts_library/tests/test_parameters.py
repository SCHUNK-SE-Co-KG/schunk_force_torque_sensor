from schunk_fts_library.driver import Driver


def test_driver_offers_getting_parameters():
    driver = Driver()
    driver.connect(host="192.168.0.100", port=82)

    param = "0001"  # product_name
    expected_value = "465453"  # FTS
    response = driver.get_parameter(index=param)
    assert response.error_code == "00"
    assert response.param_value == expected_value


def test_driver_offers_setting_parameters():
    driver = Driver()
    driver.connect(host="192.168.0.100", port=82)

    param = "0050"  # switch ft-signal units
    expected_value = "00"
    response = driver.set_parameter(value=expected_value, index=param)
    assert response.error_code == "10"  # user level not sufficient
