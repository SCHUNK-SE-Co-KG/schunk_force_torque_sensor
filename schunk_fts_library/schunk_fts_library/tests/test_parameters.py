from schunk_fts_library.driver import Driver


def test_driver_offers_getting_parameters():
    driver = Driver()
    driver.connect(host="192.168.0.100", port=82)

    param = "100a"  # current user level
    expected_value = "00"
    response = driver.get_parameter(index=param)
    print(f"response: {response}")
    assert response.error_code == "00"
    assert response.param_value == expected_value


def test_driver_offers_setting_parameters():
    driver = Driver()
    driver.connect(host="192.168.0.100", port=82)

    param = "1031"  # needs root access
    expected_value = "01"
    response = driver.set_parameter(value=expected_value, index=param)
    assert response.error_code == "10"  # user level not sufficient
    # assert response.param_value == expected_value
