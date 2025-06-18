from schunk_fts_library.driver import Driver
import os

HOST = os.getenv("FTS_HOST", "192.168.0.100")
PORT = int(os.getenv("FTS_PORT", 82))


def test_driver_offers_getting_parameters(sensor):
    driver = Driver(host=HOST, port=PORT)

    param = "0001"  # product_name
    expected_value = "4b4d53"  # 465453: FTS, 4b4d53: KMS
    response = driver.get_parameter(index=param)
    assert response.error_code == "00"
    assert response.param_value == expected_value


def test_driver_offers_setting_parameters(sensor):
    driver = Driver(host=HOST, port=PORT)

    param = "0050"  # switch ft-signal units
    expected_value = "00"
    response = driver.set_parameter(value=expected_value, index=param)
    assert response.error_code == expected_value
