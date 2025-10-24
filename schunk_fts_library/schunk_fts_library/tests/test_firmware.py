from schunk_fts_library.driver import Driver
import os

HOST = os.getenv("FTS_HOST", "192.168.0.100")
PORT = int(os.getenv("FTS_PORT", 82))


def test_driver_has_expected_firmware(sensor):
    driver = Driver(host=HOST, port=PORT)

    index = "1002"  # 0003, 1002
    subindex = "01"
    response = driver.get_parameter(index=index, subindex=subindex)
    assert response.error_code == "00"
    value = bytes.fromhex(response.param_value).decode("utf-8")
    print(f"\nresponse: {response}")
    print(f"value: {value}")
