from schunk_fts_library.driver import Driver


def test_driver_offers_running_commands():
    driver = Driver()
    driver.connect(host="192.168.0.100", port=82)

    cmd = "10"  # 40 = UDP streaming, 10 = TCP streaming
    response = driver.run_command(command=cmd)
    assert response.error_code == "00"
    print(response)
    driver.disconnect()
