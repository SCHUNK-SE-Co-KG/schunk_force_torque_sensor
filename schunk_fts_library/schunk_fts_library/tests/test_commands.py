from schunk_fts_library.driver import Driver


def test_driver_offers_running_commands():
    driver = Driver()

    # 41 = Stop UDP streaming
    # 40 = Start UDP streaming
    # 20 = Reset
    # 10 = Start TCP streaming
    # 11 = Stop TCP streaming
    # 12 = Tare
    cmd = "40"
    response = driver.run_command(command=cmd)
    assert response.error_code == "00"
    print(response)
