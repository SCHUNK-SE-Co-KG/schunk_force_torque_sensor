from schunk_fts_library.driver import Driver
from schunk_fts_library.utility import CommandResponse
import os

HOST = os.getenv("FTS_HOST", "192.168.0.100")
PORT = int(os.getenv("FTS_PORT", 82))


def test_driver_offers_running_commands(sensor):
    driver = Driver(host=HOST, port=PORT)

    # 41 = Stop UDP streaming
    # 40 = Start UDP streaming
    # 20 = Reset
    # 10 = Start TCP streaming
    # 11 = Stop TCP streaming
    # 12 = Tare
    commands = ["40", "41", "10", "11", "12"]
    for cmd in commands:
        response = driver.run_command(command=cmd)
        assert response.error_code == "00"


def test_commands_return_empty_responses_without_connection():
    wrong_port = 1234
    driver = Driver(host="0.0.0.0", port=wrong_port)

    cmd = "40"
    response = driver.run_command(command=cmd)
    assert response == CommandResponse()
