# Copyright 2025 SCHUNK SE & Co. KG
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# this program. If not, see <https://www.gnu.org/licenses/>.
# --------------------------------------------------------------------------------
from schunk_fts_library.driver import Driver
from schunk_fts_library.utility import CommandResponse
import pytest
import time


@pytest.mark.parametrize(
    "command, expected_error_code",
    [
        ("40", "00"),  # Start UDP streaming
        ("41", "00"),  # Stop UDP streaming
        ("12", "00"),  # Tare
        ("13", "00"),  # Tare Reset
        ("20", "00"),  # Reset
    ],
)
def test_driver_offers_running_commands(sensor, command, expected_error_code):
    HOST, PORT = sensor
    driver = Driver(host=HOST, port=PORT)
    if not driver.connection.open():
        pytest.skip("Could not open connection to sensor.")

    try:
        cmd = command
        expected_code = expected_error_code

        response = driver.run_command(command=cmd)
        assert response.error_code == expected_code
    finally:
        # Always close connection to avoid exhausting sensor connection limit
        driver.connection.close()
        # Give sensor time to clean up TCP connection (TIME_WAIT state)
        # For reset command (20), sensor needs time to reboot
        if command == "20":
            time.sleep(5.0)  # Sensor reboot time
        else:
            time.sleep(0.1)


def test_commands_return_empty_responses_without_connection():
    wrong_port = 1234
    driver = Driver(host="0.0.0.0", port=wrong_port)

    cmd = "40"
    response = driver.run_command(command=cmd)
    assert response == CommandResponse()
