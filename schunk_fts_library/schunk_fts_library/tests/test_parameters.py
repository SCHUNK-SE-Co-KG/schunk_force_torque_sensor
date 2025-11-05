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
from schunk_fts_library.utility import GetParameterResponse, SetParameterResponse
import pytest


def test_driver_offers_getting_parameters(sensor):
    HOST, PORT = sensor
    driver = Driver(host=HOST, port=PORT)

    param = "0001"  # product_name
    expected_values = ["465453", "4b4d53"]  # 465453: FTS, 4b4d53: KMS
    response = driver.get_parameter(index=param)
    assert response.error_code == "00"
    assert response.param_value in expected_values

    # When not connected
    wrong_port = 1234
    driver = Driver(host=HOST, port=wrong_port)
    response = driver.get_parameter(index="0001")
    assert response == GetParameterResponse()


@pytest.mark.skip()  # Setting parameters requires service or root privileges
def test_driver_offers_setting_parameters(sensor):
    HOST, PORT = sensor
    driver = Driver(host=HOST, port=PORT)

    param = "0050"  # switch ft-signal units
    expected_value = "00"
    response = driver.set_parameter(value=expected_value, index=param)
    assert response.error_code == expected_value

    # When not connected
    wrong_port = 1234
    driver = Driver(host=HOST, port=wrong_port)
    response = driver.set_parameter(value=expected_value, index=param)
    assert response == SetParameterResponse()
