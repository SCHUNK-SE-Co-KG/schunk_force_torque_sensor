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


def test_driver_has_expected_firmware(sensor):
    HOST, PORT = sensor
    driver = Driver(host=HOST, port=PORT)

    index = "1002"  # 0003, 1002
    subindex = "01"
    response = driver.get_parameter(index=index, subindex=subindex)
    assert response.error_code == "00"
    value = bytes.fromhex(response.param_value).decode("utf-8")
    print(f"\nresponse: {response}")
    print(f"value: {value}")
