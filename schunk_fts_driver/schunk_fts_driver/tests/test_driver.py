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
from schunk_fts_driver.driver import Driver


def test_driver_uses_dedicated_callback_group_for_publishing_ft_data(ros2):
    driver = Driver("driver")

    driver.on_configure(state=None)
    driver.on_activate(state=None)

    for handler in driver.ft_data_publisher.event_handlers:
        assert handler.callback_group != driver.default_callback_group
        assert handler.callback_group == driver.callback_group

    driver.on_deactivate(state=None)
    driver.on_cleanup(state=None)
