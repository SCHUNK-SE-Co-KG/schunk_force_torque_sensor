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
from threading import Thread
import time


def test_driver_uses_dedicated_callback_group_for_publishing_ft_data(ros2):
    driver = Driver("driver")

    driver.on_configure(state=None)
    driver.on_activate(state=None)

    for handler in driver.ft_data_publisher.event_handlers:
        assert handler.callback_group != driver.default_callback_group
        assert handler.callback_group == driver.callback_group

    driver.on_deactivate(state=None)
    driver.on_cleanup(state=None)


def test_driver_uses_library_for_sensor_communication(sensor, ros2):
    driver = Driver("driver")
    assert driver.sensor is not None


def test_driver_streams_when_configured(sensor, ros2):
    driver = Driver("driver")

    driver.on_configure(state=None)
    assert driver.sensor.is_streaming
    driver.on_cleanup(state=None)
    assert not driver.sensor.is_streaming


def test_driver_manages_a_timer_for_publishing(sensor, ros2):
    driver = Driver("driver")

    assert driver.timer is not None

    # Check that we re-use the same timer during lifetime
    initial_joints_timer = driver.timer
    for _ in range(3):
        driver.on_configure(state=None)
        driver.on_activate(state=None)
        driver.on_deactivate(state=None)
        driver.on_cleanup(state=None)
        assert driver.timer == initial_joints_timer

    driver.on_shutdown(state=None)
    assert driver.timer.is_canceled()


def test_publisher_variable_always_exists(sensor, ros2):
    driver = Driver("test_publisher_variable")
    for _ in range(3):
        assert driver.ft_data_publisher is None
        driver.on_configure(state=None)
        assert driver.ft_data_publisher is None

        driver.on_activate(state=None)
        assert driver.ft_data_publisher is not None

        driver.on_deactivate(state=None)
        assert driver.ft_data_publisher is None
        driver.on_cleanup(state=None)
        assert driver.ft_data_publisher is None


def test_timer_callbacks_dont_collide_with_lifecycle_transitions(sensor, ros2):
    driver = Driver("test_timer_collisions")
    driver.on_configure(state=None)

    # Mimic the timers' callbacks by explicitly calling the publish methods
    done = False

    def stay_busy() -> None:
        while not done:
            driver._publish_data()

    timer_thread = Thread(target=stay_busy)
    timer_thread.start()

    start = time.time()
    while time.time() < start + 2.0:
        driver.on_activate(state=None)
        driver.on_deactivate(state=None)
    done = True

    timer_thread.join()
    driver.on_cleanup(state=None)
