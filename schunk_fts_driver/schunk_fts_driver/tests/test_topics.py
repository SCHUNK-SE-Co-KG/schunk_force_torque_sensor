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

from lifecycle_msgs.msg import Transition, State
import time
import rclpy
from geometry_msgs.msg import WrenchStamped
from functools import partial
import pytest
import os

# Module-wide settings.
# The `driver` fixture uses these variables to
# launch the driver with specific connection settings.
HOST = os.getenv("FTS_HOST", "192.168.0.100")
PORT = int(os.getenv("FTS_PORT", 82))


def test_driver_advertises_state_depending_topics(sensor, lifecycle_interface):
    driver = lifecycle_interface
    topics = ["data"]
    until_change_takes_effect = 0.1

    def exist(topics: list[str]) -> bool:
        existing = driver.node.get_topic_names_and_types()
        advertised = [i[0] for i in existing]
        for topic in topics:
            if f"/schunk/driver/{topic}" not in advertised:
                return False
        return True

    for run in range(3):

        # After startup -> unconfigured
        driver.check_state(State.PRIMARY_STATE_UNCONFIGURED)

        # After configure -> inactive
        driver.change_state(Transition.TRANSITION_CONFIGURE)
        time.sleep(until_change_takes_effect)
        assert not exist(topics)

        # After activate -> active
        driver.change_state(Transition.TRANSITION_ACTIVATE)
        time.sleep(until_change_takes_effect)
        assert exist(topics)

        # After deactivate -> inactive
        driver.change_state(Transition.TRANSITION_DEACTIVATE)
        time.sleep(until_change_takes_effect)
        assert not exist(topics)

        # After cleanup -> unconfigured
        driver.change_state(Transition.TRANSITION_CLEANUP)
        time.sleep(until_change_takes_effect)


def test_driver_publishes_force_torque_data(lifecycle_interface):
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)
    messages = []

    def check_fields(msg: WrenchStamped, messages: list[WrenchStamped]) -> None:
        messages.append(msg)
        assert msg.header.stamp
        assert msg.header.frame_id
        assert pytest.approx(msg.wrench.force.x) != 0.0
        assert pytest.approx(msg.wrench.force.y) != 0.0
        assert pytest.approx(msg.wrench.force.z) != 0.0
        assert pytest.approx(msg.wrench.torque.x) != 0.0
        assert pytest.approx(msg.wrench.torque.y) != 0.0
        assert pytest.approx(msg.wrench.torque.z) != 0.0

    _ = driver.node.create_subscription(
        WrenchStamped,
        "/schunk/driver/data",
        partial(check_fields, messages=messages),
        1,
    )

    timeout = time.time() + 1.0
    while time.time() < timeout and len(messages) == 0:
        rclpy.spin_once(driver.node, timeout_sec=0.1)

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)

    assert len(messages) >= 1
