#!/usr/bin/env python3
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

import rclpy
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import WrenchStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from schunk_fts_library.driver import Driver as SensorDriver
from rclpy.timer import Timer
from functools import partial
from threading import Lock
from rclpy.publisher import Publisher


class Driver(Node):

    def __init__(self, node_name: str, **kwargs):
        super().__init__(node_name, **kwargs)

        # For force-torque data
        self.callback_group = MutuallyExclusiveCallbackGroup()

        # Parameters
        self.declare_parameter("host", "192.168.0.100")
        self.declare_parameter("port", 82)
        self.declare_parameter("streaming_port", 54843)

        self.sensor: SensorDriver = SensorDriver(
            host=self.get_parameter("host").value,
            port=self.get_parameter("port").value,
            streaming_port=self.get_parameter("streaming_port").value,
        )
        self.ft_data_publisher: Publisher | None = None
        self.timer_lock: Lock = Lock()
        self.timer: Timer = self.create_timer(
            timer_period_sec=0.001,
            callback=partial(self._publish_data),
            callback_group=self.callback_group,
        )

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug("on_configure() is called.")
        if not self.sensor.streaming_on():
            return TransitionCallbackReturn.FAILURE

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug("on_activate() is called.")

        self.ft_data_publisher = self.create_publisher(
            msg_type=WrenchStamped,
            topic="~/data",
            qos_profile=1,
            callback_group=self.callback_group,
        )
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug("on_deactivate() is called.")

        with self.timer_lock:
            self.destroy_publisher(self.ft_data_publisher)
            self.ft_data_publisher = None
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug("on_cleanup() is called.")
        self.sensor.streaming_off()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug("on_shutdown() is called.")
        self.timer.cancel()
        return SetParametersResult(successful=True)

    def _publish_data(self) -> None:
        msg = WrenchStamped()
        msg.header.frame_id = self.get_name()
        msg.header.stamp = self.get_clock().now().to_msg()
        data = self.sensor.sample()
        if data:
            msg.wrench.force.x = data["fx"]
            msg.wrench.force.y = data["fy"]
            msg.wrench.force.z = data["fz"]
            msg.wrench.torque.x = data["tx"]
            msg.wrench.torque.y = data["ty"]
            msg.wrench.torque.z = data["tz"]
        with self.timer_lock:
            if self.ft_data_publisher:
                self.ft_data_publisher.publish(msg)


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    driver = Driver("driver")
    executor.add_node(driver)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        driver.destroy_node()


if __name__ == "__main__":
    main()
