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
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from schunk_fts_library.driver import Driver as SensorDriver
from threading import Lock
from rclpy.publisher import Publisher
from threading import Thread, Event
import gc as garbage_collector
import time


class Driver(Node):

    def __init__(self, node_name: str, **kwargs):
        super().__init__(node_name, **kwargs)

        # For force-torque data
        self.data_callback_group = MutuallyExclusiveCallbackGroup()
        # For sensor state
        self.state_callback_group = MutuallyExclusiveCallbackGroup()

        # Parameters
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8082)
        self.declare_parameter("streaming_port", 54843)

        self.sensor: SensorDriver = SensorDriver(
            host=self.get_parameter("host").value,
            port=self.get_parameter("port").value,
            streaming_port=self.get_parameter("streaming_port").value,
        )
        self.ft_data_publisher: Publisher | None = None
        self.publisher_lock: Lock = Lock()
        self.period: float = 0.001  # sec
        self.thread: Thread = Thread()
        self.stop_event: Event = Event()

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug("on_configure() is called.")
        if not self.sensor.streaming_on():
            return TransitionCallbackReturn.FAILURE

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug("on_activate() is called.")
        garbage_collector.disable()

        self.ft_data_publisher = self.create_publisher(
            msg_type=WrenchStamped,
            topic="~/data",
            qos_profile=1,
            callback_group=self.data_callback_group,
        )
        self.ft_state_publisher = self.create_publisher(
            msg_type=DiagnosticStatus,
            topic="~/state",
            qos_profile=1,
            callback_group=self.state_callback_group,
        )
        self.stop_event.clear()
        self.thread = Thread(target=self._publish_data)
        self.thread.start()
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug("on_deactivate() is called.")
        garbage_collector.enable()

        self.stop_event.set()
        with self.publisher_lock:
            self.destroy_publisher(self.ft_data_publisher)
            self.ft_data_publisher = None
        self.thread.join()
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug("on_cleanup() is called.")
        self.sensor.streaming_off()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug("on_shutdown() is called.")
        return SetParametersResult(successful=True)

    def _publish_data(self) -> None:
        data_msg = WrenchStamped()
        state_msg = DiagnosticStatus()
        data_msg.header.frame_id = self.get_name()
        state_msg.name = self.sensor.name
        state_msg.hardware_id = self.sensor.hardware_id
        level, message = self._get_status_level()
        state_msg.level = level
        state_msg.message = message

        next_time = time.perf_counter()

        while rclpy.ok() and not self.stop_event.is_set():
            now = time.perf_counter()
            data_msg.header.stamp = self.get_clock().now().to_msg()
            data = self.sensor.sample()
            if data:
                data_msg.wrench.force.x = data["fx"]
                data_msg.wrench.force.y = data["fy"]
                data_msg.wrench.force.z = data["fz"]
                data_msg.wrench.torque.x = data["tx"]
                data_msg.wrench.torque.y = data["ty"]
                data_msg.wrench.torque.z = data["tz"]
                state_msg.values = [
                    KeyValue(key="Packet Counter", value=str(data["counter"]))
                ]

            if now >= next_time:
                with self.publisher_lock:
                    if self.ft_data_publisher:
                        self.ft_data_publisher.publish(data_msg)
                    if self.ft_state_publisher:
                        self.ft_state_publisher.publish(state_msg)
                    next_time += self.period
            else:
                time.sleep(max(0, next_time - now))

    def _get_status_level(self) -> tuple[int, str]:
        level = self.sensor.get_status()
        match level:
            case 0:
                return (DiagnosticStatus.OK, "Ready for Operation")
            case 1:
                return (DiagnosticStatus.WARN, "Process Data Invalid")
            case 2:
                return (DiagnosticStatus.ERROR, "Temperature Out Of Range")
            case 3:
                return (DiagnosticStatus.ERROR, "Hardware Error")
            case 4:
                return (DiagnosticStatus.ERROR, "Sensor Not Connected")
            case 5:
                return (DiagnosticStatus.WARN, "User-Defined Overrange Limits Exceeded")
            case None:
                return (DiagnosticStatus.STALE, "Sensor not connected")
            case -1:
                return (DiagnosticStatus.STALE, "Sensor connected but no Data")
            case _:
                return (DiagnosticStatus.ERROR, "Unknown Error")


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
