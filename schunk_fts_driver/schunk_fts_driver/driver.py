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
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import WrenchStamped
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from schunk_fts_library.driver import Driver as SensorDriver
from schunk_fts_library.utility import FTData
from threading import Lock
from rclpy.publisher import Publisher
from rclpy.service import Service
from threading import Thread, Event
import gc as garbage_collector
import time
from example_interfaces.srv import Trigger

from schunk_fts_interfaces.srv import (  # type: ignore [attr-defined]
    SendCommand,
    SetParameter,
)


# Error codes from the Interface Control Document
ERROR_CODE_MAP = {
    # Commands
    "00": "Success (No Error)",
    "01": "Unknown Command",
    "02": "Invalid Command Length",
    "03": "Invalid Command Value",
    "04": "Busy",
    "05": "Streaming Active",
    "06": "Storage Error",
    "07": "Internal Bus Error",
    "08": "Timeout",
    # Parameter
    "10": "User Level not Sufficient",
    "11": "Is Read Only",
    "12": "Is Write Only",
    "13": "Index Does not Exist",
    "14": "Subindex Does not Exist",
    "15": "Invalid Parameter Value Length",
    "16": "Invalid Parameter Value",
    "17": "Login Failed",
    "18": "Login Blocked, Try Again Later",
    "19": "Parameters are locked",
    # Firmware Update
    "30": "Update Verification Failed",
    "31": "Update State Error",
}


class Driver(Node):

    def __init__(self, node_name: str, **kwargs):
        super().__init__(node_name, **kwargs)

        # For force-torque data
        self.data_callback_group = MutuallyExclusiveCallbackGroup()
        # For sensor state
        self.state_callback_group = MutuallyExclusiveCallbackGroup()
        # For services
        self.service_callback_group = MutuallyExclusiveCallbackGroup()

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
        self.period: float = 0.0005  # sec
        self.thread: Thread = Thread()
        self.stop_event: Event = Event()

        self.send_command_service: Service | None = None
        self.set_parameter_service: Service | None = None
        self.tare_service: Service | None = None
        self.reset_tare_service: Service | None = None

        self._last_state_level = None
        self._base_stamp = None
        self._last_conter = -1
        self._is_sensor_ok = False

        self._last_data = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug("on_configure() is called.")
        self.sensor.streaming_on()
        time.sleep(0.1)  # Wait for the sensor to start streaming
        level, message = self._get_status_level()
        self._is_sensor_ok = level == DiagnosticStatus.OK
        if not self.sensor.is_streaming or not self._is_sensor_ok:
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

        latching_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.ft_state_publisher = self.create_publisher(
            msg_type=DiagnosticStatus,
            topic="~/state",
            qos_profile=latching_qos,
            callback_group=self.state_callback_group,
        )

        # Create services
        self.send_command_service = self.create_service(
            SendCommand,
            "~/send_command",
            self._send_command_callback,
            callback_group=self.service_callback_group,
        )
        self.set_parameter_service = self.create_service(
            SetParameter,
            "~/set_parameter",
            self._set_parameter_callback,
            callback_group=self.service_callback_group,
        )
        self.tare_service = self.create_service(
            Trigger,
            "~/tare",
            self._tare_callback,
            callback_group=self.service_callback_group,
        )
        self.reset_tare_service = self.create_service(
            Trigger,
            "~/reset_tare",
            self._reset_tare_callback,
            callback_group=self.service_callback_group,
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

        # Destroy services
        if self.send_command_service:
            self.destroy_service(self.send_command_service)
            self.send_command_service = None
        if self.set_parameter_service:
            self.destroy_service(self.set_parameter_service)
            self.set_parameter_service = None
        if self.tare_service:
            self.destroy_service(self.tare_service)
            self.tare_service = None
        if self.reset_tare_service:
            self.destroy_service(self.reset_tare_service)
            self.reset_tare_service = None

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

        next_time = time.perf_counter()

        while rclpy.ok() and not self.stop_event.is_set():
            now = time.perf_counter()

            data = self.sensor.sample()
            level, message = self._get_status_level(data)
            self._is_sensor_ok = level == DiagnosticStatus.OK

            if self._is_sensor_ok:
                if not data or data == self._last_data:
                    time.sleep(0.0001)
                    continue
                self._last_data = data  # type: ignore
                counter = data["counter"]

                if self._base_stamp is None or counter < self._last_conter:
                    self._base_stamp = self.get_clock().now()

                offset_ns = (
                    counter * 1000 * 1000
                )  # microseconds sampling period (1 kHz)
                offset_duration = rclpy.duration.Duration(nanoseconds=offset_ns)
                stamp = self._base_stamp + offset_duration
                data_msg.header.stamp = stamp.to_msg()
                self._last_conter = counter

                data_msg.wrench.force.x = data["fx"]
                data_msg.wrench.force.y = data["fy"]
                data_msg.wrench.force.z = data["fz"]
                data_msg.wrench.torque.x = data["tx"]
                data_msg.wrench.torque.y = data["ty"]
                data_msg.wrench.torque.z = data["tz"]

            if now >= next_time:
                with self.publisher_lock:
                    if self.ft_data_publisher and self._is_sensor_ok:
                        self.ft_data_publisher.publish(data_msg)
                    if self.ft_state_publisher:
                        if level != self._last_state_level:
                            state_msg.level = level
                            state_msg.message = message
                            self._last_state_level = level  # type: ignore
                            self.ft_state_publisher.publish(state_msg)
                    next_time += self.period
            else:
                time.sleep(max(0, next_time - now))

    def _get_status_level(self, data: FTData | None = None) -> tuple[int, str]:
        status = self.sensor.get_status(data)
        if status is None:
            return (DiagnosticStatus.ERROR, "No signal")

        messages = status.summary()

        if status.hardware_error:
            return (DiagnosticStatus.ERROR, "; ".join(messages))
        elif status.temperature_out_of_range or status.process_data_invalid:
            return (DiagnosticStatus.WARN, "; ".join(messages))
        elif not status.ready:
            return (DiagnosticStatus.WARN, "Not ready for operation")
        elif messages:
            return (DiagnosticStatus.WARN, "; ".join(messages))
        else:
            return (DiagnosticStatus.OK, "OK")

    def _send_command_callback(
        self, request: SendCommand.Request, response: SendCommand.Response
    ) -> SendCommand.Response:
        self.get_logger().info(
            f"Received SendCommand request with command_id: '{request.command_id}'"
        )
        try:
            command_response = self.sensor.run_command(request.command_id)
            error_code = command_response.error_code

            if error_code == "00":
                response.success = True
                response.error_message = ""
                self.get_logger().info("Command successful.")
            else:
                response.success = False
                response.error_message = ERROR_CODE_MAP.get(
                    error_code, f"Unknown Error Code: {error_code}"
                )
                self.get_logger().error(f"Command failed: {response.error_message}")

        except Exception as e:
            response.success = False
            response.error_message = (
                f"An exception occurred during command execution: {str(e)}"
            )
            self.get_logger().error(response.error_message)

        return response

    def _tare_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> SendCommand.Response:
        try:
            command_response = self.sensor.tare()
            error_code = command_response.error_code

            if error_code == "00":
                response.success = True
                response.error_message = ""
                self.get_logger().info("Tare successful.")
            else:
                response.success = False
                response.error_message = ERROR_CODE_MAP.get(
                    error_code, f"Unknown Error Code: {error_code}"
                )
                self.get_logger().error(f"Tare failed: {response.error_message}")

        except Exception as e:
            response.success = False
            response.error_message = f"An exception occurred during tare: {str(e)}"
            self.get_logger().error(response.error_message)

        return response

    def _reset_tare_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> SendCommand.Response:
        try:
            command_response = self.sensor.tare_reset()
            error_code = command_response.error_code

            if error_code == "00":
                response.success = True
                response.error_message = ""
                self.get_logger().info("Tare reset successful.")
            else:
                response.success = False
                response.error_message = ERROR_CODE_MAP.get(
                    error_code, f"Unknown Error Code: {error_code}"
                )
                self.get_logger().error(f"Tare reset failed: {response.error_message}")

        except Exception as e:
            response.success = False
            response.error_message = (
                f"An exception occurred during command execution: {str(e)}"
            )
            self.get_logger().error(response.error_message)

        return response

    def _set_parameter_callback(
        self, request: SetParameter.Request, response: SetParameter.Response
    ) -> SetParameter.Response:
        self.get_logger().info(
            f"Received SetParameter request: "
            f"index='{request.param_index}', subindex='{request.param_subindex}'"
            f"value='{request.param_value}'"
        )
        try:
            set_param_response = self.sensor.set_parameter(
                value=request.param_value,
                index=request.param_index,
                subindex=request.param_subindex,
            )
            error_code = set_param_response.error_code

            if error_code == "00":
                response.success = True
                response.error_message = ""
                self.get_logger().info("SetParameter successful.")
            else:
                response.success = False
                response.error_message = ERROR_CODE_MAP.get(
                    error_code, f"Unknown Error Code: {error_code}"
                )
                self.get_logger().error(
                    f"SetParameter failed: {response.error_message}"
                )

        except Exception as e:
            response.success = False
            response.error_message = (
                f"An exception occurred during parameter setting: {str(e)}"
            )
            self.get_logger().error(response.error_message)

        return response


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
