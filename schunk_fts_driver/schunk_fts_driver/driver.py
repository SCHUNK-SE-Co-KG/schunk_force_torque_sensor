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
from geometry_msgs.msg import WrenchStamped
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.time import Time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from schunk_fts_library.driver import Driver as SensorDriver
from schunk_fts_library.utility import FTData, FTSample
from threading import Lock
from rclpy.publisher import Publisher
from rclpy.service import Service
from threading import Thread, Event
import gc as garbage_collector
import time
from std_srvs.srv import Trigger

from schunk_fts_interfaces.srv import (  # type: ignore [attr-defined]
    SendCommand,
    SetParameter,
    SelectToolSetting,
    SelectNoiseFilter,
)
from schunk_fts_interfaces.msg import WrenchStampedBatch  # type: ignore [attr-defined]


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
        self.declare_parameter("host", "192.168.0.100")
        self.declare_parameter("port", 82)
        self.declare_parameter("streaming_port", 54843)
        self.declare_parameter("output_rate", "1000")

        output_rate = str(self.get_parameter("output_rate").value)
        self.sensor: SensorDriver = self._make_sensor_driver()
        self.ft_data_publisher: Publisher | None = None
        self.ft_state_publisher: Publisher | None = None
        self._ft_data_publisher_handle: Publisher | None = None
        self._ft_state_publisher_handle: Publisher | None = None
        self.publisher_lock: Lock = Lock()
        self.period: float = 0.0005  # sec
        self.thread: Thread = Thread()
        self.stop_event: Event = Event()

        self.send_command_service: Service | None = None
        self.set_parameter_service: Service | None = None
        self.tare_service: Service | None = None
        self.reset_tare_service: Service | None = None
        self.select_tool_setting_service: Service | None = None
        self.select_noise_filter_service: Service | None = None

        self._last_state_level: bytes | None = None
        self._base_stamp_ros: Time | None = None
        self._base_stamp_ns: int = 0
        self._last_counter: int = -1
        self._base_counter: int = -1
        self._is_sensor_ok: bool = False
        self._connection_lost: bool = False
        self._last_skip_warning_time: float = 0.0
        self._publish_sample_batches: bool = output_rate == "500_16"

    def _make_sensor_driver(self) -> SensorDriver:
        output_rate = str(self.get_parameter("output_rate").value)
        return SensorDriver(
            host=self.get_parameter("host").value,
            port=self.get_parameter("port").value,
            streaming_port=self.get_parameter("streaming_port").value,
            output_rate=output_rate,
        )

    @staticmethod
    def _packet_gap(previous_counter: int, counter: int) -> int:
        if previous_counter == -1:
            return 0
        return (counter - previous_counter - 1 + 65536) % 65536

    @staticmethod
    def _status_level_from_bits(bits: int) -> bytes:
        if bits & (1 << 3):
            return DiagnosticStatus.ERROR
        if (
            bits & (1 << 1)
            or bits & (1 << 2)
            or bits & (1 << 4)
            or bits & (1 << 5)
            or not (bits & (1 << 0))
        ):
            return DiagnosticStatus.WARN
        return DiagnosticStatus.OK

    @classmethod
    def _calculate_sample_timestamp_ns(
        cls,
        data: FTData,
        base_counter: int,
        base_stamp_ns: int,
        sample_period_ns: int,
    ) -> int:
        sample_index = int(data.get("sample_index", 0))
        samples_per_packet = int(data.get("samples_per_packet", 1))
        packet_period_ns = sample_period_ns * samples_per_packet
        counter_delta = (int(data["counter"]) - base_counter + 65536) % 65536
        return (
            base_stamp_ns
            + counter_delta * packet_period_ns
            + sample_index * sample_period_ns
        )

    @staticmethod
    def _set_stamp_from_ns(stamp, stamp_ns: int) -> None:
        stamp.sec = stamp_ns // 1_000_000_000
        stamp.nanosec = stamp_ns % 1_000_000_000

    @classmethod
    def _fill_wrench_stamped(
        cls,
        msg: WrenchStamped,
        sample: FTSample,
        stamp_ns: int,
        frame_id: str,
    ) -> WrenchStamped:
        msg.header.frame_id = frame_id
        cls._set_stamp_from_ns(msg.header.stamp, stamp_ns)
        msg.wrench.force.x = sample[5]
        msg.wrench.force.y = sample[6]
        msg.wrench.force.z = sample[7]
        msg.wrench.torque.x = sample[8]
        msg.wrench.torque.y = sample[9]
        msg.wrench.torque.z = sample[10]
        return msg

    def _publish_samples(
        self,
        data_publisher: Publisher,
        data_msg: WrenchStamped,
        samples: list[FTSample],
        packet_stamp_ns: int,
        sample_period_ns: int,
    ) -> None:
        frame_id = self.get_name()
        if self._publish_sample_batches:
            batch_msg = WrenchStampedBatch()
            batch_msg.header.frame_id = frame_id
            self._set_stamp_from_ns(batch_msg.header.stamp, packet_stamp_ns)
            batch_msg.packet_counter = samples[0][0]
            batch_msg.packet_id = samples[0][1]
            batch_msg.samples_per_packet = samples[0][3]
            batch_msg.samples = [
                self._fill_wrench_stamped(
                    WrenchStamped(),
                    sample,
                    packet_stamp_ns + sample[2] * sample_period_ns,
                    frame_id,
                )
                for sample in samples
            ]
            data_publisher.publish(batch_msg)
            return

        for sample in samples:
            self._fill_wrench_stamped(
                data_msg,
                sample,
                packet_stamp_ns + sample[2] * sample_period_ns,
                frame_id,
            )
            data_publisher.publish(data_msg)

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug("on_configure() is called.")
        self.sensor = self._make_sensor_driver()
        self.sensor.streaming_on()
        time.sleep(0.1)  # Wait for the sensor to start streaming
        level, message = self._get_status_level()
        self._is_sensor_ok = level == DiagnosticStatus.OK
        if not self.sensor.is_streaming or not self._is_sensor_ok:
            self.get_logger().error(f"Sensor not streaming or not OK: {message}")
            return TransitionCallbackReturn.FAILURE

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug("on_activate() is called.")
        garbage_collector.disable()

        sensor_data_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=64,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        if self._ft_data_publisher_handle is None:
            data_msg_type = (
                WrenchStampedBatch if self._publish_sample_batches else WrenchStamped
            )
            self._ft_data_publisher_handle = self.create_publisher(
                msg_type=data_msg_type,
                topic="~/data",
                qos_profile=sensor_data_qos,
                callback_group=self.data_callback_group,
            )
        self.ft_data_publisher = self._ft_data_publisher_handle

        latching_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        if self._ft_state_publisher_handle is None:
            self._ft_state_publisher_handle = self.create_publisher(
                msg_type=DiagnosticStatus,
                topic="~/state",
                qos_profile=latching_qos,
                callback_group=self.state_callback_group,
            )
        self.ft_state_publisher = self._ft_state_publisher_handle

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
        self.select_tool_setting_service = self.create_service(
            SelectToolSetting,
            "~/select_tool_setting",
            self._select_tool_setting_callback,
            callback_group=self.service_callback_group,
        )
        self.select_noise_filter_service = self.create_service(
            SelectNoiseFilter,
            "~/select_noise_filter",
            self._select_noise_filter_callback,
            callback_group=self.service_callback_group,
        )

        self.stop_event.clear()
        self.sensor.clear_samples()
        self._base_stamp_ros = None
        self._last_counter = -1
        self._base_counter = -1
        self.thread = Thread(target=self._publish_data)
        self.thread.start()
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug("on_deactivate() is called.")
        garbage_collector.enable()

        self.stop_event.set()
        if self.thread.is_alive():
            self.thread.join()

        with self.publisher_lock:
            self.ft_data_publisher = None
            self.ft_state_publisher = None

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
        if self.select_tool_setting_service:
            self.destroy_service(self.select_tool_setting_service)
            self.select_tool_setting_service = None
        if self.select_noise_filter_service:
            self.destroy_service(self.select_noise_filter_service)
            self.select_noise_filter_service = None

        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug("on_cleanup() is called.")
        self.sensor.streaming_off()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug("on_shutdown() is called.")
        return TransitionCallbackReturn.SUCCESS

    def _publish_data(self) -> None:
        data_msg = WrenchStamped()
        state_msg = DiagnosticStatus()
        state_msg.name = self.sensor.name
        state_msg.hardware_id = self.sensor.hardware_id
        sample_period_ns = self.sensor.output_rate_mode.sample_period_ns

        while rclpy.ok() and not self.stop_event.is_set():
            samples = self.sensor.sample_batch()

            # Check if connection was lost (data is None for extended period)
            if samples is None:
                if not self._connection_lost and self._is_sensor_ok:
                    self.get_logger().warning(
                        "Connection lost - waiting for sensor to reconnect..."
                    )
                    self._connection_lost = True
                    # Publish warning state
                    with self.publisher_lock:
                        if self.ft_state_publisher:
                            state_msg.level = DiagnosticStatus.WARN
                            state_msg.message = (
                                "Connection lost - attempting reconnection"
                            )
                            self.ft_state_publisher.publish(state_msg)
                continue

            # Connection restored
            if self._connection_lost:
                self.get_logger().info(
                    "Connection restored - resuming data publication"
                )
                self._connection_lost = False
                # Reset timestamp and counter tracking on reconnection
                self._base_stamp_ros = None
                self._last_counter = -1
                self._base_counter = -1
                # Publish OK state
                with self.publisher_lock:
                    if self.ft_state_publisher:
                        state_msg.level = DiagnosticStatus.OK
                        state_msg.message = "Connection restored"
                        self.ft_state_publisher.publish(state_msg)
                        self._last_state_level = DiagnosticStatus.OK

            data_publisher = self.ft_data_publisher
            state_publisher = self.ft_state_publisher

            first_sample = samples[0]
            counter = first_sample[0]
            samples_per_packet = first_sample[3]
            status_bits = first_sample[4]
            is_new_udp_packet = (
                self._last_counter == -1 or counter != self._last_counter
            )

            if is_new_udp_packet:
                packets_skipped = self._packet_gap(self._last_counter, counter)
                if packets_skipped > 0:
                    now = time.monotonic()
                    if now - self._last_skip_warning_time >= 1.0:
                        self._last_skip_warning_time = now
                        self.get_logger().warning(
                            f"Loop is too slow! "
                            f"Skipped {packets_skipped} packets. "
                            f"(Last: {self._last_counter}, New: {counter})"
                        )
                self._last_counter = counter

            level = self._status_level_from_bits(status_bits)
            self._is_sensor_ok = level == DiagnosticStatus.OK

            if self._is_sensor_ok and self._base_stamp_ros is None:
                self._base_stamp_ros = self.get_clock().now()
                self._base_stamp_ns = self._base_stamp_ros.nanoseconds  # type: ignore
                self._base_stamp_ns -= first_sample[2] * sample_period_ns
                self._base_counter = counter

            if self._is_sensor_ok and data_publisher:
                packet_period_ns = sample_period_ns * samples_per_packet
                counter_delta = (counter - self._base_counter + 65536) % 65536
                packet_stamp_ns = self._base_stamp_ns + counter_delta * packet_period_ns
                try:
                    self._publish_samples(
                        data_publisher,
                        data_msg,
                        samples,
                        packet_stamp_ns,
                        sample_period_ns,
                    )
                except Exception:
                    return

            if state_publisher and level != self._last_state_level:
                try:
                    _, message = self._get_status_level({"status_bits": status_bits})
                    state_msg.level = level
                    state_msg.message = message
                    self._last_state_level = level  # type: ignore
                    state_publisher.publish(state_msg)
                except Exception:
                    return

    def _get_status_level(self, data: FTData | None = None) -> tuple[bytes, str]:
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
            # Strip 0x prefix if present for compatibility
            command_id = request.command_id
            if command_id.startswith("0x") or command_id.startswith("0X"):
                command_id = command_id[2:]

            command_response = self.sensor.run_command(command_id)
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
    ) -> Trigger.Response:
        try:
            command_response = self.sensor.tare()
            error_code = command_response.error_code

            if error_code == "00":
                response.success = True
                response.message = ""
                self.get_logger().info("Tare successful.")
            else:
                response.success = False
                response.message = ERROR_CODE_MAP.get(
                    error_code, f"Unknown Error Code: {error_code}"
                )
                self.get_logger().error(f"Tare failed: {response.message}")

        except Exception as e:
            response.success = False
            response.message = f"An exception occurred during tare: {str(e)}"
            self.get_logger().error(response.message)

        return response

    def _reset_tare_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        try:
            command_response = self.sensor.tare_reset()
            error_code = command_response.error_code

            if error_code == "00":
                response.success = True
                response.message = ""
                self.get_logger().info("Tare reset successful.")
            else:
                response.success = False
                response.message = ERROR_CODE_MAP.get(
                    error_code, f"Unknown Error Code: {error_code}"
                )
                self.get_logger().error(f"Tare reset failed: {response.message}")

        except Exception as e:
            response.success = False
            response.message = (
                f"An exception occurred during command execution: {str(e)}"
            )
            self.get_logger().error(response.message)

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

    def _select_tool_setting_callback(
        self,
        request: SelectToolSetting.Request,
        response: SelectToolSetting.Response,
    ) -> SelectToolSetting.Response:
        self.get_logger().info(
            f"Received SelectToolSetting request with tool_index: {request.tool_index}"
        )
        try:
            command_response = self.sensor.select_tool_setting(request.tool_index)
            error_code = command_response.error_code

            if error_code == "00":
                response.success = True
                response.error_message = ""
                self.get_logger().info(
                    f"Tool setting {request.tool_index} selected successfully."
                )
            else:
                response.success = False
                response.error_message = ERROR_CODE_MAP.get(
                    error_code, f"Unknown Error Code: {error_code}"
                )
                self.get_logger().error(
                    f"Select tool setting failed: {response.error_message}"
                )

        except Exception as e:
            response.success = False
            response.error_message = (
                f"An exception occurred during tool setting selection: {str(e)}"
            )
            self.get_logger().error(response.error_message)

        return response

    def _select_noise_filter_callback(
        self,
        request: SelectNoiseFilter.Request,
        response: SelectNoiseFilter.Response,
    ) -> SelectNoiseFilter.Response:
        self.get_logger().info(
            "Received SelectNoiseFilter request "
            f"with filter_number: {request.filter_number}"
        )
        try:
            command_response = self.sensor.select_noise_filter(request.filter_number)
            error_code = command_response.error_code

            if error_code == "00":
                response.success = True
                response.error_message = ""
                filter_factors = [1, 2, 4, 8, 16]
                factor = (
                    filter_factors[request.filter_number]
                    if request.filter_number < len(filter_factors)
                    else request.filter_number
                )
                self.get_logger().info(
                    f"Noise filter {request.filter_number} "
                    f"(factor {factor}) selected successfully."
                )
            else:
                response.success = False
                response.error_message = ERROR_CODE_MAP.get(
                    error_code, f"Unknown Error Code: {error_code}"
                )
                self.get_logger().error(
                    f"Select noise filter failed: {response.error_message}"
                )

        except Exception as e:
            response.success = False
            response.error_message = (
                f"An exception occurred during noise filter selection: {str(e)}"
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
