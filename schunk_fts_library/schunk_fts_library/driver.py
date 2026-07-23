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
from schunk_fts_library.utility import (
    Connection,
    Stream,
    FTData,
    FTSample,
    FTDataBuffer,
    SetParameterRequest,
    SetParameterResponse,
    GetParameterRequest,
    GetParameterResponse,
    CommandRequest,
    CommandResponse,
    CommandWithParameterRequest,
)
from threading import Thread, Lock
import asyncio
import time
from dataclasses import dataclass
import struct


OUTPUT_RATE_PARAMETER_INDEX = "1020"
OUTPUT_RATE_PARAMETER_SUBINDEX = "00"
SUPPORTED_OUTPUT_RATES = ("1000", "500", "250", "100", "500_16")
UDP_DESTINATION_PORT_PARAMETER_INDEX = "1033"
UDP_DESTINATION_PORT_PARAMETER_SUBINDEX = "00"
DEFAULT_STREAMING_PORT = 54843
MIN_UDP_DESTINATION_PORT = 1
MAX_UDP_DESTINATION_PORT = 65534


@dataclass(frozen=True)
class OutputRateMode:
    output_rate: str
    parameter_value: str
    packet_rate_hz: int
    sample_rate_hz: int
    samples_per_packet: int

    @property
    def sample_period_ns(self) -> int:
        return round(1_000_000_000 / self.sample_rate_hz)

    @property
    def packet_period_ns(self) -> int:
        return self.sample_period_ns * self.samples_per_packet


OUTPUT_RATE_TO_MODE = {
    "1000": OutputRateMode("1000", "00", 1000, 1000, 1),
    "500": OutputRateMode("500", "01", 500, 500, 1),
    "250": OutputRateMode("250", "02", 250, 250, 1),
    "100": OutputRateMode("100", "03", 100, 100, 1),
    "500_16": OutputRateMode("500_16", "0a", 500, 8000, 16),
}


def _normalize_output_rate(output_rate: int | str) -> str:
    normalized = str(output_rate)
    if normalized not in OUTPUT_RATE_TO_MODE:
        supported_rates = ", ".join(SUPPORTED_OUTPUT_RATES)
        raise ValueError(
            f"Unsupported output_rate {output_rate}. "
            f"Supported output rates are: {supported_rates}"
        )
    return normalized


def _udp_destination_port_to_parameter_value(port: int) -> str:
    if not isinstance(port, int):
        raise ValueError("UDP destination port must be an integer")
    if not MIN_UDP_DESTINATION_PORT <= port <= MAX_UDP_DESTINATION_PORT:
        raise ValueError(
            "UDP destination port must be between "
            f"{MIN_UDP_DESTINATION_PORT} and {MAX_UDP_DESTINATION_PORT}"
        )
    return f"{port:04x}"


@dataclass
class SensorStatus:
    ready: bool
    process_data_invalid: bool
    temperature_out_of_range: bool
    hardware_error: bool
    mech_overrange: bool
    user_overrange: bool

    @classmethod
    def from_bits(cls, bits: int) -> "SensorStatus":
        return cls(
            ready=bool(bits & (1 << 0)),
            process_data_invalid=bool(bits & (1 << 1)),
            temperature_out_of_range=bool(bits & (1 << 2)),
            hardware_error=bool(bits & (1 << 3)),
            mech_overrange=bool(bits & (1 << 4)),
            user_overrange=bool(bits & (1 << 5)),
        )

    def summary(self) -> list[str]:
        """Return human-readable descriptions of active flags."""
        msgs = []
        if not self.ready:
            msgs.append("Not ready for operation")
        if self.process_data_invalid:
            msgs.append("Process data invalid")
        if self.temperature_out_of_range:
            msgs.append("Temperature out of range")
        if self.hardware_error:
            msgs.append("Hardware error")
        if self.mech_overrange:
            msgs.append("Mechanical overrange")
        if self.user_overrange:
            msgs.append("User-defined overrange")
        return msgs


class Driver(object):
    def __init__(
        self,
        host: str = "192.168.0.100",
        port: int = 82,
        streaming_port: int = DEFAULT_STREAMING_PORT,
        output_rate: int | str = 1000,
        streaming_source_host: str | None = None,
    ) -> None:
        output_rate = _normalize_output_rate(output_rate)

        self.host = host
        self.port = port
        self.streaming_port = streaming_port
        self.streaming_source_host = streaming_source_host
        self.output_rate = output_rate
        self.output_rate_mode = OUTPUT_RATE_TO_MODE[output_rate]
        self.output_rate_parameter_value = self.output_rate_mode.parameter_value
        self.connection: Connection = Connection(host=host, port=port)
        self.ft_data: FTDataBuffer = FTDataBuffer()
        self.stream: Stream = Stream(
            port=streaming_port,
            source_host=self.streaming_source_host,
        )
        self.stream_update_thread: Thread = Thread()
        self.is_streaming = False
        self.name = "SCHUNK FTS"  # Placeholder until we can read it from the device
        self.hardware_id = "unknown"
        self.timeout_sec = 0.1
        self.received_data = False
        self.producer_packet_count = 0
        self.last_producer_counter = -1
        self.producer_start_time = time.perf_counter()
        self._lock: Lock = Lock()
        self._streaming_lock: Lock = Lock()
        self.reconnect_interval = 1.0  # seconds between reconnection attempts
        self.auto_reconnect = False

    def streaming_on(
        self, timeout_sec: float = 0.1, auto_reconnect: bool = True
    ) -> bool:
        with self._streaming_lock:
            if self.is_streaming:
                return True
            if not isinstance(timeout_sec, float):
                self.is_streaming = False
                return False
            if not self.connection.open():
                self.is_streaming = False
                return False

            self.is_streaming = True
            self.auto_reconnect = auto_reconnect
            self.stream_update_thread = Thread(
                target=asyncio.run,
                args=(self._update(),),
                daemon=True,
            )
            self.stream_update_thread.start()
            max_duration = time.time() + timeout_sec
            while not self.stream.is_open():
                time.sleep(0.01)
                if time.time() > max_duration:
                    self.is_streaming = False
                    return False
            nameasciistr = self.get_parameter(
                index="0001", subindex="00"
            ).param_value  # Product Name Parameter in ASCII
            self.name = "".join(
                [
                    chr(int(nameasciistr[i : i + 2], 16))
                    for i in range(0, len(nameasciistr), 2)
                ]
            ).strip(
                "\x00"
            )  # Convert hex to ASCII and strip null characters
            self.hardware_id = self.get_parameter(
                index="0001", subindex="03"
            ).param_value  # Product ID Parameter
            if not self._configure_output_rate():
                self.is_streaming = False
                self.auto_reconnect = False
                self.connection.close()
                return False
            if self.set_udp_destination_port(self.streaming_port).error_code != "00":
                self.is_streaming = False
                self.auto_reconnect = False
                self.connection.close()
                return False
            self.start_udp_stream()
            return True

    def streaming_off(self) -> None:
        with self._streaming_lock:
            self.stop_udp_stream()
            self.auto_reconnect = False
            self.is_streaming = False
            if self.stream_update_thread.is_alive():
                self.stream_update_thread.join()
            self.connection.close()

    def sample(self) -> FTData | None:
        if not self.is_streaming:
            return None

        return self.ft_data.get()

    def sample_batch(self) -> list[FTSample] | None:
        if not self.is_streaming:
            return None

        return self.ft_data.get_sample_batch()

    def clear_samples(self) -> None:
        self.ft_data.clear()

    def get_parameter(self, index: str, subindex: str = "00") -> GetParameterResponse:
        req = GetParameterRequest()
        req.command_id = "f0"
        req.param_index = index
        req.param_subindex = subindex
        msg = req.to_bytes()
        data = bytearray()
        with self._lock:
            with self.connection as sensor:
                if sensor:
                    sensor.send(msg)
                    data = sensor.receive()
        response = GetParameterResponse()
        if data:
            response.from_bytes(data)
        return response

    def set_parameter(
        self, value: str, index: str, subindex: str = "00"
    ) -> SetParameterResponse:
        req = SetParameterRequest()
        req.command_id = "f1"
        req.param_index = index
        req.param_subindex = subindex
        req.param_value = value
        msg = req.to_bytes()
        data = bytearray()

        with self.connection as sensor:
            if sensor:
                sensor.send(msg)
                data = sensor.receive()

        response = SetParameterResponse()
        if data:
            response.from_bytes(data)
        return response

    def _configure_output_rate(self) -> bool:
        response = self.set_parameter(
            value=self.output_rate_parameter_value,
            index=OUTPUT_RATE_PARAMETER_INDEX,
            subindex=OUTPUT_RATE_PARAMETER_SUBINDEX,
        )
        return response.error_code == "00"

    def set_udp_destination_port(self, port: int) -> SetParameterResponse:
        return self.set_parameter(
            value=_udp_destination_port_to_parameter_value(port),
            index=UDP_DESTINATION_PORT_PARAMETER_INDEX,
            subindex=UDP_DESTINATION_PORT_PARAMETER_SUBINDEX,
        )

    def get_udp_destination_port(self) -> int | None:
        response = self.get_parameter(
            index=UDP_DESTINATION_PORT_PARAMETER_INDEX,
            subindex=UDP_DESTINATION_PORT_PARAMETER_SUBINDEX,
        )
        if response.error_code != "00":
            return None
        if len(response.param_value) != 4:
            return None
        return int.from_bytes(bytes.fromhex(response.param_value), byteorder="little")

    def run_command(self, command: str) -> CommandResponse:
        req = CommandRequest()
        req.command_id = command
        msg = req.to_bytes()
        response = CommandResponse()

        if self.connection:
            try:
                self.connection.send(msg)
            except Exception:
                return response
            data = self.connection.receive()
            response.from_bytes(data)

        return response

    def start_udp_stream(self) -> CommandResponse:
        return self.run_command("40")

    def stop_udp_stream(self) -> CommandResponse:
        return self.run_command("41")

    def tare(self) -> CommandResponse:
        return self.run_command("12")

    def tare_reset(self) -> CommandResponse:
        return self.run_command("13")

    def select_tool_setting(self, tool_index: int) -> CommandResponse:
        """Select a tool setting (0-3) with Tool Center Point and Overrange Limits.

        Args:
            tool_index: Tool settings index (0-3)

        Returns:
            CommandResponse with error_code "00" on success
        """
        if not 0 <= tool_index <= 3:
            response = CommandResponse()
            response.error_code = "03"  # Invalid Command Value
            return response

        req = CommandWithParameterRequest()
        req.command_id = "30"
        req.parameter = f"{tool_index:02x}"
        msg = req.to_bytes()
        response = CommandResponse()

        if self.connection:
            try:
                self.connection.send(msg)
            except Exception:
                return response
            data = self.connection.receive()
            response.from_bytes(data)

        return response

    def select_noise_filter(self, filter_number: int) -> CommandResponse:
        """Select noise reduction filter using rolling average.

        Args:
            filter_number: Filter number (0-4) for factors 1, 2, 4, 8, 16

        Returns:
            CommandResponse with error_code "00" on success
        """
        if not 0 <= filter_number <= 4:
            response = CommandResponse()
            response.error_code = "03"  # Invalid Command Value
            return response

        req = CommandWithParameterRequest()
        req.command_id = "31"
        req.parameter = f"{filter_number:02x}"
        msg = req.to_bytes()
        response = CommandResponse()

        if self.connection:
            try:
                self.connection.send(msg)
            except Exception:
                return response
            data = self.connection.receive()
            response.from_bytes(data)

        return response

    def get_status(self, status: FTData | None = None) -> SensorStatus | None:
        if not self.is_streaming:
            return None
        status = status or self.sample()
        if status is None:
            return None
        bits = status["status_bits"]
        return SensorStatus.from_bits(bits)

    def _attempt_reconnect(self) -> bool:
        """Attempt to reconnect to the sensor and restart UDP streaming.

        Returns:
            True if reconnection successful, False otherwise
        """
        try:
            # Ensure any old connection is fully closed and socket reset
            try:
                self.connection.close()
            except Exception as e:
                print(f"Error closing connection: {e}")

            # Small delay to allow socket cleanup and sensor to boot
            time.sleep(0.5)

            # Try to open TCP connection
            if not self.connection.open():
                return False

            # Verify connection is actually open and stable
            if not self.connection.is_open:
                return False

            # Give the connection a moment to stabilize
            time.sleep(0.1)

            # Try to start UDP stream with proper error handling
            try:
                with self._lock:
                    if not self._configure_output_rate():
                        print("Failed to configure sensor output rate")
                        self.connection.close()
                        return False

                    if (
                        self.set_udp_destination_port(self.streaming_port).error_code
                        != "00"
                    ):
                        print("Failed to configure sensor UDP destination port")
                        self.connection.close()
                        return False

                    response = self.start_udp_stream()

                    # Check if we got a valid response structure
                    if not hasattr(response, "error_code"):
                        print("Invalid response structure from sensor")
                        self.connection.close()
                        return False

                    # Check if response has valid error code
                    if not response.error_code:
                        print("Empty error code in response")
                        self.connection.close()
                        return False

                    if response.error_code != "00":
                        print(f"Sensor returned error code: {response.error_code}")
                        self.connection.close()
                        return False

            except struct.error as e:
                print(
                    "Failed to parse sensor response "
                    f"(sensor may still be booting): {e}"
                )
                self.connection.close()
                return False
            except Exception as e:
                print(f"Failed to start UDP stream: {e}")
                self.connection.close()
                return False

            # Wait for stream to stabilize
            time.sleep(0.15)

            print("Reconnection successful")
            return True

        except Exception as e:
            print(f"Reconnection attempt failed: {e}")
            try:
                self.connection.close()
            except Exception as e:
                print(f"Error closing connection: {e}")
            return False

    async def _update(self) -> None:
        self.producer_start_time = time.perf_counter()
        with self.stream:
            last_packet_time = time.perf_counter()
            connection_lost = False
            while self.is_streaming or (connection_lost and self.auto_reconnect):
                packets = self.stream.read()
                if not packets:
                    now = time.perf_counter()
                    # Check for timeout regardless of whether we've received data before
                    if now - last_packet_time > self.timeout_sec:
                        if not connection_lost:
                            print("Connection lost - UDP stream timeout")
                            connection_lost = True
                            self.received_data = (
                                False  # Reset to avoid repeated buffer warnings
                            )
                            try:
                                self.connection.close()
                            except Exception as e:
                                print(f"Error closing connection: {e}")

                        # Attempt reconnection if auto_reconnect is enabled
                        if self.auto_reconnect:
                            await asyncio.sleep(self.reconnect_interval)
                            if self._attempt_reconnect():
                                connection_lost = False
                                last_packet_time = time.perf_counter()
                                self.received_data = True
                        else:
                            self.is_streaming = False
                    await asyncio.sleep(0.01)
                    continue

                # If we receive packets after connection was lost, mark as reconnected
                if connection_lost:
                    print("Connection restored - receiving data again")
                    connection_lost = False

                last_packet_time = time.perf_counter()
                self.received_data = True

                # Process every packet in the batch.
                for packet in packets:
                    self.producer_packet_count += 1
                    try:
                        current_counter = struct.unpack_from("<H", packet, 2)[0]
                    except (struct.error, IndexError):
                        continue

                    # Initialize counter on first packet or after reconnection
                    if self.last_producer_counter == -1:
                        self.last_producer_counter = current_counter
                    else:
                        self.last_producer_counter = current_counter

                    # Put the valid packet into our high-speed buffer.
                    self.ft_data.put(packet)

                # Yield control to the event loop.
                await asyncio.sleep(0.0)
