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
    FTDataBuffer,
    SetParameterRequest,
    SetParameterResponse,
    GetParameterRequest,
    GetParameterResponse,
    CommandRequest,
    CommandResponse,
)
from threading import Thread, Lock
import asyncio
import time
from dataclasses import dataclass
import struct


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
        self, host: str = "192.168.0.100", port: int = 82, streaming_port: int = 54843
    ) -> None:
        self.connection: Connection = Connection(host=host, port=port)
        self.ft_data: FTDataBuffer = FTDataBuffer()
        self.stream: Stream = Stream(port=streaming_port)
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

    def streaming_on(self, timeout_sec: float = 0.1) -> bool:
        if self.is_streaming:
            return True
        if not isinstance(timeout_sec, float):
            self.is_streaming = False
            return False
        if not self.connection.open():
            self.is_streaming = False
            return False

        self.is_streaming = True
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
        self.start_udp_stream()
        return True

    def streaming_off(self) -> None:
        self.stop_udp_stream()
        self.is_streaming = False
        if self.stream_update_thread.is_alive():
            self.stream_update_thread.join()
        self.connection.close()

    def sample(self) -> FTData | None:
        if not self.is_streaming:
            return None

        return self.ft_data.get()

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

    def get_status(self, status: FTData | None = None) -> SensorStatus | None:
        if not self.is_streaming:
            return None
        status = status or self.sample()
        if status is None:
            return None
        bits = status["status_bits"]
        return SensorStatus.from_bits(bits)

    async def _update(self) -> None:
        self.producer_start_time = time.perf_counter()
        with self.stream:
            last_packet_time = time.perf_counter()
            while self.is_streaming:
                packets = self.stream.read()
                if not packets:
                    now = time.perf_counter()
                    if self.received_data and (
                        now - last_packet_time > self.timeout_sec
                    ):
                        self.is_streaming = False
                    continue

                # Process every packet in the batch.
                for packet in packets:
                    self.producer_packet_count += 1
                    try:
                        current_counter = struct.unpack_from("<H", packet, 2)[0]
                    except (struct.error, IndexError):
                        continue

                    self.last_producer_counter = current_counter

                    # Put the valid packet into our high-speed buffer.
                    self.ft_data.put(packet)

                # Yield control to the event loop.
                await asyncio.sleep(0.0)
