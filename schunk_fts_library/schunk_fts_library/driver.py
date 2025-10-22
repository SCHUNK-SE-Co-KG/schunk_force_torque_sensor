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
from threading import Thread
import asyncio
import time


class Driver(object):
    def __init__(
        self, host: str = "192.168.0.100", port: int = 82, streaming_port: int = 54843
    ) -> None:
        self.connection: Connection = Connection(host=host, port=port)
        self.ft_data: FTDataBuffer = FTDataBuffer()
        self.stream: Stream = Stream(port=streaming_port)
        self.stream_update_thread: Thread = Thread()
        self.is_streaming = False

    def streaming_on(self, timeout_sec: float = 0.1) -> bool:
        if self.is_streaming:
            return True
        if not isinstance(timeout_sec, float):
            self.is_streaming = False
            return False
        if not self.connection.connect():
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
        self.start_udp_stream()
        return True

    def streaming_off(self) -> None:
        self.stop_udp_stream()
        self.is_streaming = False
        if self.stream_update_thread.is_alive():
            self.stream_update_thread.join()
        self.connection.disconnect()

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

        if self.connection:
            self.connection.send(msg)
            data = self.connection.receive()

        response = GetParameterResponse()
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

        if self.connection:
            self.connection.send(msg)
            data = self.connection.receive()

        response = SetParameterResponse()
        response.from_bytes(data)
        return response

    def run_command(self, command: str) -> CommandResponse:
        req = CommandRequest()
        req.command_id = command
        msg = req.to_bytes()

        if self.connection:
            self.connection.send(msg)
            data = self.connection.receive()

        response = CommandResponse()
        response.from_bytes(data)
        return response

    def start_udp_stream(self) -> CommandResponse:
        return self.run_command("40")

    def stop_udp_stream(self) -> CommandResponse:
        return self.run_command("41")

    async def _update(self) -> None:
        with self.stream:
            while self.is_streaming:
                packet = self.stream.read()
                if len(packet) == len(self.ft_data):
                    values = self.ft_data.decode(packet)
                    self.ft_data.put(values)
