from .utility import (
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
        self.update_thread: Thread = Thread()
        self.is_streaming = False

    def streaming_on(self, timeout_sec: float = 0.1) -> bool:
        if self.is_streaming:
            return True
        if not isinstance(timeout_sec, float):
            return False
        self.is_streaming = True
        self.update_thread = Thread(
            target=asyncio.run,
            args=(self._update(),),
            daemon=True,
        )
        self.update_thread.start()
        max_duration = time.time() + timeout_sec
        while not self.stream.is_open():
            time.sleep(0.01)
            if time.time() > max_duration:
                self.is_streaming = False
                return False
        return True

    def streaming_off(self) -> None:
        self.is_streaming = False
        while self.update_thread.is_alive():
            time.sleep(0.01)

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

        with self.connection as sensor:
            if sensor:
                sensor.send(msg)
                data = sensor.receive()

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

        with self.connection as sensor:
            if sensor:
                sensor.send(msg)
                data = sensor.receive()

        response = SetParameterResponse()
        response.from_bytes(data)
        return response

    def run_command(self, command: str) -> CommandResponse:
        req = CommandRequest()
        req.command_id = command
        msg = req.to_bytes()

        with self.connection as sensor:
            if sensor:
                sensor.send(msg)
                data = sensor.receive()

        response = CommandResponse()
        response.from_bytes(data)
        return response

    async def _update(self) -> None:
        with self.stream:
            while self.is_streaming:
                packet = self.stream.read()
                if len(packet) == len(self.ft_data):
                    values = self.ft_data.decode(packet)
                    self.ft_data.put(values)
