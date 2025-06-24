from .utility import (
    Connection,
    Stream,
    SetParameterRequest,
    SetParameterResponse,
    GetParameterRequest,
    GetParameterResponse,
    CommandRequest,
    CommandResponse,
)
from threading import Thread
import asyncio


class Driver(object):
    def __init__(self, host: str = "192.168.0.100", port: int = 82) -> None:
        self.connection: Connection = Connection(host=host, port=port)
        self.stream: Stream = Stream(port=54843)
        self.update_thread: Thread = Thread()
        self.is_streaming = False

    def streaming_on(self) -> None:
        self.is_streaming = True
        self.update_thread = Thread(
            target=asyncio.run,
            args=(self._update(),),
            daemon=True,
        )
        self.update_thread.start()

    def streaming_off(self) -> None:
        self.is_streaming = False

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
                await asyncio.sleep(0.01)
