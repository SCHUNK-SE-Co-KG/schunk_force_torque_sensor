from .utility import (
    Connection,
    SetParameterRequest,
    SetParameterResponse,
    GetParameterRequest,
    GetParameterResponse,
    CommandRequest,
    CommandResponse,
)


class Driver(object):
    def __init__(self) -> None:
        self.host: str = "192.168.0.100"
        self.port: int = 82
        self.connection: Connection = Connection(self.host, self.port)

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
