from .utility import (
    SetParameterRequest,
    SetParameterResponse,
    GetParameterRequest,
    GetParameterResponse,
)
import socket
from socket import socket as Socket


class Driver(object):
    def __init__(self) -> None:
        self.socket: Socket = Socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(1)
        self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.host: str = "192.168.0.100"
        self.port: int = 82
        self.connected: bool = False

    def connect(self, host: str, port: int) -> bool:
        if self.connected:
            if host != self.host or port != self.port:
                return False
            return True
        result = self.socket.connect_ex((host, port))
        if result != 0:
            return False
        self.connected = True
        return True

    def disconnect(self) -> bool:
        self.socket.close()
        return True

    def get_parameter(self, index: str, subindex: str = "00") -> GetParameterResponse:
        req = GetParameterRequest()
        req.command_id = "f0"
        req.param_index = index
        req.param_subindex = subindex
        msg = req.to_bytes()
        self.socket.sendall(msg)
        data = bytearray(self.socket.recv(1024))
        response = GetParameterResponse()
        response.from_bytes(data)
        return response

    def set_parameter(
        self, value: bytearray, index: str, subindex: str = "00"
    ) -> SetParameterResponse:
        req = SetParameterRequest()
        req.command_id = "f1"
        req.param_index = index
        req.param_subindex = subindex
        req.param_value = value
        msg = req.to_bytes()
        self.socket.sendall(msg)
        data = bytearray(self.socket.recv(1024))
        response = SetParameterResponse()
        response.from_bytes(data)
        return response
