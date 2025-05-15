import socket
from socket import socket as Socket
import struct


class Driver(object):
    def __init__(self) -> None:
        self.socket: Socket = Socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(1)
        self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.host: str = "192.168.0.100"
        self.port: int = 82
        self.connected: bool = False
        self.sync_bytes: int = int("FFFF", 16)

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

    def get_parameter(self, index: str, subindex: str = "00") -> bytearray:
        cmd_id = int("F0", 16)
        payload = struct.pack("<BHB", cmd_id, int(index, 16), int(subindex, 16))
        packet_counter = 1
        message = (
            struct.pack("<HHH", self.sync_bytes, packet_counter, len(payload)) + payload
        )
        self.socket.sendall(message)
        response = bytearray(self.socket.recv(1024))
        return response
