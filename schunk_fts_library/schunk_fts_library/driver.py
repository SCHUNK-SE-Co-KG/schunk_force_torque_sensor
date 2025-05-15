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
