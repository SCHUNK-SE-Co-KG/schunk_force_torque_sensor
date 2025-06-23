import struct
import socket
from socket import socket as Socket


class Stream(object):
    def __init__(self, port: int) -> None:
        self.port: int = port
        self.is_open: bool = False
        self._reset_socket()

    def read(self) -> bytearray:
        msg = bytearray()
        if self.is_open:
            try:
                data, _ = self.socket.recvfrom(1024)
                msg.extend(data)
            except TimeoutError:
                pass
        return msg

    def __enter__(self) -> "Stream":
        if self.port < 1024 or self.port > 65535:
            pass
        else:
            if self.socket.fileno() == -1:  # already closed once
                self._reset_socket()
            try:
                self.socket.bind(("127.0.0.1", self.port))
                self.is_open = True
            except OSError as e:
                print(f"Stream: General socket error: {e}")
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.is_open = False
        self.socket.close()

    def _reset_socket(self) -> None:
        self.socket: Socket = Socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.settimeout(1.0)


class Connection(object):

    def __init__(self, host: str, port: int) -> None:
        self.host = host
        self.port = port
        self._reset_socket()
        self.is_connected: bool = False

    def send(self, data: bytearray) -> bool:
        if not self.is_connected:
            return False
        try:
            self.socket.sendall(data)
            return True
        except socket.timeout:
            print("Send: Timed out")
        except (BrokenPipeError, ConnectionResetError) as e:
            print(f"Send: Connection error: {e}")
        except OSError as e:
            print(f"Send: General socket error: {e}")
        return False

    def receive(self) -> bytearray:
        data = bytearray()
        if not self.is_connected:
            return data
        data = bytearray(self.socket.recv(1024))
        return data

    def __bool__(self) -> bool:
        return self.is_connected

    def __enter__(self) -> "Connection":
        try:
            if self.socket.fileno() == -1:  # already closed once
                self._reset_socket()
            self.socket.connect((self.host, self.port))
            self.is_connected = True
        except socket.gaierror as e:
            print(f"Connect: Address-related error: {e}")
        except socket.timeout:
            print("Connect: Timed out.")
        except ConnectionRefusedError as e:
            print(f"Connect: Refused by the server: {e}")
        except OSError as e:
            print(f"Connect: General socket error: {e}")
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        try:
            self.socket.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.is_connected = False
        self.socket.close()

    def _reset_socket(self) -> None:
        self.socket: Socket = Socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(2)
        self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)


class Message(object):
    def __init__(self) -> None:
        self.sync: str = "ffff"
        self.counter: int = 0

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        return self.__dict__ == other.__dict__

    def __str__(self):
        output = ""
        for key, value in self.__dict__.items():
            output += f"{key}: {value}, "
        output = output[:-2]
        return output

    def to_bytes(self) -> bytearray:
        payload = bytearray()
        for field in getattr(self, "__fields__", []):
            value = getattr(self, field)
            little_endian_bytes = bytes.fromhex(value)[::-1]
            payload.extend(little_endian_bytes)
        self.payload_len = len(payload)
        data = bytearray(
            bytes.fromhex(self.sync)
            + struct.pack("<H", self.counter)
            + struct.pack("<H", self.payload_len)
            + payload
        )
        return data


class GetParameterRequest(Message):
    __fields__ = ["command_id", "param_index", "param_subindex"]

    def __init__(self) -> None:
        super().__init__()
        for field in self.__fields__:
            setattr(self, field, "")

    def from_bytes(self, data: bytearray) -> bool:
        self.sync = data[0:2].hex()
        self.counter = struct.unpack("H", data[2:4])[0]
        self.payload_len = struct.unpack("H", data[4:6])[0]
        self.command_id = data[6:7].hex()
        self.param_index = data[7:9][::-1].hex()
        self.param_subindex = data[9:10].hex()
        return True


class GetParameterResponse(Message):
    __fields__ = [
        "command_id",
        "error_code",
        "param_index",
        "param_subindex",
        "param_value",
    ]

    def __init__(self) -> None:
        super().__init__()
        for field in self.__fields__:
            setattr(self, field, "")

    def from_bytes(self, data: bytearray) -> bool:
        self.sync = data[0:2].hex()
        self.counter = struct.unpack("H", data[2:4])[0]
        self.payload_len = struct.unpack("H", data[4:6])[0]
        self.command_id = data[6:7].hex()
        self.error_code = data[7:8].hex()
        self.param_index = data[8:10][::-1].hex()
        self.param_subindex = data[10:11].hex()
        self.param_value = data[11:].hex()
        return True


class SetParameterRequest(Message):
    __fields__ = ["command_id", "param_index", "param_subindex", "param_value"]

    def __init__(self) -> None:
        super().__init__()
        for field in self.__fields__:
            setattr(self, field, "")

    def from_bytes(self, data: bytearray) -> bool:
        self.sync = data[0:2].hex()
        self.counter = struct.unpack("H", data[2:4])[0]
        self.payload_len = struct.unpack("H", data[4:6])[0]
        self.command_id = data[6:7].hex()
        self.param_index = data[7:9][::-1].hex()
        self.param_subindex = data[9:10].hex()
        self.param_value = data[10:].hex()
        return True


class SetParameterResponse(Message):
    __fields__ = ["command_id", "error_code", "param_index", "param_subindex"]

    def __init__(self) -> None:
        super().__init__()
        for field in self.__fields__:
            setattr(self, field, "")

    def from_bytes(self, data: bytearray) -> bool:
        self.sync = data[0:2].hex()
        self.counter = struct.unpack("H", data[2:4])[0]
        self.payload_len = struct.unpack("H", data[4:6])[0]
        self.command_id = data[6:7].hex()
        self.error_code = data[7:8].hex()
        self.param_index = data[8:10][::-1].hex()
        self.param_subindex = data[10:11].hex()
        return True


class CommandRequest(Message):
    __fields__ = ["command_id"]

    def __init__(self) -> None:
        super().__init__()
        for field in self.__fields__:
            setattr(self, field, "")

    def from_bytes(self, data: bytearray) -> bool:
        self.sync = data[0:2].hex()
        self.counter = struct.unpack("H", data[2:4])[0]
        self.payload_len = struct.unpack("H", data[4:6])[0]
        self.command_id = data[6:7].hex()
        return True


class CommandResponse(Message):
    __fields__ = ["command_id", "error_code"]

    def __init__(self) -> None:
        super().__init__()
        for field in self.__fields__:
            setattr(self, field, "")

    def from_bytes(self, data: bytearray) -> bool:
        self.sync = data[0:2].hex()
        self.counter = struct.unpack("H", data[2:4])[0]
        self.payload_len = struct.unpack("H", data[4:6])[0]
        self.command_id = data[6:7].hex()
        self.error_code = data[7:8].hex()
        return True
