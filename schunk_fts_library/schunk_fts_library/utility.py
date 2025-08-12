import struct
import socket
from socket import socket as Socket
from threading import Lock
from typing import TypedDict

from multiprocessing import Array
import ctypes


class FTData(TypedDict):
    id: int
    status_bits: int
    fx: float
    fy: float
    fz: float
    tx: float
    ty: float
    tz: float


class FTDataBuffer(object):
    def __init__(self) -> None:
        self._data = Array(ctypes.c_double, 8)  # for simplicity
        self._length: int = 29
        self.seq = ctypes.c_uint64(0)

    def __len__(self):
        return self._length

    def put(self, data: FTData) -> None:
        self.seq.value += 1  # make it odd
        self._data[0] = data["id"]
        self._data[1] = data["status_bits"]
        self._data[2] = data["fx"]
        self._data[3] = data["fy"]
        self._data[4] = data["fz"]
        self._data[5] = data["tx"]
        self._data[6] = data["ty"]
        self._data[7] = data["tz"]
        self.seq.value += 1  # make it even

    def get(self) -> FTData:
        while True:
            before = self.seq.value
            if before % 2 != 0:  # put in progress
                continue
            data = FTData(
                id=int(self._data[0]),
                status_bits=int(self._data[1]),
                fx=self._data[2],
                fy=self._data[3],
                fz=self._data[4],
                tx=self._data[5],
                ty=self._data[6],
                tz=self._data[7],
            )
            stop = self.seq.value
            if before == stop:
                return data

    def encode(self, data: FTData) -> bytearray:
        result = bytearray()
        result.extend(bytes(struct.pack("B", data["id"])))
        result.extend(bytes(struct.pack("i", data["status_bits"])))
        result.extend(bytes(struct.pack("f", data["fx"])))
        result.extend(bytes(struct.pack("f", data["fy"])))
        result.extend(bytes(struct.pack("f", data["fz"])))
        result.extend(bytes(struct.pack("f", data["tx"])))
        result.extend(bytes(struct.pack("f", data["ty"])))
        result.extend(bytes(struct.pack("f", data["tz"])))
        return result

    def decode(self, data: bytearray) -> FTData:
        result = FTData(
            id=struct.unpack("B", data[0:1])[0],
            status_bits=struct.unpack("I", data[1:5])[0],
            fx=struct.unpack("f", data[5:9])[0],
            fy=struct.unpack("f", data[9:13])[0],
            fz=struct.unpack("f", data[13:17])[0],
            tx=struct.unpack("f", data[17:21])[0],
            ty=struct.unpack("f", data[21:25])[0],
            tz=struct.unpack("f", data[25:29])[0],
        )
        return result


class Stream(object):
    def __init__(self, port: int) -> None:
        self.port: int = port
        self._lock: Lock = Lock()
        self._is_open: bool = False
        self._reset_socket()

    def is_open(self) -> bool:
        with self._lock:
            return self._is_open

    def read(self) -> bytearray:
        msg = bytearray()
        if self.is_open():
            latest_data = None
            while True:
                try:
                    latest_data, _ = self.socket.recvfrom(1024)
                except BlockingIOError:
                    break
            if latest_data:
                msg.extend(latest_data)
        return msg

    def __enter__(self) -> "Stream":
        if self.port < 1024 or self.port > 65535:
            pass
        else:
            if self.socket.fileno() == -1:  # already closed once
                self._reset_socket()
            try:
                self.socket.bind(("127.0.0.1", self.port))
                with self._lock:
                    self._is_open = True
            except OSError as e:
                print(f"Stream: General socket error: {e}")
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        with self._lock:
            self._is_open = False
        self.socket.close()

    def _reset_socket(self) -> None:
        self.socket: Socket = Socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.setblocking(False)


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
