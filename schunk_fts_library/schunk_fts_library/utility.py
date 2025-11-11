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
import struct
import socket
from socket import socket as Socket
from threading import Lock, Event
from typing import TypedDict
import time
from queue import Queue, Empty, Full


class FTData(TypedDict):
    sync: bytes
    counter: int
    payload: int
    id: int
    status_bits: int
    fx: float
    fy: float
    fz: float
    tx: float
    ty: float
    tz: float


class FTDataBuffer(object):
    def __init__(self, maxsize=100) -> None:
        self._queue: Queue[FTData] = Queue(maxsize=maxsize)
        self._last_packet_time = time.monotonic()

    def __len__(self):
        return self._queue.qsize()

    def put(self, packet: bytearray) -> None:
        try:
            self._queue.put_nowait(packet)
        except Full:
            pass

    def get(self) -> FTData | None:
        try:
            packet = self._queue.get(timeout=0.1)
            self._last_packet_time = time.monotonic()
            return self.decode(packet)
        except Empty:
            if (
                time.monotonic() - self._last_packet_time > 0.2
            ):  # return empty FTData to indicate no signal
                print("FTDataBuffer: No data received anymore")
                return FTData(
                    sync=b"",
                    counter=0,
                    payload=0,
                    id=0,
                    status_bits=0,
                    fx=0.0,
                    fy=0.0,
                    fz=0.0,
                    tx=0.0,
                    ty=0.0,
                    tz=0.0,
                )
            return None

    @staticmethod
    def decode(data: bytearray) -> FTData:
        values = struct.unpack("<HHB I ffffff", data[2:])  # skip sync (first 2 bytes)
        return FTData(
            sync=data[0:2],
            counter=values[0],
            payload=values[1],
            id=values[2],
            status_bits=values[3],
            fx=values[4],
            fy=values[5],
            fz=values[6],
            tx=values[7],
            ty=values[8],
            tz=values[9],
        )


class Stream(object):
    def __init__(self, port: int) -> None:
        self.port: int = port
        self._lock: Lock = Lock()
        self._is_open: bool = False
        self._reset_socket()

    def is_open(self) -> bool:
        with self._lock:
            return self._is_open

    def read(self) -> list[bytes]:
        packets = []
        if self.is_open():
            while True:
                try:
                    data, _ = self.socket.recvfrom(1024)
                    packets.append(data)
                except BlockingIOError:
                    break
                except Exception:
                    break
        return packets

    def __enter__(self) -> "Stream":
        if self.port < 1024 or self.port > 65535:
            pass
        else:
            if self.socket.fileno() == -1:  # already closed once
                self._reset_socket()
            try:
                self.socket.bind(("", self.port))
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
        self.is_open: bool = False
        self.persistent: Event = Event()

    def send(self, data: bytearray) -> bool:
        if not self.is_open:
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
        if not self.is_open:
            return data
        data = bytearray(self.socket.recv(1024))
        return data

    def __bool__(self) -> bool:
        return self.is_open

    def open(self) -> bool:
        self.__enter__()
        if self.is_open:
            self.persistent.set()
            return True
        return False

    def close(self) -> None:
        self.__exit__(None, None, None)
        self.persistent.clear()

    def __enter__(self) -> "Connection":
        if self.is_open:
            return self
        try:
            if self.socket.fileno() == -1:  # already closed once
                self._reset_socket()
            self.socket.connect((self.host, self.port))
            self.is_open = True
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
        if self.persistent.is_set():
            return
        if self.is_open:
            try:
                self.socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self.is_open = False
            self.persistent.clear()
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
            try:
                value = getattr(self, field)
                little_endian_bytes = bytes.fromhex(value)[::-1]
                payload.extend(little_endian_bytes)
            except Exception as e:
                print(f"to_bytes: Error processing field '{field}': {e}")
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
