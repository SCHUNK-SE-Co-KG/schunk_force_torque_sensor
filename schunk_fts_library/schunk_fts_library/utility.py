import struct


class Message(object):
    def __init__(self) -> None:
        self.sync: str = "ffff"
        self.counter: int = 0
        self.payload_len: int = 0
        self.command_id: str = ""
        self.error_code: str = ""
        self.param_index: str = ""
        self.param_subindex: str = ""
        self.param_value: bytearray = bytearray()

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


class GetParameterRequest(Message):
    def to_bytes(self) -> bytearray:
        payload = (
            bytes.fromhex(self.command_id)
            + bytes.fromhex(self.param_index)
            + bytes.fromhex(self.param_subindex)
        )
        self.payload_len = len(payload)
        data = bytearray(
            bytes.fromhex(self.sync)
            + struct.pack("<H", self.counter)
            + struct.pack("<H", self.payload_len)
            + payload
        )
        return data

    def from_bytes(self, data: bytearray) -> bool:
        self.sync = data[0:2].hex()
        self.counter = struct.unpack("H", data[2:4])[0]
        self.payload_len = struct.unpack("H", data[4:6])[0]
        self.command_id = data[6:7].hex()
        self.param_index = data[7:9].hex()
        self.param_subindex = data[9:10].hex()
        return True


class GetParameterResponse(Message):
    def to_bytes(self) -> bytearray:
        payload = (
            bytes.fromhex(self.command_id)
            + bytes.fromhex(self.error_code)
            + bytes.fromhex(self.param_index)
            + bytes.fromhex(self.param_subindex)
            + self.param_value
        )
        self.payload_len = len(payload)
        data = bytearray(
            bytes.fromhex(self.sync)
            + struct.pack("<H", self.counter)
            + struct.pack("<H", self.payload_len)
            + payload
        )
        return data

    def from_bytes(self, data: bytearray) -> bool:
        self.sync = data[0:2].hex()
        self.counter = struct.unpack("H", data[2:4])[0]
        self.payload_len = struct.unpack("H", data[4:6])[0]
        self.command_id = data[6:7].hex()
        self.error_code = data[7:8].hex()
        self.param_index = data[8:10].hex()
        self.param_subindex = data[10:11].hex()
        self.param_value = data[11:]
        return True


class SetParameterRequest(Message):
    def to_bytes(self) -> bytearray:
        payload = (
            bytes.fromhex(self.command_id)
            + bytes.fromhex(self.param_index)
            + bytes.fromhex(self.param_subindex)
            + self.param_value
        )
        self.payload_len = len(payload)
        data = bytearray(
            bytes.fromhex(self.sync)
            + struct.pack("<H", self.counter)
            + struct.pack("<H", self.payload_len)
            + payload
        )
        return data

    def from_bytes(self, data: bytearray) -> bool:
        self.sync = data[0:2].hex()
        self.counter = struct.unpack("H", data[2:4])[0]
        self.payload_len = struct.unpack("H", data[4:6])[0]
        self.command_id = data[6:7].hex()
        self.param_index = data[7:9].hex()
        self.param_subindex = data[9:10].hex()
        self.param_value = data[10:]
        return True


class SetParameterResponse(Message):
    def to_bytes(self) -> bytearray:
        payload = (
            bytes.fromhex(self.command_id)
            + bytes.fromhex(self.error_code)
            + bytes.fromhex(self.param_index)
            + bytes.fromhex(self.param_subindex)
        )
        self.payload_len = len(payload)
        data = bytearray(
            bytes.fromhex(self.sync)
            + struct.pack("<H", self.counter)
            + struct.pack("<H", self.payload_len)
            + payload
        )
        return data

    def from_bytes(self, data: bytearray) -> bool:
        self.sync = data[0:2].hex()
        self.counter = struct.unpack("H", data[2:4])[0]
        self.payload_len = struct.unpack("H", data[4:6])[0]
        self.command_id = data[6:7].hex()
        self.error_code = data[7:8].hex()
        self.param_index = data[8:10].hex()
        self.param_subindex = data[10:11].hex()
        return True
