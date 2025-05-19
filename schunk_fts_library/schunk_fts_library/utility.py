import struct


class Response(object):
    def __init__(self) -> None:
        self.sync: str = "ffff"
        self.counter: int | None = None
        self.payload_len: int | None = None
        self.command_id: str = ""
        self.error_code: str = ""
        self.param_index: str = ""
        self.param_subindex: str = ""
        self.param_value: bytearray = bytearray()

    def __str__(self):
        return (
            f"sync: {self.sync}, "
            f"counter: {self.counter}, "
            f"payload_len: {self.payload_len}, "
            f"command_id: {self.command_id}, "
            f"error_code: {self.error_code}, "
            f"param_index: {self.param_index}, "
            f"param_subindex: {self.param_subindex}, "
            f"param_value: {self.param_value}"
        )

    def from_bytes(self, data: bytearray) -> bool:
        self.counter = struct.unpack("H", data[2:4])[0]
        self.payload_len = struct.unpack("H", data[4:6])[0]
        self.command_id = data[6:7].hex()
        self.error_code = data[7:8].hex()
        self.param_index = data[8:10].hex()
        self.param_subindex = data[10:11].hex()
        self.param_value = data[11:]
        return True

    def to_bytes(self) -> bytearray:
        payload = (
            bytes.fromhex(self.command_id)
            + struct.pack("<B", int(self.error_code, 16))
            + bytes.fromhex(self.param_index)
            + bytes.fromhex(self.param_subindex)
            + self.param_value
        )
        self.payload_len = len(payload)
        data = bytearray(
            struct.pack("<HHH", int(self.sync, 16), self.counter, self.payload_len)
            + payload
        )
        return data
