from schunk_fts_library.utility import (
    Message,
    SetParameterRequest,
    SetParameterResponse,
    GetParameterRequest,
    GetParameterResponse,
)


def test_parameter_message_has_expected_fields():
    message = Message()
    assert message.sync == "ffff"
    assert message.counter == 0
    assert message.payload_len == 0
    assert message.command_id == ""
    assert message.error_code == ""
    assert message.param_index == ""
    assert message.param_subindex == ""
    assert message.param_value == bytearray()


def test_message_offers_str_method():
    msg = Message()

    # Check if all variables occur in the output
    output = str(msg)
    for entry in msg.__dict__.keys():
        assert entry in output


def test_get_parameter_message():

    # Request
    msg = GetParameterRequest()
    msg.command_id = "f0"
    msg.param_index = "1234"
    msg.param_subindex = "02"

    data = msg.to_bytes()
    other = GetParameterRequest()
    other.from_bytes(data)

    assert msg == other

    # Response
    msg = GetParameterResponse()
    msg.command_id = "f0"
    msg.error_code = "01"
    msg.param_index = "1234"
    msg.param_subindex = "02"
    msg.param_value = bytearray(bytes.fromhex("42"))

    data = msg.to_bytes()
    other = GetParameterResponse()
    other.from_bytes(data)
    print(f"msg: {msg}")
    print(f"other: {other}")
    assert msg == other


def test_set_parameter_message():

    # Request
    msg = SetParameterRequest()
    msg.command_id = "f1"
    msg.param_index = "1234"
    msg.param_subindex = "02"
    msg.param_value = bytearray(bytes.fromhex("42"))

    data = msg.to_bytes()
    other = SetParameterRequest()
    other.from_bytes(data)

    print(f"msg: {msg}")
    print(f"other: {other}")
    assert msg == other

    # Response
    msg = SetParameterResponse()
    msg.command_id = "f1"
    msg.error_code = "01"
    msg.param_index = "1234"
    msg.param_subindex = "02"

    data = msg.to_bytes()
    other = SetParameterResponse()
    other.from_bytes(data)
    assert msg == other
