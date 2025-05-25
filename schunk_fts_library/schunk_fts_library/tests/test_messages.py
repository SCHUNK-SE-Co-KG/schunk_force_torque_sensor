from schunk_fts_library.utility import (
    Message,
    GetParameterRequest,
    GetParameterResponse,
    SetParameterRequest,
    SetParameterResponse,
    CommandRequest,
    CommandResponse,
)


def test_message_has_expected_fields():
    message = Message()
    assert message.sync == "ffff"
    assert message.counter == 0


def test_messages_are_printable():

    # Check if all variables occur in the output
    msg = GetParameterRequest()
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
    msg.param_value = "45"

    data = msg.to_bytes()
    other = GetParameterResponse()
    other.from_bytes(data)
    assert msg == other


def test_set_parameter_message():

    # Request
    msg = SetParameterRequest()
    msg.command_id = "f1"
    msg.param_index = "1234"
    msg.param_subindex = "02"
    msg.param_value = "42"

    data = msg.to_bytes()
    other = SetParameterRequest()
    other.from_bytes(data)
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


def test_command_message():

    # Request
    msg = CommandRequest()
    msg.command_id = "10"
    data = msg.to_bytes()
    other = CommandRequest()
    other.from_bytes(data)
    assert msg == other

    # Response
    msg = CommandResponse()
    msg.command_id = "10"
    msg.error_code = "05"
    data = msg.to_bytes()
    other = CommandResponse()
    other.from_bytes(data)
    assert msg == other
