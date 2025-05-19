from schunk_fts_library.utility import Response


def test_response_message_has_expected_fields():
    response = Response()
    assert response.sync == "ffff"
    assert response.counter is None
    assert response.payload_len is None
    assert response.command_id == ""
    assert response.error_code == ""
    assert response.param_index == ""
    assert response.param_subindex == ""
    assert response.param_value == bytearray()


def test_response_message_offers_bytes_conversion():

    response = Response()
    response.sync = "ffff"
    response.counter = 42
    response.command_id = "f0"
    response.error_code = "55"
    response.param_index = "1234"
    response.param_subindex = "02"
    response.param_value = bytes.fromhex("01")

    data = response.to_bytes()
    other = Response()
    assert other.from_bytes(data)

    assert other.sync == response.sync
    assert other.counter == response.counter
    assert other.command_id == response.command_id
    assert other.error_code == response.error_code
    assert other.param_index == response.param_index
    assert other.param_subindex == response.param_subindex
    assert other.param_value == response.param_value


def test_response_message_offers_str_method():
    response = Response()
    response.sync = "ffff"
    response.counter = 42
    response.error_code = "55"
    response.param_index = "1234"
    response.param_subindex = "02"
    response.param_value = bytes.fromhex("01")

    # Check if all variables occur in the output
    output = str(response)
    for entry in response.__dict__.keys():
        assert entry in output
