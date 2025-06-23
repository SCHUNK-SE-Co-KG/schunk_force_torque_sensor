from schunk_fts_library.utility import Stream
import os

PORT = int(os.getenv("FTS_STREAMING_PORT", 54843))


def test_stream_has_expected_fields():
    with Stream(port=PORT) as stream:
        assert stream.port == PORT
        assert stream.socket is not None


def test_stream_opens_with_valid_ports():
    valid_ports = [54843, 54001, 54002]
    for port in valid_ports:
        with Stream(port=port) as stream:
            assert stream.is_open


def test_stream_rejects_invalid_ports():
    invalid_ports = [0, -1, 80, 1023, 65535 + 1]
    for port in invalid_ports:
        with Stream(port=port) as stream:
            assert not stream.is_open


def test_stream_closes_socket_on_exit():
    stream = Stream(port=PORT)
    with stream:
        pass
    assert stream.socket.fileno() == -1  # means closed
    assert not stream.is_open

    # Repeated closing
    for _ in range(3):
        with stream:
            pass


def test_stream_creates_new_socket_when_reset():
    stream = Stream(port=PORT)
    before = stream.socket
    stream._reset_socket()
    after = stream.socket
    assert after != before


def test_stream_can_be_reused():
    stream = Stream(port=PORT)
    for _ in range(5):
        with stream:
            assert stream.is_open
