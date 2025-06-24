from schunk_fts_library.utility import Stream
import struct
import os
import time


PORT = int(os.getenv("FTS_STREAMING_PORT", 54843))


def encode(data: dict) -> bytearray:
    message = bytearray()
    message.extend(bytes(struct.pack("B", data["packet_id"])))
    message.extend(bytes(struct.pack("i", data["status_bits"])))
    message.extend(bytes(struct.pack("f", data["fx"])))
    message.extend(bytes(struct.pack("f", data["fy"])))
    message.extend(bytes(struct.pack("f", data["fz"])))
    message.extend(bytes(struct.pack("f", data["tx"])))
    message.extend(bytes(struct.pack("f", data["ty"])))
    message.extend(bytes(struct.pack("f", data["tz"])))
    return message


def test_stream_has_expected_fields():
    with Stream(port=PORT) as stream:
        assert stream.port == PORT
        assert stream.socket is not None


def test_stream_opens_with_valid_ports():
    valid_ports = [54843, 54001, 54002]
    for port in valid_ports:
        with Stream(port=port) as stream:
            assert stream.is_open, port


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


def test_stream_supports_reading_data(send_messages):
    data = {
        "packet_id": 1,
        "status_bits": 0x00000000,
        "fx": 1.0,
        "fy": 2.1,
        "fz": 3.3,
        "tx": 0.04,
        "ty": -17.358,
        "tz": 23.001,
    }
    msg = encode(data=data)

    # Empty return when not open
    stream = Stream(port=8001)
    assert stream.read() == bytearray()

    # Succeeds when open.
    # Make sure to send data after we bind to the port to not miss it.
    with stream:
        send_messages(8001, [msg])
        time.sleep(0.1)
        assert stream.read() == msg


def test_stream_returns_only_most_recent_data(send_messages):
    messages = []
    for id in [1, 2, 3, 4]:
        messages.append(
            encode(
                {
                    "packet_id": id,
                    "status_bits": 0x00000000,
                    "fx": 1.0,
                    "fy": 2.0,
                    "fz": 3.0,
                    "tx": 4.0,
                    "ty": 5.0,
                    "tz": 6.0,
                }
            )
        )

    with Stream(port=8001) as stream:
        send_messages(8001, messages)
        time.sleep(0.1)
        assert stream.read() == messages[-1]


def test_stream_returns_immediately_without_data():
    with Stream(port=8001) as stream:
        start = time.perf_counter()
        assert stream.read() == bytearray()
        stop = time.perf_counter()
        elapsed = (stop - start) * 1000  # ms
        assert elapsed < 0.1
