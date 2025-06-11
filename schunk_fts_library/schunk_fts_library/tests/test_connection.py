from schunk_fts_library.utility import Connection
import os

HOST = os.getenv("FTS_HOST", "192.168.0.100")
PORT = int(os.getenv("FTS_PORT", 82))


def test_connection_has_expected_fields():
    with Connection(host=HOST, port=PORT) as connection:
        assert connection.host == HOST
        assert connection.port == PORT
        assert connection.socket is not None


def test_connection_succeeds_with_valid_arguments(fts_dummy):
    for _ in range(3):
        with Connection(host=HOST, port=PORT) as connection:
            assert connection.is_connected


def test_connection_handles_invalid_arguments():
    invalid_args = [
        {"host": "10.255.255.1", "port": 8082},
        {"host": "0.1.2.3", "port": 80},
        {"host": "0.0.0.0", "port": 80},
    ]
    for arg in invalid_args:
        with Connection(host=arg["host"], port=arg["port"]) as connection:
            assert not connection.is_connected


def test_connection_closes_socket_on_exit():
    connection = Connection(host=HOST, port=PORT)
    with connection:
        pass
    assert connection.socket.fileno() == -1  # means closed

    # Repeated closing
    for _ in range(3):
        with connection:
            pass


def test_connection_supports_bool_checks(fts_dummy):

    # Successfully connected
    with Connection(host=HOST, port=PORT) as connection:
        if not connection:
            assert False

    # Not connected
    with Connection(host="10.255.255.1", port=PORT) as connection:
        if connection:
            assert False

    # Without context manager
    connection = Connection(host=HOST, port=PORT)
    assert not connection

    # With context manager
    connection = Connection(host=HOST, port=PORT)
    with connection:
        assert connection


def test_connection_creates_new_socket_when_reset():
    connection = Connection(host=HOST, port=PORT)
    before = connection.socket
    connection._reset_socket()
    after = connection.socket
    assert after != before


def test_connection_supports_reusing_the_context_manager(fts_dummy):
    connection = Connection(host=HOST, port=PORT)

    for _ in range(5):
        with connection:
            assert connection


def test_connection_supports_sending_data(fts_dummy):

    # When connected
    with Connection(host=HOST, port=PORT) as connection:
        if not connection:
            assert False
        assert connection.send(data=bytearray())

    # Not connected
    connection = Connection(host=HOST, port=PORT)
    assert not connection.send(data=bytearray())


def test_connection_handles_exceptions_when_sending():

    # Provoke broken pipe
    connection = Connection(host=HOST, port=PORT)
    connection.is_connected = True
    assert not connection.send(data=bytearray())

    # Provoke bad file descriptor
    connection = Connection(host=HOST, port=PORT)
    connection.is_connected = True
    connection.socket.close()
    assert not connection.send(data=bytearray())


def test_connection_supports_receiving_data():

    # Not connected
    connection = Connection(host=HOST, port=PORT)
    assert not connection.receive()
