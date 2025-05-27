from schunk_fts_library.utility import Connection


def test_connection_has_expected_fields():
    with Connection(host="0.0.0.0", port=8082) as connection:
        assert connection.host == "0.0.0.0"
        assert connection.port == 8082
        assert connection.socket is not None


def test_connection_succeeds_with_valid_arguments():
    for _ in range(3):
        with Connection(host="0.0.0.0", port=8082) as connection:
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
    connection = Connection(host="0.0.0.0", port=8082)
    with connection:
        pass
    assert connection.socket.fileno() == -1  # means closed

    # Repeated closing
    for _ in range(3):
        with connection:
            pass


def test_connection_supports_bool_checks():

    # Successfully connected
    with Connection(host="0.0.0.0", port=8082) as connection:
        if not connection:
            assert False

    # Not connected
    with Connection(host="10.255.255.1", port=8082) as connection:
        if connection:
            assert False

    # Without context manager
    connection = Connection(host="0.0.0.0", port=8082)
    assert not connection

    # With context manager
    connection = Connection(host="0.0.0.0", port=8082)
    with connection:
        assert connection  # should succeed on first run


def test_connection_creates_new_socket_when_reset():
    connection = Connection(host="0.0.0.0", port=8082)
    before = connection.socket
    connection._reset_socket()
    after = connection.socket
    assert after != before


def test_connection_supports_reusing_the_context_manager():
    connection = Connection(host="0.0.0.0", port=8082)

    for _ in range(5):
        with connection:
            assert connection
