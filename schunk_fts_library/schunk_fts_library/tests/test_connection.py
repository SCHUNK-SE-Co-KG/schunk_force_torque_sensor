from schunk_fts_library.utility import Connection


def test_connection_has_expected_fields():
    with Connection(host="0.0.0.0", port=8082) as connection:
        assert connection.host == "0.0.0.0"
        assert connection.port == 8082
        assert connection.socket is not None


def test_connection_succeeds_with_valid_arguments():
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

    # Repeated closing will call __exit__ again
    with connection:
        pass
