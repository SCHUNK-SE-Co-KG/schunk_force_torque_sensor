# Copyright 2025 SCHUNK SE & Co. KG
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# this program. If not, see <https://www.gnu.org/licenses/>.
# --------------------------------------------------------------------------------
from schunk_fts_library.utility import Connection
import pytest


def test_connection_has_expected_fields(sensor):
    HOST, PORT = sensor
    with Connection(host=HOST, port=PORT) as connection:
        assert connection.host == HOST
        assert connection.port == PORT
        assert connection.socket is not None


def test_connection_succeeds_with_valid_arguments(sensor):
    HOST, PORT = sensor
    for _ in range(3):
        with Connection(host=HOST, port=PORT) as connection:
            assert connection.is_open


@pytest.mark.parametrize(
    "host,port",
    [
        ("10.255.255.1", 8082),
        ("0.1.2.3", 80),
        ("256.256.256.256", 80),
    ],
)
def test_connection_handles_invalid_arguments(host, port):
    with Connection(host=host, port=port) as connection:
        assert not connection.is_open


def test_connection_closes_socket_on_exit(sensor):
    HOST, PORT = sensor
    connection = Connection(host=HOST, port=PORT)
    with connection:
        pass
    assert connection.socket.fileno() == -1  # means closed

    # Repeated closing
    for _ in range(3):
        with connection:
            pass


def test_connection_supports_bool_checks(sensor):
    HOST, PORT = sensor
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


def test_connection_creates_new_socket_when_reset(sensor):
    HOST, PORT = sensor
    connection = Connection(host=HOST, port=PORT)
    before = connection.socket
    connection._reset_socket()
    after = connection.socket
    assert after != before


def test_connection_supports_reusing_the_context_manager(sensor):
    HOST, PORT = sensor
    connection = Connection(host=HOST, port=PORT)

    for _ in range(5):
        with connection:
            assert connection


def test_connection_supports_sending_data(sensor):
    HOST, PORT = sensor
    # When connected
    with Connection(host=HOST, port=PORT) as connection:
        if not connection:
            assert False
        assert connection.send(data=bytearray())

    # Not connected
    connection = Connection(host=HOST, port=PORT)
    assert not connection.send(data=bytearray())


def test_connection_handles_exceptions_when_sending(sensor):
    HOST, PORT = sensor

    # Provoke broken pipe
    connection = Connection(host=HOST, port=PORT)
    connection.is_open = True
    assert not connection.send(data=bytearray())

    # Provoke bad file descriptor
    connection = Connection(host=HOST, port=PORT)
    connection.is_open = True
    connection.socket.close()
    assert not connection.send(data=bytearray())


def test_connection_supports_receiving_data(sensor):
    HOST, PORT = sensor
    # Not connected
    connection = Connection(host=HOST, port=PORT)
    assert not connection.receive()


def test_connection_can_be_opened_and_closed_explicitly(sensor):

    # The `with Connection()` context manager is for quick
    # accesses to the sensor.
    # The `open()` and `close()` calls are for
    # explicitly establishing a lasting connection to the sensor.
    HOST, PORT = sensor
    connection = Connection(host=HOST, port=PORT)
    assert connection.open()

    # It's allowed to open repetitively
    for _ in range(3):
        assert connection.open()

    # It's allowed to close repetitively
    for _ in range(3):
        connection.close()

    # Clean-up
    connection.close()


def test_leaving_context_manager_keeps_previous_connection_open(sensor):

    # This is important to not close a lasting connection
    # that was previously opened by another thread.
    HOST, PORT = sensor
    previous_connection = Connection(host=HOST, port=PORT)
    previous_connection.open()

    # It's safe to use the context manager here
    with previous_connection:
        pass

    assert previous_connection.is_open
    previous_connection.close()

    # Explicit `open()` calls overrule the context
    # manager and need to be closed explicitly
    with previous_connection:
        previous_connection.open()

    assert previous_connection.is_open
    previous_connection.close()
