import threading
import pytest
import socket
from socket import socket as Socket


@pytest.fixture
def send_messages():
    def _send_to(port, messages):
        def sender():
            with Socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                for msg in messages:
                    s.sendto(msg, ("127.0.0.1", port))

        threading.Thread(target=sender).start()

    return _send_to
