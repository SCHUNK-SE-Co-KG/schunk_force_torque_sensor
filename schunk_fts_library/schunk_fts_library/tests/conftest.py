import pytest
from schunk_fts_library.utility import Connection
from pathlib import Path
import subprocess
import os


def env_variables_set() -> bool:
    return os.getenv("FTS_HOST") is not None and os.getenv("FTS_PORT") is not None


@pytest.fixture(scope="module")
def sensor():

    sensor_available = False

    # Real hardware
    connection = Connection(host="192.168.0.100", port=82)
    connection.socket.settimeout(0.5)
    with connection:
        if connection:
            sensor_available = True

    # Simulated hardware
    connection = Connection(host="127.0.0.1", port=8082)
    connection.socket.settimeout(0.5)
    with connection:
        if connection and env_variables_set():
            sensor_available = True

    # CI setting
    ci_dummy = Path("/tmp/schunk_fts_dummy/debug/schunk_fts_dummy")
    if ci_dummy.exists():
        process = subprocess.Popen(
            [ci_dummy],
            stderr=subprocess.PIPE,
            text=True,
        )
        connection = Connection(host="127.0.0.1", port=8082)
        connection.socket.settimeout(5.0)
        with connection:
            if connection and env_variables_set():
                sensor_available = True

    if not sensor_available:
        pytest.skip("Sensor not reachable.")

    yield

    # Cleanup
    if ci_dummy.exists():
        process.kill()
