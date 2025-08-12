import pytest
from pathlib import Path
import subprocess
import os
import time
import socket


def sensor_available_at(host: str, port: int, timeout_sec=2.0) -> bool:
    start = time.time()
    while time.time() - start < timeout_sec:
        try:
            with socket.create_connection((host, port), timeout=0.1):
                return True
        except (ConnectionRefusedError, socket.timeout):
            time.sleep(0.1)
    return False


@pytest.fixture(scope="module")
def sensor():

    sensor_available = False
    ci_dummy = Path("/tmp/schunk_fts_dummy/debug/schunk_fts_dummy")
    process = None

    def env_variables_set() -> bool:
        return os.getenv("FTS_HOST") is not None and os.getenv("FTS_PORT") is not None

    # Real hardware
    if sensor_available_at(host="192.168.0.100", port=82):
        sensor_available = True

    # Simulated hardware
    elif sensor_available_at(host="127.0.0.1", port=8082) and env_variables_set():
        sensor_available = True

    # CI setting
    elif ci_dummy.exists():
        process = subprocess.Popen(
            [ci_dummy],
            stderr=subprocess.PIPE,
            text=True,
        )
        if sensor_available_at(host="127.0.0.1", port=8082):
            if env_variables_set():
                sensor_available = True

    try:
        if not sensor_available:
            pytest.skip("Sensor not reachable.")
        yield

    # Cleanup
    finally:
        if ci_dummy.exists() and process is not None:
            process.kill()
