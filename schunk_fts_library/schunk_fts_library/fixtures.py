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
import pytest
from pathlib import Path
import subprocess
import os
import time
import socket
import logging

logger = logging.getLogger(__name__)

REAL_SENSOR_IP = "192.168.0.100"
REAL_SENSOR_PORT = 82

DUMMY_SENSOR_IP = "127.0.0.1"
DUMMY_SENSOR_PORT = 8082


def sensor_available_at(host: str, port: int, timeout_sec=2.0) -> bool:
    start = time.time()
    while time.time() - start < timeout_sec:
        while time.time() - start < timeout_sec:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(0.1)
            result = s.connect_ex((host, port))
            s.close()

            # 0 means success (connection accepted)
            # 111 (ECONNREFUSED) means nothing listening
            # 106 (EISCONN) or 98 (EADDRINUSE) often
            # mean "already connected" → consider that as reachable
            if result == 0 or result in (98, 106):
                return True

            time.sleep(0.1)
    return False


@pytest.fixture(scope="function")
def sensor(request):

    ci_dummy = Path("/tmp/schunk_fts_dummy/debug/schunk_fts_dummy")
    process = None

    def env_variables_set() -> bool:
        return os.getenv("FTS_HOST") is not None and os.getenv("FTS_PORT") is not None

    # CI setting
    if ci_dummy.exists():
        process = subprocess.Popen(
            [ci_dummy],
            stderr=subprocess.PIPE,
            text=True,
        )
        if sensor_available_at(host="127.0.0.1", port=8082) and env_variables_set():
            ip, port = DUMMY_SENSOR_IP, DUMMY_SENSOR_PORT
        else:
            pytest.skip("CI dummy sensor not reachable.")
    elif sensor_available_at(host=REAL_SENSOR_IP, port=REAL_SENSOR_PORT):
        # Real sensor setting
        ip, port = REAL_SENSOR_IP, REAL_SENSOR_PORT
    else:
        # Local dummy sensor setting
        for _ in range(20):
            if sensor_available_at(host=DUMMY_SENSOR_IP, port=DUMMY_SENSOR_PORT):
                ip, port = DUMMY_SENSOR_IP, DUMMY_SENSOR_PORT
                break
            time.sleep(0.5)
        else:
            pytest.skip("No sensor reachable for testing.")

    request.config.sensor_ip = ip
    request.config.sensor_port = port
    yield ip, port

    if process is not None:
        process.kill()
