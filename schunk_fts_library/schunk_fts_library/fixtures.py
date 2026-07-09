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
import shutil

logger = logging.getLogger(__name__)

REAL_SENSOR_IP = os.getenv("FTS_REAL_HOST", "10.49.60.117")
REAL_SENSOR_PORT = int(os.getenv("FTS_REAL_PORT", "82"))

DUMMY_SENSOR_IP = "127.0.0.1"
DUMMY_SENSOR_PORT = 8082


def sensor_available_at(host: str, port: int, timeout_sec=2.0) -> bool:
    start = time.time()
    while time.time() - start < timeout_sec:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.settimeout(0.1)
        try:
            result = s.connect_ex((host, port))
        finally:
            s.close()

        # 0 means success (connection accepted)
        # 111 (ECONNREFUSED) means nothing listening
        # 106 (EISCONN) or 98 (EADDRINUSE) often
        # mean "already connected" → consider that as reachable
        if result == 0 or result in (98, 106):
            return True

        time.sleep(0.1)
    return False


def start_workspace_dummy() -> subprocess.Popen | None:
    workspace_src = Path(__file__).resolve().parents[2]
    dummy_dir = workspace_src / "schunk_fts_dummy"
    dummy_binary = dummy_dir / "target" / "debug" / "schunk_fts_dummy"

    if not dummy_dir.exists():
        return None

    cargo = shutil.which("cargo")
    if cargo is not None:
        subprocess.run(
            [cargo, "build", "--quiet"],
            cwd=dummy_dir,
            check=True,
            timeout=120,
        )
    elif not dummy_binary.exists():
        return None

    process = subprocess.Popen(
        [dummy_binary],
        cwd=dummy_dir,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    if sensor_available_at(DUMMY_SENSOR_IP, DUMMY_SENSOR_PORT, timeout_sec=5.0):
        return process

    process.kill()
    process.wait(timeout=2)
    return None


@pytest.fixture(scope="session")
def sensor(request):

    ci_dummy = Path("/tmp/schunk_fts_dummy/debug/schunk_fts_dummy")
    process = None

    env_host = os.getenv("FTS_HOST")
    env_port = os.getenv("FTS_PORT")
    if env_host is not None and env_port is not None:
        ip, port = env_host, int(env_port)
        if not sensor_available_at(host=ip, port=port):
            pytest.skip(f"Configured sensor at {ip}:{port} not reachable.")
        sensor_kind = "env"
    elif sensor_available_at(host=DUMMY_SENSOR_IP, port=DUMMY_SENSOR_PORT):
        ip, port = DUMMY_SENSOR_IP, DUMMY_SENSOR_PORT
        sensor_kind = "dummy-running"
    elif ci_dummy.exists():
        process = subprocess.Popen(
            [ci_dummy],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            text=True,
        )
        if sensor_available_at(host=DUMMY_SENSOR_IP, port=DUMMY_SENSOR_PORT):
            ip, port = DUMMY_SENSOR_IP, DUMMY_SENSOR_PORT
            sensor_kind = "dummy-ci"
        else:
            pytest.skip("CI dummy sensor not reachable.")
    elif process := start_workspace_dummy():
        ip, port = DUMMY_SENSOR_IP, DUMMY_SENSOR_PORT
        sensor_kind = "dummy-workspace"
    elif sensor_available_at(host=REAL_SENSOR_IP, port=REAL_SENSOR_PORT):
        ip, port = REAL_SENSOR_IP, REAL_SENSOR_PORT
        sensor_kind = "real"
    else:
        pytest.skip("No dummy or real sensor reachable for testing.")

    request.config.sensor_ip = ip
    request.config.sensor_port = port
    request.config.sensor_kind = sensor_kind
    yield ip, port

    if process is not None:
        process.terminate()
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            process.kill()
