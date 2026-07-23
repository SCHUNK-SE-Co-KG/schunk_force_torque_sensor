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
REAL_SENSOR_PAIR = (
    (
        os.getenv("FTS_REAL_HOST_1", "10.49.60.117"),
        int(os.getenv("FTS_REAL_PORT_1", str(REAL_SENSOR_PORT))),
    ),
    (
        os.getenv("FTS_REAL_HOST_2", "10.49.60.123"),
        int(os.getenv("FTS_REAL_PORT_2", str(REAL_SENSOR_PORT))),
    ),
)

DUMMY_SENSOR_IP = "127.0.0.1"
DUMMY_SENSOR_PORT = 8082
DUMMY_SENSOR_PAIR_PORTS = (8082, 8083)


def sensor_available_at(host: str, port: int, timeout_sec=2.0) -> bool:
    start = time.time()
    while time.time() - start < timeout_sec:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.settimeout(0.1)
        except OSError:
            return False
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


def _workspace_src() -> Path:
    for parent in Path(__file__).resolve().parents:
        if (parent / "schunk_fts_dummy").is_dir():
            return parent
    return Path(__file__).resolve().parents[2]


def _dummy_binary() -> tuple[Path, Path] | None:
    workspace_src = _workspace_src()
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

    return dummy_dir, dummy_binary


def start_workspace_dummy(port: int = DUMMY_SENSOR_PORT) -> subprocess.Popen | None:
    dummy_paths = _dummy_binary()
    if dummy_paths is None:
        return None
    dummy_dir, dummy_binary = dummy_paths
    env = os.environ.copy()
    env["SCHUNK_FTS_DUMMY_TCP_PORT"] = str(port)

    process = subprocess.Popen(
        [dummy_binary],
        cwd=dummy_dir,
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    if sensor_available_at(DUMMY_SENSOR_IP, port, timeout_sec=5.0):
        return process

    process.kill()
    process.wait(timeout=2)
    return None


def _stop_process(process: subprocess.Popen | None) -> None:
    if process is None:
        return
    process.terminate()
    try:
        process.wait(timeout=2)
    except subprocess.TimeoutExpired:
        process.kill()


def _configured_sensor_pair() -> tuple[tuple[str, int], tuple[str, int]] | None:
    host_1 = os.getenv("FTS_HOST_1")
    port_1 = os.getenv("FTS_PORT_1")
    host_2 = os.getenv("FTS_HOST_2")
    port_2 = os.getenv("FTS_PORT_2")
    if host_1 is None or port_1 is None or host_2 is None or port_2 is None:
        return None
    return (host_1, int(port_1)), (host_2, int(port_2))


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

    _stop_process(process)


@pytest.fixture(scope="session")
def sensor_pair(request):
    configured_pair = _configured_sensor_pair()
    if configured_pair is not None:
        sensor_1, sensor_2 = configured_pair
        if not sensor_available_at(*sensor_1) or not sensor_available_at(*sensor_2):
            pytest.skip(
                "Configured sensor pair is not reachable: "
                f"{sensor_1[0]}:{sensor_1[1]}, {sensor_2[0]}:{sensor_2[1]}."
            )
        sensor_kind = "env-pair"
        processes: list[subprocess.Popen] = []
    elif all(sensor_available_at(host, port) for host, port in REAL_SENSOR_PAIR):
        sensor_1, sensor_2 = REAL_SENSOR_PAIR
        sensor_kind = "real-pair"
        processes = []
    else:
        processes = []
        sensors = []
        for port in DUMMY_SENSOR_PAIR_PORTS:
            if sensor_available_at(DUMMY_SENSOR_IP, port, timeout_sec=0.2):
                sensors.append((DUMMY_SENSOR_IP, port))
                continue
            process = start_workspace_dummy(port)
            if process is None:
                for started_process in processes:
                    _stop_process(started_process)
                pytest.skip("Could not start two dummy sensors for pair testing.")
            processes.append(process)
            sensors.append((DUMMY_SENSOR_IP, port))
        sensor_1, sensor_2 = sensors
        sensor_kind = "dummy-pair"

    request.config.sensor_pair_kind = sensor_kind
    request.config.sensor_pair = (sensor_1, sensor_2)
    yield sensor_1, sensor_2

    for process in processes:
        _stop_process(process)
