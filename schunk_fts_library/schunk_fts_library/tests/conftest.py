import pytest
from schunk_fts_library.utility import Connection
from pathlib import Path
import subprocess


@pytest.fixture(scope="module")
def fts_dummy():
    print("start FTS dummy")

    # Run the dummy if we are in a CI setup.
    ci_dummy = Path("/tmp/schunk_fts_dummy/debug/schunk_fts_dummy-")
    if ci_dummy.exists():
        process = subprocess.Popen(
            [ci_dummy],
            stderr=subprocess.PIPE,
            text=True,
        )

    # Check if we can reach the dummy
    with Connection(host="127.0.0.1", port=8082) as connection:
        if not connection:
            pytest.skip("FTS dummy not reachable.")

    # Cleanup
    if ci_dummy.exists():
        process.kill()
    print("stop FTS dummy")
