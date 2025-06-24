from schunk_fts_library.utility import FTDataBuffer, FTData
from threading import Thread
import pytest


def test_buffer_offers_putting_and_getting_data():
    buffer = FTDataBuffer()

    # Without data
    data = buffer.get()
    assert data["id"] == 0
    assert data["status_bits"] == 0
    assert pytest.approx(data["fx"]) == 0.0
    assert pytest.approx(data["fy"]) == 0.0
    assert pytest.approx(data["fz"]) == 0.0
    assert pytest.approx(data["tx"]) == 0.0
    assert pytest.approx(data["ty"]) == 0.0
    assert pytest.approx(data["tz"]) == 0.0

    # Put + get
    data = FTData(id=0, status_bits=0, fx=1.0, fy=2.0, fz=3.0, tx=4.0, ty=5.0, tz=6.0)
    buffer.put(data=data)
    assert buffer.get() == data


def test_buffer_supports_concurrent_accesses():
    buffer = FTDataBuffer()

    nr_iterations = 100

    def access():
        for n in range(nr_iterations):
            data = FTData(
                id=0, status_bits=0, fx=1.0, fy=2.0, fz=3.0, tx=4.0, ty=5.0, tz=6.0
            )
            buffer.put(data=data)
            assert buffer.get() == data

    threads = []
    for i in range(10):
        thread = Thread(target=access, daemon=True)
        thread.start()
        threads.append(thread)

    for thread in threads:
        thread.join()
        assert not thread.is_alive()
