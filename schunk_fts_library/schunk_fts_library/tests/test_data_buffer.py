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
    data = FTData(
        sync=0xFFFF,
        counter=42,
        payload=29,
        id=0,
        status_bits=0,
        fx=1.0,
        fy=2.0,
        fz=3.0,
        tx=4.0,
        ty=5.0,
        tz=6.0,
    )
    buffer.put(data=data)
    assert buffer.get() == data


def test_buffer_supports_encoding_and_decoding():
    buffer = FTDataBuffer()
    data = FTData(
        sync=0xFFFF,
        counter=42,
        payload=29,
        id=0,
        status_bits=0,
        fx=1.0,
        fy=2.0,
        fz=3.0,
        tx=4.0,
        ty=5.0,
        tz=6.0,
    )
    encoded_data = buffer.encode(data)
    assert buffer.decode(encoded_data) == data


def test_buffer_knows_expected_length():
    buffer = FTDataBuffer()
    assert len(buffer) == buffer._length


def test_buffer_supports_concurrent_accesses():
    buffer = FTDataBuffer()

    nr_iterations = 100000

    data0 = FTData(
        sync=0xFFFF,
        counter=1,
        payload=29,
        id=0,
        status_bits=0,
        fx=0.0,
        fy=0.0,
        fz=0.0,
        tx=0.0,
        ty=0.0,
        tz=0.0,
    )
    data1 = FTData(
        sync=0xFFFF,
        counter=2,
        payload=29,
        id=1,
        status_bits=1,
        fx=1.1,
        fy=1.1,
        fz=1.1,
        tx=1.1,
        ty=1.1,
        tz=1.1,
    )

    def write():
        for n in range(nr_iterations):
            if n % 2 == 0:
                buffer.put(data=data0)
            else:
                buffer.put(data=data1)

    exception = None

    def read():
        nonlocal exception
        for n in range(nr_iterations):
            # Check data consistency
            data = buffer.get()
            data_consistent = data == data0 or data == data1
            try:
                assert data_consistent
            except AssertionError as e:
                print(f"data: {data}")
                exception = e

    threads = []

    # Single producer, single consumer
    writing_thread = Thread(target=write, daemon=True)
    writing_thread.start()
    threads.append(writing_thread)

    reading_thread = Thread(target=read, daemon=True)
    reading_thread.start()
    threads.append(reading_thread)

    for thread in threads:
        thread.join()
        assert not thread.is_alive()

    if exception:
        raise exception
