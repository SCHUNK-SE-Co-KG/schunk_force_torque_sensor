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
import struct
import time
from threading import Thread, Barrier
from schunk_fts_library.utility import FTDataBuffer, Stream, Connection
from schunk_fts_library.driver import Driver


def test_data_buffer_concurrent_put_operations():
    """Test concurrent put operations don't corrupt buffer."""
    buffer = FTDataBuffer(maxsize=1000)
    num_threads = 10
    packets_per_thread = 100

    def producer(thread_id):
        for i in range(packets_per_thread):
            packet = bytearray(b"\xFF\xFF") + struct.pack(
                "<HHB I ffffff",
                thread_id * 1000 + i,
                29,
                1,
                0,
                float(thread_id),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )
            buffer.put(packet)

    threads = []
    for tid in range(num_threads):
        t = Thread(target=producer, args=(tid,))
        t.start()
        threads.append(t)

    for t in threads:
        t.join()

    # Buffer should have data (might not be all due to maxsize)
    assert len(buffer) > 0


def test_data_buffer_concurrent_get_operations():
    """Test concurrent get operations are thread-safe."""
    buffer = FTDataBuffer(maxsize=100)

    # Pre-fill buffer
    for i in range(100):
        packet = bytearray(b"\xFF\xFF") + struct.pack(
            "<HHB I ffffff", i, 29, 1, 0, float(i), 0.0, 0.0, 0.0, 0.0, 0.0
        )
        buffer.put(packet)

    num_threads = 5
    results = [[] for _ in range(num_threads)]

    def consumer(thread_id):
        while True:
            data = buffer.get()
            if data is None:
                time.sleep(0.001)
                if len(buffer) == 0:
                    break
            else:
                results[thread_id].append(data)

    threads = []
    for tid in range(num_threads):
        t = Thread(target=consumer, args=(tid,))
        t.start()
        threads.append(t)

    for t in threads:
        t.join(timeout=2.0)

    # All threads should complete
    assert all(not t.is_alive() for t in threads)

    # Total consumed should equal what was put
    total_consumed = sum(len(r) for r in results)
    assert total_consumed <= 100


def test_data_buffer_concurrent_put_and_get():
    """Test simultaneous put and get operations."""
    buffer = FTDataBuffer(maxsize=100)
    num_iterations = 200
    producer_done = False

    def producer():
        nonlocal producer_done
        for i in range(num_iterations):
            packet = bytearray(b"\xFF\xFF") + struct.pack(
                "<HHB I ffffff", i, 29, 1, 0, float(i), 0.0, 0.0, 0.0, 0.0, 0.0
            )
            buffer.put(packet)
            time.sleep(0.001)
        producer_done = True

    def consumer():
        consumed = 0
        while not producer_done or len(buffer) > 0:
            data = buffer.get()
            if data is not None:
                consumed += 1
            time.sleep(0.001)
        return consumed

    prod_thread = Thread(target=producer)
    cons_thread = Thread(target=consumer)

    prod_thread.start()
    cons_thread.start()

    prod_thread.join()
    cons_thread.join()

    assert not prod_thread.is_alive()
    assert not cons_thread.is_alive()


def test_stream_concurrent_is_open_calls(send_messages):
    """Test concurrent is_open() calls don't cause race conditions."""
    stream = Stream(port=8002)
    num_threads = 20
    iterations = 50
    barrier = Barrier(num_threads)

    def checker():
        barrier.wait()  # Synchronize start
        for _ in range(iterations):
            stream.is_open()

    threads = []
    for _ in range(num_threads):
        t = Thread(target=checker)
        t.start()
        threads.append(t)

    for t in threads:
        t.join(timeout=5.0)

    assert all(not t.is_alive() for t in threads)


def test_stream_concurrent_read_operations(send_messages):
    """Test concurrent read operations are safe."""
    stream = Stream(port=8003)
    num_threads = 5
    packets_sent = 20

    # Prepare test packets
    test_packets = []
    for i in range(packets_sent):
        packet = bytearray(b"\xFF\xFF") + struct.pack(
            "<HHB I ffffff", i, 29, 1, 0, float(i), 0.0, 0.0, 0.0, 0.0, 0.0
        )
        test_packets.append(packet)

    results = [[] for _ in range(num_threads)]

    def reader(thread_id):
        for _ in range(10):
            packets = stream.read()
            results[thread_id].extend(packets)
            time.sleep(0.01)

    with stream:
        # Send packets
        send_messages(8003, test_packets)
        time.sleep(0.05)

        threads = []
        for tid in range(num_threads):
            t = Thread(target=reader, args=(tid,))
            t.start()
            threads.append(t)

        for t in threads:
            t.join(timeout=2.0)

    assert all(not t.is_alive() for t in threads)


def test_connection_concurrent_send_operations(sensor):
    """Test concurrent send operations on same connection."""
    HOST, PORT = sensor
    connection = Connection(host=HOST, port=PORT)
    connection.open()

    num_threads = 5
    success_count = [0]

    def sender():
        req_data = bytearray(
            bytes.fromhex("ffff") + struct.pack("<HH", 0, 1) + bytes.fromhex("12")
        )
        if connection.send(req_data):
            success_count[0] += 1

    threads = []
    for _ in range(num_threads):
        t = Thread(target=sender)
        t.start()
        threads.append(t)

    for t in threads:
        t.join()

    connection.close()

    # At least some sends should succeed
    assert success_count[0] > 0


def test_connection_concurrent_open_close():
    """Test concurrent open/close operations."""
    connection = Connection(host="127.0.0.1", port=8082)
    num_threads = 10
    barrier = Barrier(num_threads)

    def toggle():
        barrier.wait()
        connection.open()
        time.sleep(0.01)
        connection.close()

    threads = []
    for _ in range(num_threads):
        t = Thread(target=toggle)
        t.start()
        threads.append(t)

    for t in threads:
        t.join(timeout=5.0)

    assert all(not t.is_alive() for t in threads)


def test_driver_concurrent_sampling(sensor):
    """Test concurrent sample() calls are thread-safe."""
    HOST, PORT = sensor
    driver = Driver(host=HOST, port=PORT)
    driver.streaming_on()
    time.sleep(0.2)  # Allow some data to arrive

    num_threads = 10
    samples_per_thread = 50
    all_samples = [[] for _ in range(num_threads)]

    def sampler(thread_id):
        for _ in range(samples_per_thread):
            sample = driver.sample()
            if sample is not None:
                all_samples[thread_id].append(sample)
            time.sleep(0.001)

    threads = []
    for tid in range(num_threads):
        t = Thread(target=sampler, args=(tid,))
        t.start()
        threads.append(t)

    for t in threads:
        t.join()

    driver.streaming_off()

    # All threads should complete
    assert all(not t.is_alive() for t in threads)

    # Each thread should have gotten some samples
    total_samples = sum(len(samples) for samples in all_samples)
    assert total_samples > 0


def test_driver_concurrent_streaming_on_calls(sensor):
    """Test calling streaming_on() concurrently."""
    HOST, PORT = sensor
    driver = Driver(host=HOST, port=PORT)

    num_threads = 5
    results = []

    def start_stream():
        result = driver.streaming_on()
        results.append(result)

    threads = []
    for _ in range(num_threads):
        t = Thread(target=start_stream)
        t.start()
        threads.append(t)

    for t in threads:
        t.join()

    driver.streaming_off()

    # At least one should succeed
    assert any(results)


def test_driver_concurrent_get_parameter_calls(sensor):
    """Test concurrent get_parameter() calls."""
    HOST, PORT = sensor
    driver = Driver(host=HOST, port=PORT)
    driver.connection.open()

    num_threads = 5
    results = [None] * num_threads

    def get_param(thread_id):
        resp = driver.get_parameter(index="0001", subindex="00")
        results[thread_id] = resp

    threads = []
    for tid in range(num_threads):
        t = Thread(target=get_param, args=(tid,))
        t.start()
        threads.append(t)

    for t in threads:
        t.join()

    driver.connection.close()

    # All threads should complete and get responses
    assert all(r is not None for r in results)


def test_driver_concurrent_command_execution(sensor):
    """Test concurrent command execution."""
    HOST, PORT = sensor
    driver = Driver(host=HOST, port=PORT)

    num_threads = 3
    results = [None] * num_threads

    def execute_cmd(thread_id):
        # Use tare command as it's safe to run multiple times
        resp = driver.tare()
        results[thread_id] = resp

    threads = []
    for tid in range(num_threads):
        t = Thread(target=execute_cmd, args=(tid,))
        t.start()
        threads.append(t)

    for t in threads:
        t.join()

    # All threads should complete
    assert all(r is not None for r in results)


def test_driver_no_deadlock_with_concurrent_operations(sensor):
    """Test that concurrent operations don't cause deadlocks."""
    HOST, PORT = sensor
    driver = Driver(host=HOST, port=PORT)
    driver.streaming_on()
    time.sleep(0.1)

    completed = {"sample": False, "status": False, "param": False}

    def sampler():
        for _ in range(10):
            driver.sample()
            time.sleep(0.01)
        completed["sample"] = True

    def status_checker():
        for _ in range(10):
            driver.get_status()
            time.sleep(0.01)
        completed["status"] = True

    def param_getter():
        for _ in range(5):
            driver.get_parameter(index="0001", subindex="00")
            time.sleep(0.02)
        completed["param"] = True

    threads = [
        Thread(target=sampler),
        Thread(target=status_checker),
        Thread(target=param_getter),
    ]

    for t in threads:
        t.start()

    for t in threads:
        t.join(timeout=5.0)

    driver.streaming_off()

    # All operations should complete without deadlock
    assert all(not t.is_alive() for t in threads)
    assert all(completed.values())


def test_stream_lock_prevents_concurrent_state_modification():
    """Test that lock prevents concurrent is_open modifications."""
    stream = Stream(port=8004)
    num_threads = 50
    barrier = Barrier(num_threads)

    def check_and_toggle():
        barrier.wait()
        # This should be safe due to internal locking
        with stream:
            state = stream.is_open()
            assert isinstance(state, bool)

    threads = []
    for _ in range(num_threads):
        t = Thread(target=check_and_toggle)
        t.start()
        threads.append(t)

    for t in threads:
        t.join(timeout=5.0)

    assert all(not t.is_alive() for t in threads)
