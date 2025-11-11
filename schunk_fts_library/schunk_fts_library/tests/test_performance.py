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
import time
import struct
import psutil
import os
from schunk_fts_library.driver import Driver
from schunk_fts_library.utility import FTDataBuffer, Stream


def test_performance_data_acquisition_rate(sensor):
    """Test data acquisition rate meets sensor specifications."""
    HOST, PORT = sensor
    driver = Driver(host=HOST, port=PORT)

    assert driver.streaming_on()
    time.sleep(0.1)  # Allow stream to stabilize

    # Measure acquisition rate over 1 second
    start_time = time.perf_counter()
    sample_count = 0
    test_duration = 1.0
    samples = []

    while time.perf_counter() - start_time < test_duration:
        sample = driver.sample()
        if sample is not None:
            sample_count += 1
            samples.append(sample)

    elapsed_time = time.perf_counter() - start_time
    driver.streaming_off()

    # Calculate rate
    actual_rate = sample_count / elapsed_time

    # Sensor should provide at least 100 Hz (typical minimum)
    # Real sensor can do up to 8000 Hz
    assert actual_rate >= 100, f"Rate too low: {actual_rate:.2f} Hz"

    # Verify samples are unique (not duplicates)
    unique_counters = set(s["counter"] for s in samples if s is not None)
    assert len(unique_counters) > sample_count * 0.9  # At least 90% unique


def test_performance_sampling_latency(sensor):
    """Test sampling latency is within acceptable range."""
    HOST, PORT = sensor
    driver = Driver(host=HOST, port=PORT)

    assert driver.streaming_on()
    time.sleep(0.2)  # Allow buffer to fill

    # Measure latency of 100 sample() calls
    latencies = []
    for _ in range(100):
        start = time.perf_counter()
        sample = driver.sample()
        end = time.perf_counter()
        if sample is not None:
            latencies.append((end - start) * 1000)  # Convert to ms

    driver.streaming_off()

    # Calculate statistics
    avg_latency = sum(latencies) / len(latencies)
    max_latency = max(latencies)

    # Latency should be low (< 10 ms average for sampling)
    assert avg_latency < 10.0, f"Average latency too high: {avg_latency:.3f} ms"
    assert max_latency < 50.0, f"Max latency too high: {max_latency:.3f} ms"


def test_performance_buffer_throughput(send_messages):
    """Test FTDataBuffer throughput."""
    buffer = FTDataBuffer(maxsize=1000)
    num_packets = 10000

    # Generate test packets
    packets = []
    for i in range(num_packets):
        packet = bytearray(b"\xFF\xFF") + struct.pack(
            "<HHB I ffffff", i, 29, 1, 0, float(i), 0.0, 0.0, 0.0, 0.0, 0.0
        )
        packets.append(packet)

    # Measure put throughput
    start = time.perf_counter()
    for packet in packets:
        buffer.put(packet)
    put_time = time.perf_counter() - start

    put_rate = num_packets / put_time

    # Should handle at least 10,000 packets/sec
    assert put_rate > 10000, f"Put rate too low: {put_rate:.0f} packets/sec"

    # Measure get throughput
    retrieved = 0
    start = time.perf_counter()
    while len(buffer) > 0:
        sample = buffer.get()
        if sample is not None:
            retrieved += 1
    get_time = time.perf_counter() - start

    get_rate = retrieved / get_time if get_time > 0 else 0

    # Should handle at least 5,000 packets/sec
    assert get_rate > 5000, f"Get rate too low: {get_rate:.0f} packets/sec"


def test_performance_decode_speed():
    """Test FTDataBuffer.decode performance."""
    # Create a typical packet
    packet = bytearray(b"\xFF\xFF") + struct.pack(
        "<HHB I ffffff", 12345, 29, 1, 0x00000001, 1.5, -2.3, 3.7, 0.15, -0.23, 0.37
    )

    num_iterations = 10000

    # Measure decode time
    start = time.perf_counter()
    for _ in range(num_iterations):
        FTDataBuffer.decode(packet)
    elapsed = time.perf_counter() - start

    decode_rate = num_iterations / elapsed

    # Should decode at least 50,000 packets/sec
    assert decode_rate > 50000, f"Decode rate too low: {decode_rate:.0f} packets/sec"


def test_performance_stream_read_overhead():
    """Test stream read operation overhead."""
    stream = Stream(port=54850)

    with stream:
        # Measure read overhead when no data available
        iterations = 1000
        start = time.perf_counter()
        for _ in range(iterations):
            stream.read()
        elapsed = time.perf_counter() - start

        avg_time_ms = (elapsed / iterations) * 1000

        # Should return immediately (< 1 ms average)
        assert avg_time_ms < 1.0, f"Read overhead too high: {avg_time_ms:.3f} ms"


def test_performance_memory_usage_stable(sensor):
    """Test that memory usage remains stable during streaming."""
    HOST, PORT = sensor
    driver = Driver(host=HOST, port=PORT)

    process = psutil.Process(os.getpid())
    initial_memory = process.memory_info().rss / 1024 / 1024  # MB

    assert driver.streaming_on()

    # Sample for 5 seconds
    end_time = time.time() + 5.0
    sample_count = 0
    while time.time() < end_time:
        sample = driver.sample()
        if sample is not None:
            sample_count += 1

    final_memory = process.memory_info().rss / 1024 / 1024  # MB
    driver.streaming_off()

    memory_increase = final_memory - initial_memory

    # Memory increase should be minimal (< 50 MB for 5 seconds)
    assert (
        memory_increase < 50
    ), f"Memory increased by {memory_increase:.2f} MB in 5 seconds"
    assert sample_count > 0, "No samples received during test"


def test_performance_buffer_queue_efficiency():
    """Test buffer queue operations are efficient."""
    buffer = FTDataBuffer(maxsize=100)

    # Fill buffer
    packet = bytearray(b"\xFF\xFF") + struct.pack(
        "<HHB I ffffff", 0, 29, 1, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    )

    start = time.perf_counter()
    for _ in range(100):
        buffer.put(packet)
    fill_time = time.perf_counter() - start

    # Should fill quickly (< 10 ms)
    assert fill_time < 0.01, f"Buffer fill too slow: {fill_time * 1000:.2f} ms"

    # Empty buffer
    start = time.perf_counter()
    while len(buffer) > 0:
        buffer.get()
    empty_time = time.perf_counter() - start

    # Should empty quickly (< 100 ms)
    assert empty_time < 0.1, f"Buffer empty too slow: {empty_time * 1000:.2f} ms"


def test_performance_concurrent_sampling_scalability(sensor):
    """Test that concurrent sampling scales reasonably."""
    from threading import Thread

    HOST, PORT = sensor
    driver = Driver(host=HOST, port=PORT)
    driver.streaming_on()
    time.sleep(0.2)

    def sampler(results, thread_id):
        count = 0
        start = time.perf_counter()
        for _ in range(100):
            if driver.sample() is not None:
                count += 1
        elapsed = time.perf_counter() - start
        results[thread_id] = (count, elapsed)

    # Test with different numbers of concurrent samplers
    for num_threads in [1, 2, 5, 10]:
        results = {}
        threads = []

        for tid in range(num_threads):
            t = Thread(target=sampler, args=(results, tid))
            t.start()
            threads.append(t)

        for t in threads:
            t.join()

        # Calculate average time per thread
        avg_time = sum(r[1] for r in results.values()) / num_threads

        # Time shouldn't increase dramatically with more threads
        assert avg_time < 2.0, f"Sampling too slow with {num_threads} threads"

    driver.streaming_off()


def test_performance_packet_counter_tracking(sensor):
    """Test packet counter tracking for dropped packet detection."""
    HOST, PORT = sensor
    driver = Driver(host=HOST, port=PORT)

    assert driver.streaming_on()
    time.sleep(1.0)  # Collect data

    # Check that producer is tracking counters
    packet_count = driver.producer_packet_count
    last_counter = driver.last_producer_counter

    driver.streaming_off()

    # Should have received packets
    assert packet_count > 0, "No packets received"
    assert last_counter >= 0, "Counter not tracked"


def test_performance_streaming_startup_time(sensor):
    """Test streaming startup time is reasonable."""
    HOST, PORT = sensor
    driver = Driver(host=HOST, port=PORT)

    start = time.perf_counter()
    success = driver.streaming_on(timeout_sec=2.0)
    startup_time = time.perf_counter() - start

    if success:
        driver.streaming_off()

        # Startup should be quick (< 1 second)
        assert startup_time < 1.0, f"Startup too slow: {startup_time:.3f} seconds"
    else:
        pytest.skip("Could not start streaming")


def test_performance_streaming_shutdown_time(sensor):
    """Test streaming shutdown time is reasonable."""
    HOST, PORT = sensor
    driver = Driver(host=HOST, port=PORT)

    assert driver.streaming_on()
    time.sleep(0.2)

    start = time.perf_counter()
    driver.streaming_off()
    shutdown_time = time.perf_counter() - start

    # Shutdown should be quick (< 0.5 second)
    assert shutdown_time < 0.5, f"Shutdown too slow: {shutdown_time:.3f} seconds"


def test_performance_buffer_no_memory_leak():
    """Test that buffer doesn't leak memory with continuous use."""
    import gc

    gc.collect()
    buffer = FTDataBuffer(maxsize=100)
    packet = bytearray(b"\xFF\xFF") + struct.pack(
        "<HHB I ffffff", 0, 29, 1, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    )

    process = psutil.Process(os.getpid())
    initial_memory = process.memory_info().rss / 1024 / 1024  # MB

    # Cycle through buffer many times
    for _ in range(10000):
        buffer.put(packet)
        buffer.get()

    gc.collect()
    final_memory = process.memory_info().rss / 1024 / 1024  # MB

    memory_increase = final_memory - initial_memory

    # Memory increase should be minimal (< 10 MB)
    assert memory_increase < 10, f"Possible memory leak: {memory_increase:.2f} MB"


def test_performance_message_serialization_speed():
    """Test message serialization/deserialization speed."""
    from schunk_fts_library.utility import GetParameterRequest, GetParameterResponse

    # Test request serialization
    req = GetParameterRequest()
    req.command_id = "f0"
    req.param_index = "0001"
    req.param_subindex = "00"

    iterations = 10000
    start = time.perf_counter()
    for _ in range(iterations):
        req.to_bytes()
    elapsed = time.perf_counter() - start

    serialization_rate = iterations / elapsed

    # Should serialize at least 100,000 messages/sec
    assert (
        serialization_rate > 100000
    ), f"Serialization too slow: {serialization_rate:.0f} msg/sec"

    # Test response deserialization
    response_data = bytearray()
    response_data += bytes.fromhex("ffff")
    response_data += struct.pack("<H", 1)
    response_data += struct.pack("<H", 7)
    response_data += bytes.fromhex("f0")
    response_data += bytes.fromhex("00")
    response_data += bytes.fromhex("0100")
    response_data += bytes.fromhex("00")
    response_data += bytes.fromhex("1234")

    start = time.perf_counter()
    for _ in range(iterations):
        resp = GetParameterResponse()
        resp.from_bytes(response_data)
    elapsed = time.perf_counter() - start

    deserialization_rate = iterations / elapsed

    # Should deserialize at least 50,000 messages/sec
    assert (
        deserialization_rate > 50000
    ), f"Deserialization too slow: {deserialization_rate:.0f} msg/sec"
