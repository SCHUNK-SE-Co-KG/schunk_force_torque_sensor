#!/usr/bin/env python3
"""
Test script to verify automatic reconnection functionality.

This script simulates connection loss by monitoring the UDP stream and
demonstrates that the driver will automatically reconnect when the sensor
comes back online.

Usage:
    # Terminal 1: Run the dummy sensor
    cd schunk_fts_dummy && cargo run

    # Terminal 2: Test reconnection
    source install/setup.bash
    python3 test_reconnection.py
"""

import pytest
import time
from schunk_fts_library.driver import Driver


@pytest.mark.skip(reason="Requires manual sensor disconnect/reconnect actions")
def test_auto_reconnection():
    """Test automatic reconnection when sensor connection is lost."""
    print("=" * 80)
    print("AUTOMATIC RECONNECTION TEST")
    print("=" * 80)

    # Connect to sensor (dummy at 127.0.0.1:8082 or real at 192.168.0.100:82)
    host = "192.168.0.100"
    port = 82
    streaming_port = 54843

    print(f"\n1. Connecting to sensor at {host}:{port}...")
    driver = Driver(host=host, port=port, streaming_port=streaming_port)

    # Start streaming with auto-reconnect enabled
    if not driver.streaming_on(timeout_sec=1.0, auto_reconnect=True):
        print("   ❌ Failed to start streaming")
        print(
            "   Make sure the dummy sensor is running: cd schunk_fts_dummy && cargo run"
        )
        return False

    print("   ✓ Streaming started successfully")

    # Verify we're receiving data
    print("\n2. Verifying data reception...")
    data_count = 0
    start_time = time.time()
    while time.time() - start_time < 2.0:
        data = driver.sample()
        if data:
            data_count += 1
        time.sleep(0.01)

    print(f"   ✓ Received {data_count} data samples")

    # Instructions for manual test
    print("\n3. Manual reconnection test:")
    print("   Please STOP the dummy sensor now (Ctrl+C in the sensor terminal)")
    print("   or disconnect the real sensor if you're testing with hardware.")
    print()

    # Wait for user to disconnect sensor
    input("   Press ENTER after you have DISCONNECTED the sensor...")
    print()
    print("   Waiting for connection loss detection (should happen within 0.5s)...")

    # Wait for connection loss to be detected
    connection_lost_detected = False
    wait_start = time.time()
    while time.time() - wait_start < 5.0:
        data = driver.sample()
        time.sleep(0.1)
        if not data or data["counter"] == 0:
            # This means we're getting empty data (connection lost)
            connection_lost_detected = True
            break

    if connection_lost_detected:
        print("   ✓ Connection loss detected!")
    else:
        print("   ⚠ Connection loss not detected yet (continuing anyway)")

    print()
    print("   Now RESTART the sensor:")
    print("   - For dummy sensor: Run 'cargo run' in schunk_fts_dummy directory")
    print("   - For real sensor: Power it back on or reconnect network")
    print()
    input("   Press ENTER after you have RESTARTED the sensor...")
    print()
    print("   Monitoring for automatic reconnection (up to 30 seconds)...")

    # Monitor for reconnection
    start_time = time.time()
    sample_counts = []
    reconnection_detected = False

    while time.time() - start_time < 30.0:
        # Count samples in 1-second windows
        window_start = time.time()
        window_count = 0

        while time.time() - window_start < 1.0:
            data = driver.sample()
            if data:
                window_count += 1
                status = driver.get_status(data)
                if status:
                    pass  # Status is good
            time.sleep(0.001)

        # Print status every second
        elapsed = int(time.time() - start_time)
        if window_count > 0:
            print(f"   [{elapsed:2d}s] ✓ Receiving data ({window_count} samples/sec)")
            if not reconnection_detected:
                print("   ✓✓✓ RECONNECTION SUCCESSFUL! ✓✓✓")
                reconnection_detected = True
        else:
            print(f"   [{elapsed:2d}s] ⚠ No data (waiting for reconnection...)")

        sample_counts.append(window_count)

        # Exit early if we've successfully reconnected
        if reconnection_detected and elapsed >= 5:
            print(f"   Reconnection verified for {elapsed} seconds. Test successful!")
            break

    # Summary
    print("\n4. Test Summary:")
    windows_with_data = sum(1 for c in sample_counts if c > 0)
    windows_without_data = sum(1 for c in sample_counts if c == 0)
    print(f"   Windows with data: {windows_with_data}")
    print(f"   Windows without data: {windows_without_data}")

    if reconnection_detected:
        print("   ✓✓✓ RECONNECTION TEST PASSED! ✓✓✓")
        print(
            "       The driver successfully detected connection loss and reconnected!"
        )
    else:
        print("   ❌ RECONNECTION TEST FAILED")
        print("      The driver did not successfully reconnect within 30 seconds")

    # Cleanup
    print("\n5. Stopping streaming...")
    driver.streaming_off()
    print("   ✓ Streaming stopped")

    print("\n" + "=" * 80)
    print("TEST COMPLETE")
    print("=" * 80)
    return True


if __name__ == "__main__":
    try:
        success = test_auto_reconnection()
        exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        exit(1)
    except Exception as e:
        print(f"\n\n❌ Test failed with error: {e}")
        import traceback

        traceback.print_exc()
        exit(1)
