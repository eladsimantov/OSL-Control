#!/usr/bin/env python3
"""
test_clock.py - Test and tutorial script for the OSL Clock (SoftRealtimeLoop).

This script demonstrates:
1. How to initialize and run a SoftRealtimeLoop at a target frequency (Hz).
2. How the loop automatically maintains consistent time steps (dt) and avoids drift.
3. How to access loop time metrics (elapsed time, sleep percentage, jitter).
4. How to stop the clock dynamically (e.g. clock.stop()).
5. How the loop handles CTRL-C (KeyboardInterrupt) signals gracefully.

Relationship to OSL readthedocs tutorial:
------------------------------------------
In the official OSL tutorial, `osl.clock` is used under the hood. Since the `OpenSourceLeg`
robot class depends on Dephy legacy actuators (which require the 'flexsea' package),
custom hardware projects (like this ODrive/SRI loadcell system) import the OSL clock utility
directly from `opensourceleg.utilities` as `SoftRealtimeLoop`. The features, loops, and
timing behavior are identical.

Author: Antigravity AI
Date: 2026-05-25
"""

import time
import os
import sys

# Add project path for imports
tests_path = os.path.dirname(os.path.abspath(__file__))
project_path = os.path.join(tests_path, "..")
if project_path not in sys.path:
    sys.path.insert(0, project_path)

from opensourceleg.utilities import SoftRealtimeLoop

def test_fixed_frequency_loop():
    print("\n--- 1. Testing a 10 Hz Loop (0.1s dt) for 2 seconds ---")
    frequency = 10.0
    dt = 1.0 / frequency
    
    # Initialize the clock. 
    # - report=True prints loop performance and timing metrics at loop termination.
    # - maintain_original_phase=False corrects dt relative to the previous iteration.
    clock = SoftRealtimeLoop(dt=dt, report=True)
    
    print("Loop starting. Press CTRL+C to interrupt it early...")
    try:
        # The clock yields the elapsed time (seconds) since the loop started.
        for t in clock:
            print(f"  Iteration: {clock.n:2d} | Elapsed Time: {t:.2f}s | Real Time: {time.monotonic():.3f}s")
            
            # Simulate some processing time inside the loop (must be less than dt)
            time.sleep(0.02)
            
            # Condition to stop the loop after 2 seconds
            if t >= 2.0:
                print("  Time limit reached. Stopping clock...")
                clock.stop()  # Raises StopIteration on the next check to exit loop cleanly
                
    except KeyboardInterrupt:
        print("  Loop interrupted by user.")
    finally:
        print("--- 10 Hz Loop Ended ---")


def test_dynamic_stop():
    print("\n--- 2. Testing a 100 Hz Loop (0.01s dt) with Dynamic Stop ---")
    frequency = 100.0
    dt = 1.0 / frequency
    
    # Running at a higher frequency for 100 cycles
    clock = SoftRealtimeLoop(dt=dt, report=True)
    
    print("Loop starting...")
    for t in clock:
        # Check every 20 cycles
        if clock.n % 20 == 0:
            print(f"  Cycle: {clock.n:3d} | Time elapsed: {t:.3f}s")
            
        # Perform mock operations (simulating reading ODrive encoders/currents)
        dummy_read = t * 10.0
        
        # Stop iteration after 100 cycles
        if clock.n >= 100:
            print(f"  Reached 100 iterations ({t:.3f}s). Stopping loop...")
            clock.stop()
            
    print("--- 100 Hz Loop Ended ---")


def test_loop_killer_signals():
    print("\n--- 3. Testing Loop Graceful Shutdown ---")
    print("This loop runs at 50 Hz. Try pressing CTRL+C to watch the loop")
    print("shut down cleanly and execute post-loop cleanup operations.")
    print("Running for maximum 3 seconds...")
    
    clock = SoftRealtimeLoop(dt=1.0/50.0, report=True)
    
    try:
        for t in clock:
            print(clock.n)
            # Doing mock sensor reads
            if clock.n % 10 == 0:
                print(f"  Time: {t:.2f}s (running...)")
            time.sleep(0.005)
            
            if t >= 3.0:
                print("  Maximum duration reached.")
                clock.stop()
    except KeyboardInterrupt:
        pass
    finally:
        # This code ALWAYS runs even if CTRL+C was pressed,
        # ensuring that motors can be idled and resources freed.
        print("  [CLEANUP] Idling actuators and shutting down CAN buses safely...")
        print("--- Loop Graceful Shutdown Done ---")


if __name__ == "__main__":
    test_fixed_frequency_loop()
    test_dynamic_stop()
    test_loop_killer_signals()
