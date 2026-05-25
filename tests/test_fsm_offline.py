#!/usr/bin/env python3
"""
test_fsm_offline.py - Automated test script to verify that both Baseline FSM
and Two-States Gait FSM execute successfully in offline simulation mode.
"""

import os
import sys

# Ensure the root project directory is in the path
tests_path = os.path.dirname(os.path.abspath(__file__))
project_path = os.path.join(tests_path, "..")
if project_path not in sys.path:
    sys.path.insert(0, project_path)

from src.fsm.baseline import run_baseline_fsm
from src.fsm.two_states import run_two_states_fsm

def test_offline_baseline():
    print("\n" + "="*50)
    print("      TESTING BASELINE FSM (OFFLINE MODE) ")
    print("="*50 + "\n")
    try:
        # Run baseline FSM in offline mode for 2.0 seconds
        run_baseline_fsm(offline=True, max_duration=2.0)
        print("\n--> Offline Baseline FSM completed successfully.")
        return True
    except Exception as e:
        print(f"\n--> ERROR running Offline Baseline FSM: {e}", file=sys.stderr)
        return False

def test_offline_two_states():
    print("\n" + "="*50)
    print("      TESTING TWO-STATES FSM (OFFLINE MODE) ")
    print("="*50 + "\n")
    try:
        # Run two-states FSM in offline mode for 2.0 seconds
        run_two_states_fsm(offline=True, max_duration=2.0)
        print("\n--> Offline Two-States FSM completed successfully.")
        return True
    except Exception as e:
        print(f"\n--> ERROR running Offline Two-States FSM: {e}", file=sys.stderr)
        return False

if __name__ == "__main__":
    success_baseline = test_offline_baseline()
    success_two_states = test_offline_two_states()
    
    print("\n" + "="*50)
    print("               TEST RUN SUMMARY ")
    print("="*50)
    print(f"Baseline FSM Offline Run:   {'PASSED' if success_baseline else 'FAILED'}")
    print(f"Two-States FSM Offline Run: {'PASSED' if success_two_states else 'FAILED'}")
    print("="*50 + "\n")
    
    if not (success_baseline and success_two_states):
        sys.exit(1)
    sys.exit(0)
