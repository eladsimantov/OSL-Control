#!/usr/bin/env python3
"""
main_fsm.py - Entry point for running the OpenSourceLeg Knee FSM on the Raspberry Pi or offline.

This script allows running either:
1. Baseline FSM (calibration + holding position)
2. Two-states Gait FSM (Stance/Swing transitions using load cell and timeouts)

All parameters (force thresholds, angles, CAN channel names) are defined inside:
- src/fsm/baseline.py
- src/fsm/two_states.py

Usage:
  # Run baseline FSM (will use settings from src/fsm/baseline.py)
  python main_fsm.py --mode baseline

  # Force offline simulation mode
  python main_fsm.py --mode two_states --offline
"""

import argparse
import sys
import os

# Ensure the root project directory is in the path
project_path = os.path.dirname(os.path.abspath(__file__))
if project_path not in sys.path:
    sys.path.insert(0, project_path)

from src.fsm.baseline import run_baseline_fsm
from src.fsm.two_states import run_two_states_fsm

def main():
    parser = argparse.ArgumentParser(
        description="OpenSourceLeg Knee Actuator Finite State Machine Runner"
    )

    parser.add_argument(
        "--mode",
        choices=["baseline", "two_states"],
        default="baseline",
        help="Which state machine to run."
    )
    
    parser.add_argument(
        "--offline",
        action="store_true",
        default=None,
        help="Force offline simulation mode. If not specified, uses the value from the FSM config."
    )

    args = parser.parse_args()

    # Pass the offline override if provided
    offline_override = True if args.offline else None

    if args.mode == "baseline":
        run_baseline_fsm(offline=offline_override)
    elif args.mode == "two_states":
        run_two_states_fsm(offline=offline_override)

if __name__ == "__main__":
    main()
