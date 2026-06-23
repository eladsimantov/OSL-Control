#!/usr/bin/env python3
"""
main_fsm.py - Entry point for running the Finite State Machine on the Raspberry Pi.

This script allows running either:
1. Baseline FSM (calibration/config + holding position)
2. Two-states Gait FSM (Stance/Swing transitions using load cell and timeouts)
   - Supports selecting the Stance phase controller (Impedance holding or CVP controller)
   - Supports dynamic offset adjustment in real-time

All parameters (force thresholds, angles, CAN channel names) are defined inside:
- src/fsm/baseline.py
- src/fsm/two_states.py

Usage:
  # Run baseline FSM
  python main_fsm.py --mode baseline

  # Run two-states FSM with standard impedance stance controller
  python main_fsm.py --mode two_states

  # Run two-states FSM with CVP stance controller and 5.0 degree offset
  python main_fsm.py --mode two_states --stance-control cvp --stance-offset 5.0
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
        "--stance-control",
        choices=["impedance", "cvp"],
        default="impedance",
        help="Stance controller type for two_states mode (default: impedance)."
    )

    parser.add_argument(
        "--stance-offset",
        type=float,
        default=0.0,
        help="Initial offset for the CVP stance controller in degrees (default: 0.0)."
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Maximum run duration in seconds (default: infinite)."
    )

    args = parser.parse_args()

    if args.mode == "baseline":
        run_baseline_fsm(
            max_duration=args.duration
        )
    elif args.mode == "two_states":
        run_two_states_fsm(
            max_duration=args.duration,
            stance_control=args.stance_control,
            stance_offset=args.stance_offset
        )

if __name__ == "__main__":
    main()
