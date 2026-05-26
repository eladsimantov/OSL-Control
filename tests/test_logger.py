import sys
import os

# Include project path for imports
project_path = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if project_path not in sys.path:
    sys.path.insert(0, project_path)

from opensourceleg.logging import LOGGER, LogLevel
from opensourceleg.utilities import SoftRealtimeLoop

class DummyActuator:
    """A dummy class representing an actuator to show attribute tracking."""
    def __init__(self):
        self.angle = 0.0
        self.current = 1.0


def main():
    # 1. Configure Logger levels and CSV file logging
    # Set console stream level (DEBUG logs everything, INFO hides debug prints)
    LOGGER.set_stream_level(LogLevel.DEBUG)
    
    # Enable CSV telemetry tracking and name the file
    LOGGER.set_csv_logging(True)
    LOGGER.set_file_name("gait_loop_log")

    # 2. Setup real-time telemetry variables to log to CSV
    x = 0.0
    knee = DummyActuator()
    current_state = "CALIBRATING"

    # Track lambdas / local variables
    LOGGER.track_function(lambda: x, "x_value")
    LOGGER.track_function(lambda: current_state, "state")
    
    # Track adapter properties
    LOGGER.track_attributes(knee, ["angle", "current"])

    # 3. Start real-time loop
    loop = SoftRealtimeLoop(dt=0.1)
    
    # [REPLACE PRINT] LOGGER.info is ideal for setup or event announcements
    LOGGER.info("FSM setup complete. Starting real-time control loop...")

    for t in loop:
        # Simulate variable updates in the control loop
        x = t * 10.0
        knee.angle = t * 0.5
        knee.current = 1.0 + 0.1 * t

        # State transition event
        if t >= 0.5 and current_state == "CALIBRATING":
            current_state = "STANCE"
            # [REPLACE PRINT] LOGGER.info is ideal for state transitions
            LOGGER.info(f"State transition -> {current_state}")

        # Update telemetry data buffer (saves to simple_log.csv in background)
        LOGGER.update()

        # [REPLACE PRINT] LOGGER.debug is ideal for high-frequency loop printouts. 
        # (Can be completely hidden in production by setting stream level to LogLevel.INFO)
        if int(t * 10) % 2 == 0:
            LOGGER.debug(f"Time: {t:.2f}s | State: {current_state} | Angle: {knee.angle:.2f} rad")

        # [REPLACE PRINT] LOGGER.warning is ideal for safety threshold checks
        if knee.current > 1.08:
            LOGGER.warning(f"High motor current threshold exceeded: {knee.current:.2f} A")

        if t >= 1.0:
            LOGGER.info("Gait loop duration reached. Terminating...")
            loop.stop()

    # 4. Flush and close log files (must be called to write CSV to disk)
    LOGGER.close()
    
    if LOGGER.csv_logging_enabled:
        LOGGER.info(f"Session data successfully saved to CSV: {LOGGER.csv_path}")


if __name__ == "__main__":
    main()
