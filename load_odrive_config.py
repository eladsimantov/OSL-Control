import odrive
from odrive.enums import *
import time

def configure_odrive(odrv):
    print("Configuring ODrive...")

    # Axis 0 Configuration
    axis0 = odrv.axis0

    # Motor configuration
    axis0.motor.config.current_lim = 10.0  # Example value, adjust as needed
    axis0.motor.config.pole_pairs = 7  # Example value, adjust as needed
    axis0.motor.config.calibration_current = 10.0
    axis0.motor.config.resistance_calib_max_voltage = 2.0
    axis0.motor.config.requested_current_range = 25.0
    axis0.motor.config.torque_constant = 8.27

    # Encoder configuration
    axis0.encoder.config.cpr = 8192
    axis0.encoder.config.mode = ENCODER_MODE_INCREMENTAL
    axis0.encoder.config.bandwidth = 1000.0

    # Controller configuration
    axis0.controller.config.pos_gain = 1.0
    axis0.controller.config.vel_gain = 0.02
    axis0.controller.config.vel_integrator_gain = 0.1
    axis0.controller.config.vel_limit = 10.0

    # General axis configuration
    axis0.config.startup_motor_calibration = True
    axis0.config.startup_encoder_index_search = False
    axis0.config.startup_encoder_offset_calibration = True
    axis0.config.startup_closed_loop_control = True

    # CAN configuration
    odrv.can.config.baud_rate = 250000  # Example value, adjust as needed

    # System-wide configuration
    odrv.config.dc_bus_overvoltage_trip_level = 30.0
    odrv.config.dc_bus_undervoltage_trip_level = 14.0
    odrv.config.max_regen_current = 0.0
    odrv.config.uart_a_baudrate = 115200

    print("Configuration applied.")

def main():
    print("Finding an ODrive...")
    odrv = odrive.find_any()

    # Apply configuration
    configure_odrive(odrv)

    # Save the configuration to the ODrive
    print("Saving configuration...")
    odrv.save_configuration()

    # Reboot the ODrive to apply the changes
    print("Rebooting ODrive...")
    odrv.reboot()

    print("Configuration saved and ODrive rebooted.")

if __name__ == "__main__":
    main()