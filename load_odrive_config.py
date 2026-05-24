import odrive
from odrive.enums import *
import time
import math

# def configure_odrive(odrv):
#     print("Configuring ODrive...")

#     # Axis 0 Configuration
#     axis0 = odrv.axis0

#     # Motor configuration
#     axis0.motor.config.current_lim = 10.0  # Example value, adjust as needed
#     axis0.motor.config.pole_pairs = 21  # Example value, adjust as needed
#     axis0.motor.config.calibration_current = 10.0
#     axis0.motor.config.resistance_calib_max_voltage = 2.0
#     axis0.motor.config.requested_current_range = 25.0
#     axis0.motor.config.torque_constant = 8.27

#     # Encoder configuration
#     axis0.encoder.config.cpr = 8192
#     axis0.encoder.config.mode = ENCODER_MODE_INCREMENTAL
#     axis0.encoder.config.bandwidth = 1000.0

#     # Controller configuration
#     axis0.controller.config.pos_gain = 1.0
#     axis0.controller.config.vel_gain = 0.02
#     axis0.controller.config.vel_integrator_gain = 0.1
#     axis0.controller.config.vel_limit = 10.0

#     # General axis configuration
#     axis0.config.startup_motor_calibration = True
#     axis0.config.startup_encoder_index_search = False
#     axis0.config.startup_encoder_offset_calibration = True
#     axis0.config.startup_closed_loop_control = True

#     # CAN configuration
#     odrv.can.config.baud_rate = 250000  # Example value, adjust as needed

#     # System-wide configuration
#     odrv.config.dc_bus_overvoltage_trip_level = 30.0
#     odrv.config.dc_bus_undervoltage_trip_level = 14.0
#     odrv.config.max_regen_current = 0.0
#     odrv.config.uart_a_baudrate = 115200

#     print("Configuration applied.")

def david_config(odrv):
    print("Configuring ODrive as David did...")

    odrv.config.dc_bus_overvoltage_trip_level = 24
    odrv.config.dc_bus_undervoltage_trip_level = 10.5
    odrv.config.dc_max_positive_current = math.inf
    odrv.config.dc_max_negative_current = -math.inf
    odrv.config.brake_resistor0.enable = False
    odrv.axis0.config.motor.motor_type = MotorType.PMSM_CURRENT_CONTROL
    odrv.axis0.config.motor.pole_pairs = 21
    odrv.axis0.config.motor.torque_constant = 0.0827
    odrv.axis0.config.motor.current_soft_max = 10
    odrv.axis0.config.motor.current_hard_max = 23
    odrv.axis0.config.motor.calibration_current = 10
    odrv.axis0.config.motor.resistance_calib_max_voltage = 2
    odrv.axis0.config.calibration_lockin.current = 10
    odrv.axis0.motor.motor_thermistor.config.enabled = False
    odrv.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
    odrv.axis0.controller.config.input_mode = InputMode.VEL_RAMP
    odrv.axis0.controller.config.vel_limit = 10
    odrv.axis0.controller.config.vel_limit_tolerance = 1.2
    odrv.axis0.config.torque_soft_min = -math.inf
    odrv.axis0.config.torque_soft_max = math.inf
    odrv.axis0.trap_traj.config.accel_limit = 10
    odrv.axis0.controller.config.vel_ramp_rate = 10
    odrv.can.config.protocol = Protocol.SIMPLE
    odrv.can.config.baud_rate = 0
    odrv.axis0.config.can.node_id = 0
    odrv.axis0.config.can.heartbeat_msg_rate_ms = 100
    odrv.axis0.config.can.encoder_msg_rate_ms = 10
    odrv.axis0.config.can.iq_msg_rate_ms = 10
    odrv.axis0.config.can.torques_msg_rate_ms = 10
    odrv.axis0.config.can.error_msg_rate_ms = 100
    odrv.axis0.config.can.temperature_msg_rate_ms = 1000
    odrv.axis0.config.can.bus_voltage_msg_rate_ms = 1000
    odrv.axis0.config.enable_watchdog = False
    odrv.axis0.config.load_encoder = EncoderId.ONBOARD_ENCODER0
    odrv.axis0.config.commutation_encoder = EncoderId.ONBOARD_ENCODER0
    odrv.config.enable_uart_a = False

def main():
    print("Finding an ODrive...")
    odrv = odrive.find_any()

    # Apply configuration
    david_config(odrv)
    # configure_odrive(odrv)

    # Save the configuration to the ODrive
    print("Saving configuration...")
    odrv.save_configuration()

    # Reboot the ODrive to apply the changes
    print("Rebooting ODrive...")
    odrv.reboot()

    print("Configuration saved and ODrive rebooted.")

if __name__ == "__main__":
    main()