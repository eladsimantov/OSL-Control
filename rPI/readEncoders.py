import pigpio 
import time

# Initialize pigpio
pi = pigpio.pi()
if not pi.connected:
    exit(0)

# GPIO for PWM input from encoder
PWM_PIN = 18
pi.set_mode(PWM_PIN, pigpio.INPUT)

def read_pwm_duty_cycle(pin):
    duty_cycle, frequency = pi.pwm(pin)
    return duty_cycle / 1000000.0  # Convert to percentage
