import pigpio
import time

# Initialize pigpio
pi = pigpio.pi()
if not pi.connected:
    exit()

# GPIO pin for PWM input
PWM_PIN = 18
pi.set_mode(PWM_PIN, pigpio.INPUT)

# Function to read PWM duty cycle
def read_pwm_duty_cycle(pin):
    # Get PWM parameters: returns (duty_cycle, frequency)
    duty_cycle, frequency = pi.pwm(pin)
    return duty_cycle / 1000000.0  # Duty cycle as fraction (0.0 to 1.0)

# Main loop to read and print PWM
try:
    while True:
        duty = read_pwm_duty_cycle(PWM_PIN)
        print(f"PWM Duty Cycle: {duty:.3f}")
        time.sleep(0.1)  # Adjust sampling rate as needed
except KeyboardInterrupt:
    pi.stop()