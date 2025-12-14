import pigpio
import time

# === CONFIG ===
RC_PIN = 17  # GPIO pin connected to RC receiver signal

# === Setup pigpio ===
pi = pigpio.pi()
if not pi.connected:
    raise Exception("Failed to connect to pigpio daemon. Run 'sudo pigpiod' first.")

# Variables to store pulse width
pulse_width = 1500  # Default middle position
pulse_start = 0

def pwm_callback(gpio, level, tick):
    """Callback function to measure PWM pulse width"""
    global pulse_width, pulse_start
    if level == 1:  # Rising edge
        pulse_start = tick
    elif level == 0:  # Falling edge
        pulse_width = pigpio.tickDiff(pulse_start, tick)

def map_pwm_to_range(pwm_value, min_pwm=1000, max_pwm=2000, min_out=-1.0, max_out=1.0):
    """Map PWM value (typically 1000-2000) to desired range (-1 to 1)"""
    # Clamp to expected range
    pwm_value = max(min_pwm, min(max_pwm, pwm_value))
    # Map to output range
    return min_out + (pwm_value - min_pwm) * (max_out - min_out) / (max_pwm - min_pwm)

# Set up GPIO pin and callback
pi.set_mode(RC_PIN, pigpio.INPUT)
cb = pi.callback(RC_PIN, pigpio.EITHER_EDGE, pwm_callback)

print("Reading PWM signal from GPIO 17...")
print("Press Ctrl+C to exit\n")

try:
    while True:
        raw_value = pulse_width
        mapped_value = map_pwm_to_range(raw_value)
        
        print(f"Raw PWM: {raw_value:4d} μs  |  Mapped: {mapped_value:+.3f}", end='\r')
        time.sleep(0.1)  # Update 10 times per second

except KeyboardInterrupt:
    print("\n\nStopping...")

finally:
    cb.cancel()
    pi.stop()
    print("Cleanup complete")
