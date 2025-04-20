#!/usr/bin/env python3

"""
Controls the 52Pi PWM fan based on CPU temperature.

Reads CPU temperature from /sys/class/thermal/thermal_zone0/temp and adjusts
the fan speed connected to BCM pin 14 using PWM.
"""

import time
import RPi.GPIO as GPIO
import sys  # For exiting gracefully

# --- Configuration ---
FAN_PIN = 14  # BCM pin used to control the fan PWM
PWM_FREQ = 100  # Hz - Frequency for PWM signal
CHECK_INTERVAL = 5  # seconds - How often to check temperature and adjust fan

# Temperature thresholds and corresponding duty cycles (in percent)
# Ensure keys are sorted from lowest temp to highest
# The fan runs at the speed defined for the highest threshold met.
# Example: 42°C meets the 40°C threshold, fan runs at 85%
#          47°C meets the 45°C threshold, fan runs at 100%
SPEED_MAP = {
    0: 0,  # Temperature below first threshold (e.g., < 40°C), fan is off
    40: 85,  # Temperature >= 40°C
    45: 100,  # Temperature >= 45°C
}

# --- Functions ---


def get_cpu_temp():
    """Read CPU temperature from system file and return in degrees Celsius."""
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            temp_milli = int(f.read().strip())
        return temp_milli / 1000.0
    except (FileNotFoundError, ValueError, OSError) as e:
        print(f"Warning: Could not read CPU temperature: {e}", file=sys.stderr)
        # Return a safe default (e.g., high temp to ensure fan runs if sensor fails)
        # Or return None/raise exception if preferred error handling
        return 99.0  # Assume high temp on error to be safe


def get_fan_duty_cycle(temp):
    """Determine the fan duty cycle based on temperature and SPEED_MAP."""
    # Iterate through thresholds from highest to lowest
    # The first threshold met determines the speed
    sorted_thresholds = sorted(SPEED_MAP.keys(), reverse=True)
    for threshold in sorted_thresholds:
        if temp >= threshold:
            return SPEED_MAP[threshold]
    return 0  # Should not be reached if 0 threshold exists, but safe default


# --- Main Execution ---

pwm = None  # Initialize pwm variable

try:
    # Setup GPIO
    GPIO.setwarnings(False)  # Suppress warnings about channel usage
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(FAN_PIN, GPIO.OUT)

    # Initialize PWM
    pwm = GPIO.PWM(FAN_PIN, PWM_FREQ)
    pwm.start(0)  # Start with fan off

    print("Starting fan control loop...")
    while True:
        temp = get_cpu_temp()
        duty_cycle = get_fan_duty_cycle(temp)

        # Apply the new duty cycle
        pwm.ChangeDutyCycle(duty_cycle)

        # Optional: Log temperature and duty cycle
        # print(f"Temp: {temp:.1f}°C -> Fan Duty Cycle: {duty_cycle}%")

        # Wait for the next check
        time.sleep(CHECK_INTERVAL)

except KeyboardInterrupt:
    print("KeyboardInterrupt detected. Stopping fan and cleaning up.")
except Exception as e:
    print(f"An error occurred: {e}", file=sys.stderr)
finally:
    # Ensure cleanup happens on exit, error, or KeyboardInterrupt
    print("Cleaning up GPIO...")
    if pwm:
        pwm.stop()
    GPIO.cleanup()
    print("Fan control stopped.")
