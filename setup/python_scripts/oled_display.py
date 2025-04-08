#!/usr/bin/env python3

import time
import subprocess
import socket # For getting IP address efficiently
import psutil # For getting CPU%, Mem%, Disk% efficiently
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from luma.core.render import canvas

# --- Configuration ---
I2C_PORT = 1
I2C_ADDRESS = 0x3c
OLED_WIDTH = 128
OLED_HEIGHT = 32
SENSOR_INTERVAL = 2.0  # seconds: How often to fetch sensor data (Keep it low for efficiency)
DISPLAY_UPDATE_INTERVAL = 0.5 # seconds: Base refresh rate for display (e.g., for blinking)

# --- Setup I2C and OLED ---
try:
    serial = i2c(port=I2C_PORT, address=I2C_ADDRESS)
    device = ssd1306(serial, width=OLED_WIDTH, height=OLED_HEIGHT)
except Exception as e:
    print(f"Error initializing OLED: {e}")
    print("Is the OLED connected and I2C enabled?")
    exit(1)

# --- Helper Functions (Using efficient methods) ---

def get_ip_address():
    """Get the primary IP address of the machine."""
    try:
        # Create a dummy socket to connect to an external server (doesn't actually send data)
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0.1) # Prevent blocking indefinitely
        s.connect(("8.8.8.8", 80)) # Google DNS as target
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        # Could be no network, return placeholder
        return "?.?.?.?"

def get_cpu_temperature():
    """Get CPU temperature using vcgencmd (Raspberry Pi specific)."""
    # Note: Using vcgencmd as in the previous efficient version.
    # If you strongly prefer /sys/class/thermal/thermal_zone0/temp,
    # you can adapt the function from your original script.
    try:
        temp_str = subprocess.check_output("vcgencmd measure_temp", shell=True).decode("utf-8")
        # Example output: "temp=48.0'C" -> "48.0"
        return temp_str.split('=')[1].split("'")[0]
    except Exception:
        return "??" # Return placeholder on error

def get_undervoltage():
    """Check for undervoltage using vcgencmd (Raspberry Pi specific). Returns True if throttled."""
    try:
        throttled_str = subprocess.check_output("vcgencmd get_throttled", shell=True).decode("utf-8")
        # Example output: "throttled=0x50005" or "throttled=0x0"
        throttled_hex = throttled_str.split('=')[1].strip()
        # Check if under-voltage has occurred (bit 0) or is currently active (bit 16)
        throttled_val = int(throttled_hex, 16)
        return (throttled_val & 0x1) or (throttled_val & 0x10000)
    except Exception:
        return False # Assume OK if command fails

# --- Drawing Function (Using your original layout) ---

def system_info(device, draw, cpu_temp, ip, cpu_usage, mem_percent, disk_usage, voltage_text):
    """Draw the sensor data to the OLED display using original layout."""
    x = 2  # Left margin for drawing text
    # Clear background first
    draw.rectangle(device.bounding_box, outline="black", fill="black")

    # Line 1: CPU temperature and CPU usage.
    # Note: cpu_temp from get_cpu_temperature might be "48.0", adapt if needed
    # If get_cpu_temperature returns a number, format it: f"{cpu_temp:.1f}°C"
    # If it returns a string like "48.0", use it directly: f"{cpu_temp}°C"
    draw.text((x, 0), 'CPU:', fill="white")
    draw.text((36, 0), f"{cpu_temp}°C", fill="white") # Using value directly
    draw.text((82, 0), f"{int(cpu_usage)}%", fill="white")

    # Line 2: IP address and voltage status text.
    draw.text((x, 10), 'IP:', fill="white")
    draw.text((20, 10), ip, fill="white")
    draw.text((90, 10), voltage_text, fill="white")

    # Line 3: SD usage and memory usage percentage.
    # disk_usage should be like "XX%"
    draw.text((x, 20), 'SD:', fill="white")
    draw.text((36, 20), disk_usage, fill="white") # disk_usage is already formatted as "XX%"
    draw.text((70, 20), 'MEM:', fill="white")
    draw.text((104, 20), f"{int(mem_percent)}%", fill="white") # mem_percent is a float from psutil

    # Draw the white border around the screen from original
    draw.rectangle(device.bounding_box, outline="white")


# --- Main Loop (Efficient version) ---

# Initialize state variables
last_sensor_update_time = 0
last_display_update_time = 0
needs_redraw = True # Force initial draw
blink_state = True
voltage_text = "V: OK " # Initial text with trailing space

# Initialize cached sensor values
cached_temp = "??"
cached_ip = "?.?.?.?"
cached_cpu = 0.0
cached_mem = 0.0
cached_disk = "--%" # Initial format matches requirement
cached_undervoltage = False

# Initialize psutil's cpu_percent baseline
psutil.cpu_percent(interval=None)
time.sleep(0.1) # Short delay after first call

print("Starting system monitor loop with original layout...")
try:
    while True:
        current_time = time.time()
        force_redraw = False

        # --- 1. Gather Sensor Data (at SENSOR_INTERVAL) ---
        if current_time - last_sensor_update_time >= SENSOR_INTERVAL:
            cached_temp = get_cpu_temperature()
            cached_ip = get_ip_address()
            # Use psutil - more efficient
            cached_cpu = psutil.cpu_percent(interval=None) # Get % since last call
            cached_mem = psutil.virtual_memory().percent
            cached_disk_usage = psutil.disk_usage('/')
            cached_disk = f"{int(cached_disk_usage.percent)}%" # Format for display
            cached_undervoltage = get_undervoltage()

            last_sensor_update_time = current_time
            force_redraw = True # Data changed, need to redraw

        # --- 2. Determine Voltage Text & Blinking ---
        previous_voltage_text = voltage_text # Store state before logic
        if cached_undervoltage:
            # Blink "V: LOW" based on blink_state
            if blink_state:
                voltage_text = "V: LOW"
            else:
                voltage_text = "      "  # Blank text of same width as "V: LOW"
            # Only toggle blink state based on DISPLAY_UPDATE_INTERVAL timing
            if current_time - last_display_update_time >= DISPLAY_UPDATE_INTERVAL:
                 blink_state = not blink_state
        else:
            voltage_text = "V: OK " # Ensure trailing space matches "V: LOW" width approx
            blink_state = True # Reset blink state

        # Check if voltage text actually changed, necessitating a redraw
        if voltage_text != previous_voltage_text:
            force_redraw = True

        # --- 3. Update Display (Conditionally) ---
        # Redraw if forced (sensor update, voltage text change) AND enough time has passed
        if force_redraw and (current_time - last_display_update_time >= DISPLAY_UPDATE_INTERVAL):
            with canvas(device) as draw:
                # CALL THE ORIGINAL DRAWING FUNCTION
                system_info(
                    device, draw,
                    cpu_temp=cached_temp,       # Pass cached temp
                    ip=cached_ip,               # Pass cached IP
                    cpu_usage=cached_cpu,       # Pass cached CPU %
                    mem_percent=cached_mem,     # Pass cached Mem %
                    disk_usage=cached_disk,     # Pass cached Disk % (already formatted)
                    voltage_text=voltage_text   # Pass current voltage text
                )
            last_display_update_time = current_time # Record time of successful draw
            needs_redraw = False # We just redrew
        elif needs_redraw: # Handle the very first draw immediately
             with canvas(device) as draw:
                 # CALL THE ORIGINAL DRAWING FUNCTION
                 system_info(
                    device, draw,
                    cpu_temp=cached_temp,       # Use initial values
                    ip=cached_ip,
                    cpu_usage=cached_cpu,
                    mem_percent=cached_mem,
                    disk_usage=cached_disk,
                    voltage_text=voltage_text
                 )
             last_display_update_time = current_time
             needs_redraw = False


        # --- 4. Sleep ---
        # Calculate time until the *next* potential display update
        time_to_next_update = max(0, (last_display_update_time + DISPLAY_UPDATE_INTERVAL) - time.time())
        # Also consider time until next sensor read (though usually longer)
        time_to_next_sensor = max(0, (last_sensor_update_time + SENSOR_INTERVAL) - time.time())

        # Sleep for the minimum required time, but not excessively long.
        # Cap sleep time to avoid being unresponsive if intervals are very long.
        sleep_duration = min(time_to_next_update, time_to_next_sensor, DISPLAY_UPDATE_INTERVAL)

        # Ensure a tiny sleep even if 0 to prevent pegging CPU in tight loops
        time.sleep(max(0.05, sleep_duration))

except KeyboardInterrupt:
    print("Stopping.")
    # Optional: Clear display on exit
    try:
        with canvas(device) as draw:
            draw.rectangle(device.bounding_box, outline="black", fill="black")
    except Exception:
        pass # Ignore errors during cleanup
finally:
    # Optional: Cleanup resources if needed by luma library or others
    pass