#!/usr/bin/env python3

import time
import subprocess
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from luma.core.render import canvas

# Explicitly define I2C interface and OLED display
serial = i2c(port=1, address=0x3c)
device = ssd1306(serial, width=128, height=32)

# Calculate CPU usage from /proc/stat.
# This avoids using top and reduces system overhead.
# Returns current usage percentage along with updated idle/total counters.
def get_cpu_percent(prev_idle, prev_total):
    with open('/proc/stat', 'r') as f:
        fields = f.readline().strip().split()[1:]
        fields = list(map(int, fields))
        idle = fields[3]
        total = sum(fields)
    idle_delta = idle - prev_idle
    total_delta = total - prev_total
    usage = 100.0 * (1.0 - idle_delta / total_delta) if total_delta else 0
    return usage, idle, total

# Get memory usage percentage from /proc/meminfo.
# Calculates (total - available) / total * 100
def get_mem_percent():
    with open('/proc/meminfo', 'r') as f:
        lines = f.readlines()
        mem_total = int(lines[0].split()[1])
        mem_available = int(lines[2].split()[1])
    used = mem_total - mem_available
    return round((used / mem_total) * 100)

# Check for undervoltage using vcgencmd get_throttled.
# "0x0" means no undervoltage; any other value indicates undervoltage.
def get_undervoltage():
    # Run vcgencmd and remove any extra whitespace
    throttled = subprocess.getoutput("vcgencmd get_throttled | cut -d= -f2").strip()
    return throttled != "0x0"

# Draw all collected system stats to the OLED display.
# Shows CPU temp, CPU%, undervoltage alert, IP, disk usage, and memory usage.
def system_info(device, draw, temp, ip, cpu_usage, mem_percent, disk_usage, voltage_text):
    x = 2  # Left margin for drawing text
    draw.rectangle((x, 0, 128, 32), fill="black")

    # Line 1: CPU temperature and usage
    draw.text((x, 0), 'CPU:', fill="white")
    draw.text((36, 0), f"{temp}°C", fill="white")
    draw.text((82, 0), f"{int(cpu_usage)}%", fill="white")

    # Line 2: IP address and voltage status text
    draw.text((x, 10), 'IP:', fill="white")
    draw.text((20, 10), ip, fill="white")
    draw.text((90, 10), voltage_text, fill="white")

    # Line 3: SD card usage and memory usage percentage
    draw.text((x, 20), 'SD:', fill="white")
    draw.text((36, 20), disk_usage, fill="white")
    draw.text((70, 20), 'MEM:', fill="white")
    draw.text((104, 20), f"{mem_percent}%", fill="white")

    # Draw a white border around the screen
    draw.rectangle(device.bounding_box, outline="white")

# Initialize CPU stats and blinking timer
prev_idle, prev_total = 0, 0
blink_state = True   # Used for blinking the voltage display

# Timestamps to control update rates
last_sensor_update = 0
sensor_interval = 1    # seconds: sensor data updates at 1 Hz
blink_interval = 0.2   # seconds: display refresh at 5 Hz for blinking

# Cached sensor data values
temp = "??"
ip = "?.?.?.?"
cpu_usage = 0
mem_percent = 0
disk_usage = "--"
undervoltage_active = False  # Cached undervoltage status

while True:
    current_time = time.time()

    # Refresh sensor data at 1 Hz
    if current_time - last_sensor_update >= sensor_interval:
        # Get CPU temperature using vcgencmd.
        # Example output: "temp=48.0'C" → we strip non-numeric characters.
        temp = subprocess.getoutput("vcgencmd measure_temp | sed 's/[^0-9.*]//g'")

        # Get IP address using hostname -I (select first IP)
        ip = subprocess.getoutput("hostname -I | awk '{print $1}'")

        # Get disk usage of root (/) filesystem.
        disk_usage = subprocess.getoutput("df -Th / | awk '{print $6}' | tail -n1")

        # Get memory usage and CPU usage efficiently.
        mem_percent = get_mem_percent()
        cpu_usage, prev_idle, prev_total = get_cpu_percent(prev_idle, prev_total)

        # Cache undervoltage status (update once per second)
        undervoltage_active = get_undervoltage()

        last_sensor_update = current_time

    # Determine the voltage display text
    if undervoltage_active:
        # If undervoltage is active, blink "V: LOW" at 5 Hz.
        if blink_state:
            voltage_text = "V: LOW"
        else:
            voltage_text = "       "  # blank during off phase
        blink_state = not blink_state
    else:
        voltage_text = "V: OK "
        blink_state = True  # Not blinking if voltage is good

    # Render system info to the OLED display.
    with canvas(device) as draw:
        system_info(
            device, draw,
            temp=temp,
            ip=ip,
            cpu_usage=cpu_usage,
            mem_percent=mem_percent,
            disk_usage=disk_usage,
            voltage_text=voltage_text
        )

    # Wait for the blink interval
    time.sleep(blink_interval)
