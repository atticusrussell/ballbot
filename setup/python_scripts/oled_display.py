#!/usr/bin/env python3

import time
import subprocess
import socket
import os
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from luma.core.render import canvas

# Explicitly define I2C interface and OLED display
serial = i2c(port=1, address=0x3c)
device = ssd1306(serial, width=128, height=32)

# Get CPU temperature from the system file.
# The file returns the temperature in millidegrees Celsius.
def get_cpu_temp():
    try:
        with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
            temp_str = f.read().strip()
        # Convert to degrees Celsius (float) with one decimal place.
        return round(float(temp_str) / 1000.0, 1)
    except Exception:
        return "??"

# Calculate CPU usage from /proc/stat.
# Returns current usage percentage along with updated idle/total counters.
def get_cpu_percent(prev_idle, prev_total):
    try:
        with open('/proc/stat', 'r') as f:
            fields = f.readline().strip().split()[1:]
            fields = list(map(int, fields))
            idle = fields[3]
            total = sum(fields)
        idle_delta = idle - prev_idle
        total_delta = total - prev_total
        usage = 100.0 * (1.0 - idle_delta / total_delta) if total_delta else 0
        return usage, idle, total
    except Exception:
        return 0, prev_idle, prev_total

# Get memory usage percentage from /proc/meminfo.
def get_mem_percent():
    try:
        with open('/proc/meminfo', 'r') as f:
            lines = f.readlines()
            mem_total = int(lines[0].split()[1])
            mem_available = int(lines[2].split()[1])
        used = mem_total - mem_available
        return round((used / mem_total) * 100)
    except Exception:
        return 0

# Get disk (SD) usage as a percentage using os.statvfs.
def get_disk_usage(path="/"):
    try:
        st = os.statvfs(path)
        total = st.f_blocks * st.f_frsize
        free = st.f_bfree * st.f_frsize
        used_pct = 100 - int((free / total) * 100)
        return f"{used_pct}%"
    except Exception:
        return "--"

# Get the IP address using a socket.
def get_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "?.?.?.?"

# Check for undervoltage using vcgencmd get_throttled.
def get_undervoltage():
    throttled = subprocess.getoutput("vcgencmd get_throttled | cut -d= -f2").strip()
    return throttled != "0x0"

# Draw the sensor data to the OLED display.
def system_info(device, draw, cpu_temp, ip, cpu_usage, mem_percent, disk_usage, voltage_text):
    x = 2  # Left margin for drawing text
    draw.rectangle((x, 0, 128, 32), fill="black")
    
    # Line 1: CPU temperature and CPU usage.
    draw.text((x, 0), 'CPU:', fill="white")
    draw.text((36, 0), f"{cpu_temp}°C", fill="white")
    draw.text((82, 0), f"{int(cpu_usage)}%", fill="white")
    
    # Line 2: IP address and voltage status text.
    draw.text((x, 10), 'IP:', fill="white")
    draw.text((20, 10), ip, fill="white")
    draw.text((90, 10), voltage_text, fill="white")
    
    # Line 3: SD usage and memory usage percentage.
    draw.text((x, 20), 'SD:', fill="white")
    draw.text((36, 20), disk_usage, fill="white")
    draw.text((70, 20), 'MEM:', fill="white")
    draw.text((104, 20), f"{mem_percent}%", fill="white")
    
    draw.rectangle(device.bounding_box, outline="white")

# -----------------------
# Initialization of timers and cached sensor values
# -----------------------
prev_idle, prev_total = 0, 0
blink_state = True  # For blinking the voltage display

last_general_update = 0       # For CPU usage, memory, undervoltage (1 Hz)
last_cpu_temp_update = 0        # For CPU temperature (5 seconds)
last_ip_update = 0              # For IP address (10 seconds)
last_disk_update = 0            # For disk (SD) usage (10 seconds)

general_interval = 1      # seconds
cpu_temp_interval = 5     # seconds
ip_interval = 10          # seconds
disk_interval = 10        # seconds
blink_interval = 0.2       # seconds (for 5 Hz blinking)

# Cached sensor values
cpu_temp = "??"
ip_cache = "?.?.?.?"
cpu_usage = 0
mem_percent = 0
disk_usage = "--"
undervoltage_active = False

# -----------------------
# Main Loop
# -----------------------
while True:
    current_time = time.time()

    # Update general sensor data (CPU usage, mem, undervoltage) every 1 second.
    if current_time - last_general_update >= general_interval:
        mem_percent = get_mem_percent()
        cpu_usage, prev_idle, prev_total = get_cpu_percent(prev_idle, prev_total)
        undervoltage_active = get_undervoltage()
        last_general_update = current_time

    # Update CPU temperature every 5 seconds.
    if current_time - last_cpu_temp_update >= cpu_temp_interval:
        cpu_temp = get_cpu_temp()
        last_cpu_temp_update = current_time

    # Update IP address every 10 seconds.
    if current_time - last_ip_update >= ip_interval:
        ip_cache = get_ip()
        last_ip_update = current_time

    # Update disk usage every 10 seconds.
    if current_time - last_disk_update >= disk_interval:
        disk_usage = get_disk_usage("/")
        last_disk_update = current_time

    # Determine the voltage display text for blinking.
    if undervoltage_active:
        if blink_state:
            voltage_text = "V: LOW"
        else:
            voltage_text = "       "  # Blank during off phase.
        blink_state = not blink_state
    else:
        voltage_text = "V: OK "
        blink_state = True

    # Render the display using cached values.
    with canvas(device) as draw:
        system_info(device, draw, cpu_temp, ip_cache, cpu_usage, mem_percent, disk_usage, voltage_text)
    
    time.sleep(blink_interval)
