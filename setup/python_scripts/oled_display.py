#!/usr/bin/env python3

import time
import subprocess
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from luma.core.render import canvas

# Explicitly define I2C interface and OLED display
serial = i2c(port=1, address=0x3c)
device = ssd1306(serial, width=128, height=32)

def system_info(device, draw):
    x = 2  # Left margin for drawing text

    # Get CPU temperature using vcgencmd.
    # Example output: "temp=48.0'C"
    # sed removes everything except numbers, decimal point, and optional '*'
    temp = subprocess.getoutput("vcgencmd measure_temp | sed 's/[^0-9.*]//g'")

    # Get core voltage using vcgencmd.
    # Example output: "volt=0.8500V"
    # sed removes everything except digits and decimal point
    voltage = subprocess.getoutput("vcgencmd measure_volts core | sed 's/[^0-9.]//g'")

    # Check for undervoltage using vcgencmd get_throttled
    # If bit 0 or bit 16 is set in the hex output, it indicates undervoltage now or in the past
    throttled = subprocess.getoutput("vcgencmd get_throttled | cut -d= -f2")
    undervoltage = "OK" if throttled == "0x0" else "LOW"

    # Get disk usage of root (/) filesystem
    # df -Th shows disk usage with filesystem type
    # awk '{print $6}' grabs the 6th column (Use%)
    # tail -n1 skips the header and gets the actual data
    disk_usage = subprocess.getoutput("df -Th / | awk '{print $6}' | tail -n1")

    # Get memory usage percentage
    # free -m shows memory in MB; grep selects 'Mem' line
    # awk calculates (used / total) * 100
    mem_percent = subprocess.getoutput(
        "free -m | grep Mem | awk '{printf \"%.0f\", ($3 / $2) * 100}'"
    )

    # Get IP address
    # hostname -I shows all IPs assigned
    # awk '{print $1}' selects the first one (usually eth0 or wlan0)
    IP = subprocess.getoutput("hostname -I | awk '{print $1}'")

    # Clear display and set background to black
    draw.rectangle((x, 0, 128, 32), fill="black")

    # Line 1: CPU Temp and Voltage
    draw.text((x, 0), 'CPU:', fill="white")
    draw.text((36, 0), temp + '°C', fill="white")
    draw.text((82, 0), 'V:', fill="white")
    draw.text((96, 0), undervoltage, fill="white")

    # Line 2: IP Address
    draw.text((x, 10), 'IP:', fill="white")
    draw.text((36, 10), IP, fill="white")

    # Line 3: SD Card usage and Free Memory
    draw.text((x, 20), 'SD:', fill="white")
    draw.text((36, 20), disk_usage, fill="white")
    draw.text((70, 20), 'MEM:', fill="white")
    draw.text((104, 20), mem_percent + '%', fill="white")

    # Draw border
    draw.rectangle(device.bounding_box, outline="white")

while True:
    with canvas(device) as draw:
        system_info(device, draw)
        time.sleep(2)
