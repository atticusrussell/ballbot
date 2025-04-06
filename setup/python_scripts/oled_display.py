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
    x = 2
    temp = subprocess.getoutput("vcgencmd measure_temp | sed 's/[^0-9.*]//g'")
    disk_usage = subprocess.getoutput("df -Th / | awk '{print $6}' | tail -n1")
    free_mem = subprocess.getoutput("free -g | grep Mem | awk '{print $4}'")
    IP = subprocess.getoutput("hostname -I | awk '{print $1}'")

    draw.rectangle((x, 0, 128, 32), fill="black")
    draw.text((x, 0), 'CPU TEMP:', fill="white")
    draw.text((56, 0), temp, fill="white")
    draw.text((82, 0), '°C', fill="white")
    draw.text((x, 8), 'IP ADDR:', fill="white")
    draw.text((52, 8), IP, fill="white")
    draw.text((x, 16), 'SD USAGE:', fill="white")
    draw.text((58, 16), disk_usage, fill="white")
    draw.text((78, 16), 'MEM:', fill="white")
    draw.text((100, 16), free_mem, fill="white")
    draw.text((104, 16), ' GB', fill="white")
    draw.rectangle(device.bounding_box, outline="white")

while True:
    with canvas(device) as draw:
        system_info(device, draw)
        time.sleep(5)
