# OLED Display Setup (0.91 inch)

This guide walks through the process of enabling and configuring the OLED display on the 52Pi Fan Expansion Board Plus for the Raspberry Pi.

References:
- [Official Wiki Page (OLED section)](https://wiki.52pi.com/index.php?title=EP-0152#How_to_display_system_information_on_OLED_display)
- [GeeekPi RPiFEBP GitHub Repo](https://github.com/geeekpi/RPiFEBP)

---

## Requirements

This setup assumes you have cloned this repository directly onto your Raspberry Pi:

```bash
git clone https://github.com/atticusrussell/brobot.git
cd brobot
```

---

## Installation

Installation is split into three steps.

### Step 1: Install system dependencies (requires `sudo`)

This will install required system packages, including `raspi-config`, `i2c-tools`, and development libraries.

```bash
cd setup/scripts
sudo ./install-oled-dependencies.sh
```

### Step 2: Enable I2C (manual step)

Once `raspi-config` is installed, enable the I2C interface:

```bash
sudo raspi-config
# Navigate to: Interface Options -> I2C -> Enable -> YES
```

Then reboot or log out and back in so the `i2c` group membership takes effect.

---

### Step 3: Set up OLED environment (requires `sudo`)

This script sets up the Python environment and systemd service:

```bash
sudo ./setup-oled.sh
```

This script will:

- Create a virtual environment in `~/luma-env`
- Install `luma.oled` and `psutil`
- Copy the custom OLED display script
- Register and start a systemd service to run the display on boot

---

## Customization

Once installed, the active OLED script on your robot is located at:

```bash
~/luma-env/system_infor.py
```

You can edit this file directly to change what’s displayed. For example:

```bash
nano ~/luma-env/system_infor.py
```

After making changes, restart the service to apply them:

```bash
sudo systemctl restart oled.service
```

To check the service logs for errors:

```bash
journalctl -u oled.service -e
```

---

## Result

After setup and reboot, your OLED should display:

- CPU temperature
- IP address
- Memory usage
- Disk usage

You can customize these metrics or add your own.
