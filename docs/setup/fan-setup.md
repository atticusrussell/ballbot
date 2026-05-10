# Fan Control Setup (52Pi Fan Expansion Board Plus)

This guide explains how to set up the fan control for the 52Pi Fan Expansion Board Plus on your Raspberry Pi. The fan control automatically adjusts based on the CPU temperature.

References:
- [Official Wiki Page (Fan section)](https://wiki.52pi.com/index.php?title=EP-0152#How_to_control_the_fan)
- [GeeekPi RPiFEBP GitHub Repo](https://github.com/geeekpi/RPiFEBP)

---

## Requirements

This guide assumes you have cloned your repository (e.g., `ballbot`) directly onto your Raspberry Pi:

```bash
git clone https://github.com/atticusrussell/ballbot.git
cd ballbot
```

---

## Installation

The installation is performed in one single step by running the provided setup script.

### Step: Run the Combined Setup Script

```bash
cd setup/scripts
sudo ./setup-fan.sh
```

This script will:

- Install the required system dependencies (`python3`, `python3-pip`, `python3-rpi.gpio`).
- Copy the fan control Python script to `/opt/ballbot`.
- Install and start the systemd service for fan control.

After running the script, the fan control will automatically monitor the CPU temperature using the script `/opt/ballbot/fan_control.py`:
- **≥ 45°C:** Fan at 100% duty cycle  
- **≥ 40°C:** Fan at 85% duty cycle  
- **Below 40°C:** Fan off

---

## Customization

If you wish to modify the behavior (e.g., adjust temperature thresholds or change the GPIO pin), edit the Python file directly:

```bash
sudo nano /opt/ballbot/fan_control.py
```

After making any changes, restart the service to apply them:

```bash
sudo systemctl restart fan.service
```

To view logs for troubleshooting:

```bash
journalctl -u fan.service -e
```

---

## Result

Once installed, your fan control service will adjust the fan speed based on the CPU temperature, ensuring efficient cooling with minimal system overhead.
