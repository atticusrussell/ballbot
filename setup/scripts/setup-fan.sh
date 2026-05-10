#!/bin/bash
set -e

echo "Updating package list and installing fan control dependencies..."
sudo apt-get update
sudo apt-get install -y python3 python3-pip python3-rpi.gpio

echo "Adding current user to gpio group if it exists..."
if getent group gpio > /dev/null; then
    sudo usermod -a -G gpio $USER
    echo "User $USER added to gpio group."
else
    echo "Warning: 'gpio' group not found. GPIO access might require manual setup."
fi

# Determine repo root relative to this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(realpath "$SCRIPT_DIR/../..")"  # Assumes script is in setup/scripts

INSTALL_DIR="/opt/ballbot"
FAN_SCRIPT_SRC="$REPO_ROOT/setup/python_scripts/fan_control.py"
FAN_SCRIPT_DEST="$INSTALL_DIR/fan_control.py"
SERVICE_FILE_SRC="$REPO_ROOT/setup/systemd_files/fan.service"
SERVICE_FILE_DEST="/etc/systemd/system/fan.service"

echo "Ensuring target install directory exists: $INSTALL_DIR"
sudo mkdir -p "$INSTALL_DIR"
sudo chown $USER:$USER "$INSTALL_DIR"

echo "Copying fan control script to $INSTALL_DIR..."
sudo cp "$FAN_SCRIPT_SRC" "$FAN_SCRIPT_DEST"
sudo chmod +x "$FAN_SCRIPT_DEST"

echo "Installing systemd service for fan control..."
sudo cp "$SERVICE_FILE_SRC" "$SERVICE_FILE_DEST"

echo "Reloading systemd daemon..."
sudo systemctl daemon-reload

echo "Enabling and restarting fan control service..."
sudo systemctl enable fan.service
sudo systemctl restart fan.service

echo "Fan control setup complete. Service 'fan.service' enabled and started."
echo "Check status with: sudo systemctl status fan.service"
echo "Check logs with: journalctl -u fan.service -e"
echo "IMPORTANT: You may need to log out and log back in or reboot for the group membership changes to take effect."
