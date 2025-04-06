#!/bin/bash

set -e

INSTALL_DIR="/opt/brobot"
VENV_DIR="$INSTALL_DIR/venv"

echo "Creating target install directory: $INSTALL_DIR"
sudo mkdir -p "$INSTALL_DIR"
sudo chown $USER:$USER "$INSTALL_DIR"

echo "Creating Python virtual environment in $VENV_DIR..."
python3 -m venv "$VENV_DIR"
source "$VENV_DIR/bin/activate"

echo "Installing Python packages in virtual environment..."
pip install --upgrade pip
pip install luma.oled psutil

echo "Copying OLED display script to $INSTALL_DIR"
cp setup/python_scripts/oled_display.py "$INSTALL_DIR/"

echo "Installing systemd service..."
sudo cp setup/systemd_files/oled.service /etc/systemd/system/oled.service
sudo systemctl daemon-reload
sudo systemctl enable oled.service
sudo systemctl start oled.service

echo "OLED setup complete and service started."
