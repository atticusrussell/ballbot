#!/bin/bash

set -e

echo "Installing essential system dependencies..."
sudo apt-get update
sudo apt-get install -y \
  python3 \
  python3-pip \
  python3-venv \
  i2c-tools \
  raspi-config

echo "Adding current user to the i2c group..."
sudo usermod -a -G i2c $USER

echo "Dependencies installed."
echo "You must now enable I2C manually using: sudo raspi-config"
echo "Then reboot or log out and back in before continuing."
