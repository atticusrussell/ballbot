#!/bin/bash

set -e

echo "Killing unattended-upgrade processes..."
ps aux | grep '[u]nattended-upgrade' | awk '{print $2}' | xargs -r sudo kill || true

echo "Disabling and removing unattended-upgrades..."
sudo systemctl disable --now unattended-upgrades || true
sudo apt remove -y unattended-upgrades || true
sudo apt purge -y unattended-upgrades || true

echo "Disabling APT periodic updates..."
sudo tee /etc/apt/apt.conf.d/10periodic > /dev/null <<EOF
APT::Periodic::Enable "0";
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Download-Upgradeable-Packages "0";
APT::Periodic::AutocleanInterval "0";
EOF

echo "Checking SSH status..."
sudo systemctl enable ssh
sudo systemctl start ssh

echo "Installing and enabling Avahi daemon..."
sudo apt update
sudo apt install -y avahi-daemon
sudo systemctl enable --now avahi-daemon

echo "Setting hostname to rosbot..."
sudo hostnamectl set-hostname rosbot

echo "Post Wi-Fi setup complete!"
