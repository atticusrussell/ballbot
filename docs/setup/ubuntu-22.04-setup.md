# Ubuntu Server 22.04.3 Setup for Robot

This guide walks through the initial setup of a fresh Ubuntu Server 22.04.3 install.

---

## 1. Disable cloud-init

```bash
sudo apt purge cloud-init && sudo rm -rf /etc/cloud/
sudo touch /etc/cloud/cloud-init.disabled
```

---

## 2. Set up Wi-Fi using Netplan

Create or edit `/etc/netplan/01-netcfg.yaml`:

```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: true
  wifis:
    wlan0:
      dhcp4: true
      access-points:
        "YourSSID":
          password: "YourPassword"
```

Then run:

```bash
sudo systemctl enable --now systemd-networkd
sudo systemctl enable --now systemd-resolved

sudo netplan generate
sudo netplan apply
sudo reboot
```

---

## 3. Post-WiFi Setup

After confirming that Wi-Fi works and you've rebooted, you can run a script to complete the rest of the system configuration automatically.

### 📥 Download the script

You can download the script directly to your robot:

```bash
curl -O https://raw.githubusercontent.com/atticusrussell/brobot/main/scripts/setup/post-wifi-setup.sh
chmod +x post-wifi-setup.sh
```

> **Tip:** Always review downloaded scripts before running them with elevated privileges.

### Run the script with sudo

```bash
sudo ./post-wifi-setup.sh
```

This will:
- Remove unattended-upgrades
- Disable periodic APT auto-updates
- Ensure SSH is enabled
- Install Avahi daemon for `.local` hostname access
- Set hostname to `rosbot`

---

## 4. Passwordless SSH Login (From Your Laptop)

### 4.1 Check if you already have an SSH key

```bash
ls ~/.ssh/id_rsa.pub
```

If not:

```bash
ssh-keygen -t rsa -b 4096 -C "your@email.com"
```

### 4.2 Copy your key to the robot

```bash
ssh-copy-id ubuntu@rosbot.local
```

You should now be able to log in without a password:

```bash
ssh ubuntu@rosbot.local
```
