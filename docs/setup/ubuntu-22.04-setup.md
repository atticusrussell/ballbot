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
      optional: true
  wifis:
    wlan0:
      dhcp4: true
      optional: true
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
curl -O https://raw.githubusercontent.com/atticusrussell/ballbot/main/setup/scripts/post-wifi-setup.sh
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

### 3.1 Avahi configuration (optional)
tweak a few settings to avoid hostname changing
- manually specify `host-name`
- disable `check-response-ttl`
- 

in `/etc/avahi/avahi-daemon.conf
``` ini
[server]
host-name=rospi
#domain-name=local
#browse-domains=0pointer.de, zeroconf.org
use-ipv4=yes
use-ipv6=yes
#allow-interfaces=eth0
#deny-interfaces=eth1
check-response-ttl=no
#use-iff-running=no
#enable-dbus=yes
#disallow-other-stacks=no
#allow-point-to-point=no
#cache-entries-max=4096
#clients-max=4096
#objects-per-client-max=1024
#entries-per-entry-group-max=32
ratelimit-interval-usec=1000000
ratelimit-burst=1000

[wide-area]
enable-wide-area=yes

[publish]
#disable-publishing=no
#disable-user-service-publishing=no
#add-service-cookie=no
#publish-addresses=yes
publish-hinfo=yes
publish-workstation=yes
#publish-domain=yes
#publish-dns-servers=192.168.50.1, 192.168.50.2
#publish-resolv-conf-dns-servers=yes
#publish-aaaa-on-ipv4=yes
#publish-a-on-ipv6=no

[reflector]
#enable-reflector=no
#reflect-ipv=no
#reflect-filters=_airplay._tcp.local,_raop._tcp.local

[rlimits]
#rlimit-as=
#rlimit-core=0
#rlimit-data=8388608
#rlimit-fsize=0
#rlimit-nofile=768
#rlimit-stack=8388608
#rlimit-nproc=3
```

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

## 5. ROS Humble Installation
Clone the ROS setup scripts repo on the robot, and run the humble setup script

```bash 
git clone https://github.com/Tiryoh/ros2_setup_scripts_ubuntu.git
cd ros2_setup_scripts_ubuntu
./ros2-humble-ros-base-main.sh
<enter sudo password>
```

## 6. Docker Installation

https://docs.docker.com/engine/install/ubuntu/

Before you install Docker Engine for the first time on a new host machine, you
need to set up the Docker `apt` repository. Afterward, you can install and update
Docker from the repository.

1. Set up Docker's `apt` repository.

   ```bash
   # Add Docker's official GPG key:
   sudo apt-get update
   sudo apt-get install ca-certificates curl
   sudo install -m 0755 -d /etc/apt/keyrings
   sudo curl -fsSL {{% param "download-url-base" %}}/gpg -o /etc/apt/keyrings/docker.asc
   sudo chmod a+r /etc/apt/keyrings/docker.asc

   # Add the repository to Apt sources:
   echo \
     "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] {{% param "download-url-base" %}} \
     $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | \
     sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
   sudo apt-get update
   ```

2. Install the Docker packages.


   To install the latest version, run:

   ```console
   $ sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
   ```


3. Verify that the installation is successful by running the `hello-world` image:

   ```console
   $ sudo docker run hello-world
   ```

   This command downloads a test image and runs it in a container. When the
   container runs, it prints a confirmation message and exits.

  You have now successfully installed and started Docker Engine.

### 6.1 Add your user to the docker group (optional)
So you don't need sudo every time:

```bash
sudo usermod -aG docker $USER
newgrp docker
```
Log out and back in if docker still needs sudo.


## 7. SSH helper (optional)

append the following to your `.bashrc` or `.bash_profile` on the pi

```bash
# Start ssh-agent if not already running
if ! pgrep -u "$USER" ssh-agent > /dev/null; then
    eval "$(ssh-agent -s)"
fi

# Automatically add your key (if not already added)
SSH_KEY="$HOME/.ssh/id_ed25519"
if [ -f "$SSH_KEY" ] && ! ssh-add -l | grep -q "$SSH_KEY"; then
    ssh-add "$SSH_KEY" > /dev/null 2>&1
fi
```