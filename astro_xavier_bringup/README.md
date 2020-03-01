# astro_xavier_bringup

Launch files and system configuration for the Xavier installed on Astro's head.

## Installation

    cp config/*.service ~/.config/systemd/user/
    cp config/lightdm.conf /etc/lightdm/lightdm.conf

## Other System Configuration

### On Fetch

iptables rules for connection sharing are stored in `uw_fetch_bringup`.

### On Xavier

    # Disable software update popups
    apt-get remove update-notifier

    # Modify or comment out NVidia login logo
    vi .xsessionrc
    
    # Prepare main user (for development)
    # Give access to sound devices
    sudo usermod -a -G dialout $USER
    
    # Prepare ros user (for default launch)
    sudo adduser --system --no-create-home --group ros
    sudo usermod -a -G dialout ros
    sudo chsh ros /bin/bash
    # Recommended location for "ros" user's workspace, if any
    sudo mkdir -p /var/ros
    # Where robot.log will live
    sudo mkdir -p /var/log/ros
    # Where robot.launch should go
    sudo mkdir -p /etc/ros/melodic