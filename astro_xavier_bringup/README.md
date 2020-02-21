# astro_xavier_bringup


## Installation

    cp config/*.service ~/.config/systemd/user/
    cp config/lightdm.conf /etc/lightdm/lightdm.conf

## Other System Configuration

### On Fetch

Configure connection sharing. Recommend using iptables-persist

### On Xavier

    # Disable software update popups
    apt-get remove update-notifier

    # Modify or comment out NVidia login logo
    vi .xsessionrc
    
    # Prepare ros user
    sudo adduser --system --no-create-home --group ros
    sudo chsh ros /bin/bash
    # Recommended location for "ros" user's workspace, if any
    sudo mkdir -p /var/ros
    # Where robot.log will live
    sudo mkdir -p /var/log/ros
    # Where robot.launch should go
    sudo mkdir -p /etc/ros/melodic