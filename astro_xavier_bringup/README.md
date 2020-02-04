# astro_xavier_bringup


## Installation

    cp config/*.service ~/.config/systemd/user/
    cp config/lightdm.conf /etc/lightdm/lightdm.conf

## Other System Configuration


    # Disable software update popups
    apt-get remove update-notifier

    # Modify or comment out NVidia login logo
    vi .xsessionrc