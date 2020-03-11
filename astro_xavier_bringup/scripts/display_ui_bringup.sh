#!/bin/bash

# We may want to wait until we know that RWS is up
#source /opt/ros/melodic/setup.bash
# until $(rosnode list 2> /dev/null | grep -q rosbridge_websocket)
#do
#    sleep 1
#done

# incognito: prevent profile unlocking popup
# password-store=basic: prevent keyring unlock popup
DISPLAY=:0 chromium-browser --kiosk --incognito --password-store=basic https://picsum.photos/1920/1200