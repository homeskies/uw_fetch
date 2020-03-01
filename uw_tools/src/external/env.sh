export ROS_HOSTNAME=$EXTERNAL_COMPUTER_HOSTNAME
export ROS_MASTER_URI=http://$ROBOT_HOSTNAME:11311
# Hardcoded to match Xavier's network configuration
ip_address="$(get_ip eth0)"
return_code=$?
if [[ "$return_code" != "0" ]] ; then
    echo "[UW Tools] ROS_IP is not set."
else
    export ROS_IP="$ip_address"
fi
export ROS_HOME=~/.ros
