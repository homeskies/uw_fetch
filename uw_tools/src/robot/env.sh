export ROS_HOSTNAME=$ROBOT_HOSTNAME
ip_address="$(get_ip wlan0)"
return_code=$?
if [[ "$return_code" != "0" ]] ; then
    echo "[UW Tools] ROS_IP is not set."
else
    export ROS_IP="$ip_address"
fi
export ROS_HOME=~/.ros
