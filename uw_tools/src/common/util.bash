# Networking

###
# Pulls the IP address for a given network interface
##
function get_ip(){
  if [[ $# != 1 ]]; then
      echo "Usage: $0 network_if"
      return 1
  fi
  network_if="$1"

  target_ip=$(LANG=C ip addr show $network_if | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*') &> /dev/null
  # We may get degenerate strings back. Filter them out
  if [[ "${#target_ip}" -le 5 ]] ; then
      return 1
  else
      echo $target_ip
      return 0
  fi
}

function set_rosip {
    ip_address="$(get_ip $1)"
    return_code=$?
    if [[ "$return_code" != "0" ]] ; then
        >&2 echo "[UW Tools] Could not get IP address from network interface eno1. Will not set ROS_IP. Use set_network_if to configure which network interface to use with ROS"
        return 1
    else
        export ROS_IP="$ip_address"
    fi
}