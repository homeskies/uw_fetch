#!/usr/bin/env bash
export UW_TOOLS_MODE="none"
ROS_PROMPT_MASTER_UP="up"
ROS_PROMPT_MASTER_DOWN="down"
ROS_PROMPT_MASTER_OUT="timeout"
ROS_PROMPT_MASTER_UNEXPECTED="err"
ROS_PROMPT_USE_SIM_TIME="sim_time"
function get_ros_status() {
    ros_status=""
    LC_ALL=C curl -m 0.1 -s -o /dev/null \
    --request POST "$ROS_MASTER_URI/RPC2" --data "<?xml version='1.0'?>
    <methodCall><methodName>getUri</methodName><params><param><value><string>
    </string></value></param></params></methodCall>"
    case "$?" in
        0)
            # master up and responding
            [ "$(rosparam get use_sim_time 2>&1 | grep 'true')" ] && \
            ros_status+="$ROS_PROMPT_USE_SIM_TIME "
            ros_status+="$ROS_PROMPT_MASTER_UP"
        ;;
        7)
            # master down and machine responding
            ros_status+="$ROS_PROMPT_MASTER_DOWN"
        ;;
        6|28)
            # Could not resolve host
            # Connection timeout
            ros_status+="$ROS_PROMPT_MASTER_OUT"
        ;;
        *)
            # Unexpected
            ros_status+="$ROS_PROMPT_MASTER_UNEXPECTED[$?]"
        ;;
    esac
    echo "$ros_status"
}

function __prompt_command {
    UW_TOOLS_PROMPT_ROS_STATUS=$(get_ros_status)
}

function sim-mode {
    if [[ "$UW_TOOLS_MODE" == "none" ]]; then
        export original_PROMPT_COMMAND="$PROMPT_COMMAND"
        export original_PS1="$PS1"
        export original_ROS_MASTER_URI="$ROS_MASTER_URI"
        export original_ROS_IP="$ROS_IP"
    fi

    export UW_TOOLS_MODE=sim
    export ROS_MASTER_URI=http://localhost:11311
    set_rosip lo
    export UW_TOOLS_MODE="sim"
    PROMPT_COMMAND=__prompt_command
    PS1="\[\033[41;1;37m\]lcl \$UW_TOOLS_PROMPT_ROS_STATUS\[\033[0m\]\w "
}

function robot-mode {
    if [[ "$UW_TOOLS_MODE" == "none" ]]; then
        export original_PROMPT_COMMAND="$PROMPT_COMMAND"
        export original_PS1="$PS1"
        export original_ROS_MASTER_URI="$ROS_MASTER_URI"
        export original_ROS_IP="$ROS_IP"
    fi
    export ROS_MASTER_URI="http://$ROBOT_HOSTNAME:11311"
    set_rosip eno1
    export UW_TOOLS_MODE="robot"
    PROMPT_COMMAND=__prompt_command
    PS1="\[\033[44;1;37m\]\${ROBOT_NAME:0:3} \$UW_TOOLS_PROMPT_ROS_STATUS\[\033[0m\]\w "
}

function exit-mode {
    export ROS_MASTER_URI="$original_ROS_MASTER_URI"
    export ROS_IP="$original_ROS_IP"
    PS1="$original_PS1"
    PROMPT_COMMAND="$original_PROMPT_COMMAND"
    export UW_TOOLS_MODE="none"
}
