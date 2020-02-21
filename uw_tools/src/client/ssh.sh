#!/usr/bin/env bash
###
# ssh_robot and helper aliases
###

function ssh_robot_usage {
cat <<- EOF
usage: ssh_robot remote_path [remote_user]
Opens an SSH session with the currently activated robot.
Configures X-forwarding, SSH agent forwarding, environment variables and more.
Options:
    -c --via-cs cs_username        Set the CS username to proxy the connection with
    -d --dettached                 Do not automatically enter a tmux session
    -h --help                      Print this message
    -o --hostname                  Set the hostname to connect to
    -n --new-session               Enter a new session
    -r --remote-hostname           Use the robot's remote hostname
    -s --session session           Name of the session to connect to
    -u --user username             Set the username to connect as
    -v --via username hostname     Set the username and hostname to use for proxying the connection
Notes:
If \$UW_REMOTE_WORKSPACE is set, remote_path will automatically
point to "~/workspaces/\$UW_REMOTE_WORKSPACE"
EOF
    return 1
}

function ssh-robot {
    local key_path="$HOME/.ssh/$ROBOT_KEYNAME"
    if [[ ! -f $key_path ]]; then
        2>&1 echo "$key_path doesn't exist. Please symlink your desired SSH key to the path, or generate a new one"
        return 1
    fi
    local user=$ROBOT_USERNAME
    local hostname="$ROBOT_HOSTNAME"
    local proxy_host=""
    local args=""
    local session_name="$(whoami)"
    local on_entrance="start_tmux $session_name false"
    local use_proxy="false"
    local proxy_username=""
    local proxy_host=""
    local proxy_cmd=""
    while [[ $# -gt 0 ]]; do
        local key="$1"
        case $key in
            -u|--user)
                user="$2"
                shift
            ;;
            -h|--help)
                ssh_robot_usage
            ;;
            -o|--hostname)
                hostname="$2"
                shift
            ;;
            -v|--via)
                use_proxy="true"
                proxy_username="$2"
                proxy_host="$3"
                shift
                shift
            ;;
            -r|--remote-hostname)
                hostname="$ROBOT_REMOTE_HOSTNAME"
            ;;
            -n|--new-session)
                on_entrance="start_tmux $session_name true"
            ;;
            -s|--session)
                session_name="$2"
                on_entrance="connect_tmux $session_name"
                shift
            ;;
            -d|--dettached)
                on_entrance="\$SHELL"
            ;;
            --)
                # Get rid of --
                shift
                # The remainder are grabbag args to pass to SSH
                args="${args}$@"
                break
            ;;
            *)
               >&2 echo "Unknown argument: $1"
               return 1
            ;;
        esac
        shift # move past argument
    done

    if [[ "$use_proxy" == "true" ]]; then
        proxy_cmd="-J $proxy_username@$proxy_host"
        if [ -z "$args" ]; then
            args=$proxy_cmd
        else
            args="$args $proxy_cmd"
        fi
    fi

    # These vars must also be added to the update-environment part of .tmux.conf
    export LC_USER_KEYNAME="$ROBOT_KEYNAME"
    export LC_USER_NAME="$(git config user.name)"
    export LC_USER_EMAIL="$(git config user.email)"
    export LC_USER="$USER"
    export LC_USER_WORKSPACE="$UW_REMOTE_WORKSPACE"
    # X forwarding, agent forwarding, force configured SSH key
    ssh -X \
        -A \
        -o SendEnv=LC_USER_KEYNAME \
        -o SendEnv=LC_USER_NAME \
        -o SendEnv=LC_USER_EMAIL \
        -o SendEnv=LC_USER_WORKSPACE \
        -o SendEnv=LC_USER \
        -o TCPKeepAlive=yes \
        -o ServerAliveInterval=60 \
        -o IdentitiesOnly=yes \
        -i $key_path \
        -t \
        $args \
        "$user@$hostname" \
        "$(typeset -f connect_tmux); $(typeset -f start_tmux); $on_entrance"
}

function ssh-xavier {
    ssh-robot -v "$ROBOT_USERNAME" "$ROBOT_HOSTNAME" -u robocup -o "$EXTERNAL_COMPUTER_HOSTNAME" "$@"
}


# arg1: name of session. Connects to the name if it exists. Uses it as a prefix to a new session if it doesn't
# arg2: bool - whether to create unique name (i.e. arg1-number)
function start_tmux {
    local name="$1"
    local unique="$2"
    # Is there a session with the passed name?
    # Unfortunately `tmux has-session` returns true if the passed string is a prefix to any existing session
    if [[ "$unique" == "false" ]]; then
        # Try to attach to the default session for this name
        tmux attach -t "$name-1" &> /dev/null || tmux new -s "$name-1"
    else
        local count=1
        while tmux ls | grep -q "${name}-${count}:"; do
            ((count += 1))
        done
        tmux new -s "$name-$count"
    fi
}

function connect_tmux {
    local name="$1"
    tmux attach -t "$name" &> /dev/null || tmux new -s "$name"
}