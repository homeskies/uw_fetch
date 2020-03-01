#!/usr/bin/env bash
###
# Welcome banner that shows in new shells
###
function _build_ssh_message {
    # If the user isn't connected via SSH, don't show anything
    if [[ -z "$SSH_CLIENT" || -z "$SSH_TTY" ]]; then
        return
    fi
    message=""
    # Clients will forward this variable to indicate which key they would like to use for Git and other purposes
    if [[ -n "$LC_USER_KEYNAME" ]]; then
        message="SSH key: $LC_USER_KEYNAME"
        if [[ ! -f ~/.ssh/$LC_USER_KEYNAME.pub ]]; then
            echo '[UW Tools] Warning: could not find your SSH key locally'
        fi

        if ! ssh-add -l &> /dev/null; then
            echo '[UW Tools] Warning: could not access ssh-agent. Signing actions may fail.'
        fi
    else
        message="SSH key: (none)"
    fi
    ssh_ip=$(echo "$SSH_CLIENT" | awk '{ print $1}')
    message=$message"\tClient IP: $ssh_ip"
    echo -e "$message"
}

function _build_workspace_message {
    # Clients will forward a preferred workspace.
    if [[ -n $LC_USER_WORKSPACE ]]; then
        echo -e "Workspace: $LC_USER_WORKSPACE"
        if [[ ! -d "$HOME/workspaces/$LC_USER_WORKSPACE" ]]; then
            echo '[UW Tools] Warning: could not find the configured workspace'
        fi
    else
        echo -e "Workspace: (none)"
    fi
}

# If the user is attached via SSH, show some extra info
ssh_message="$(_build_ssh_message)"
workspace_message="$(_build_workspace_message)"

echo -e "Robot IP: ${ROS_IP}"
echo -e "$ssh_message"
#echo -e $workspace_message