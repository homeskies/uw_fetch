# generated from uw_tools/env-hooks/uw_tools.bash.em

@[if DEVELSPACE]@
export SOURCES_ROOT="@(CMAKE_CURRENT_SOURCE_DIR)/src"
export ON_ROBOT=$(if [[ "$(hostname)" == "$ROBOT_NAME" ]]; then echo "true"; else echo "false"; fi)
if [[ "$ON_ROBOT" == "true" ]]; then
    export PATH="@(CMAKE_CURRENT_SOURCE_DIR)/scripts/available_robot/:$PATH"
    . "@(CMAKE_CURRENT_SOURCE_DIR)/scripts/uw_tools_robot"
else
    export PATH="@(CMAKE_CURRENT_SOURCE_DIR)/scripts/available_client/:$PATH"
    . "@(CMAKE_CURRENT_SOURCE_DIR)/scripts/uw_tools_client"
fi
@[else]@
# TODO(nickswalker): Add install space support
@[end if]@
