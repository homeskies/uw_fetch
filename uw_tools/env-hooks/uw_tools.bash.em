# generated from uw_tools/env-hooks/uw_tools.bash

@[if DEVELSPACE]@
export SOURCES_ROOT="@(CMAKE_CURRENT_SOURCE_DIR)/src"
. "@(CMAKE_CURRENT_SOURCE_DIR)/scripts/uw_tools_client"
@[else]@
if [ -z "$CATKIN_ENV_HOOK_WORKSPACE" ]; then
  CATKIN_ENV_HOOK_WORKSPACE="@(CMAKE_INSTALL_PREFIX)"
fi
export SOURCES_ROOT="$CATKIN_ENV_HOOK_WORKSPACE/src"
. "$CATKIN_ENV_HOOK_WORKSPACE/share/uw_tools/uw_tools"
@[end if]@
