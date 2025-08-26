#!/bin/bash
# Sourced at the end of ~/.bashrc in the container
# Provides utility functions and sources the ros workspace

# Activate WSL gpu drivers if available
# https://github.com/microsoft/wslg/blob/main/samples/container/Containers.md#containerizing-gui-applications-with-wslg
if [ -d /usr/lib/wsl/lib ]; then
  export LD_LIBRARY_PATH=/usr/lib/wsl/lib:${LD_LIBRARY_PATH}
fi

source "${INTERNAL_WORKSPACE_DIR}/scripts/devfunctions.bash"

devsource

# Additional bashrc for customizations
if [ -f ~/user.bashrc ]; then
  source ~/user.bashrc
fi
