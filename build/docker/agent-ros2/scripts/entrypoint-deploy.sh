#!/bin/bash
set -e

_term() {
  echo "Caught SIGTERM signal!"
  # shellcheck disable=SC2046
  kill -TERM $(jobs -p) 2>/dev/null
}
trap _term SIGTERM SIGINT

echo "${PATH}" > "${INTERNAL_WORKSPACE_DIR}/default_PATH"

# Pull in the deploy bashrc
# It does not get automatically loaded, because the entrypoint is not an interactive shell
source "${INTERNAL_WORKSPACE_DIR}/deploy.bashrc"

cd "${INTERNAL_WORKSPACE_DIR}"

"$@" &
wait "$!"
