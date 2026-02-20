#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
REPO_ROOT=$(dirname "$SCRIPT_DIR")

"${SCRIPT_DIR}/update-dotenv.sh"

if [[ -x "${SCRIPT_DIR}/check-nvidia.sh" ]]; then
  "${SCRIPT_DIR}/check-nvidia.sh"
fi

cd "${REPO_ROOT}"
docker compose -f build/docker-compose.dev.cuda.yml up -d agent-dev

echo "agent-dev is running."
echo "Open in container via VS Code: 'Dev Containers: Reopen in Container'."
