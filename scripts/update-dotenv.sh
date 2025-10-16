#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
REPO_ROOT=$(dirname "$SCRIPT_DIR")
DOTENV_FILE="${REPO_ROOT}/build/.env"
mkdir -p "$(dirname "$DOTENV_FILE")"

PAF_USERNAME=$(id -u -n)
PAF_UID=$(id -u)
PAF_GID=$(id -g)

cat >"$DOTENV_FILE" <<EOF
PAF_USERNAME=${PAF_USERNAME}
PAF_UID=${PAF_UID}
PAF_GID=${PAF_GID}
EOF

echo "Wrote $DOTENV_FILE with current user info:"
echo "PAF_USERNAME=${PAF_USERNAME}"
echo "PAF_UID=${PAF_UID}"
echo "PAF_GID=${PAF_GID}"
