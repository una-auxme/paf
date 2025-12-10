#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
REPO_ROOT=$(dirname "$SCRIPT_DIR")
DOTENV_FILE="${REPO_ROOT}/build/.env"
RUFF_VERSION_FILE="${REPO_ROOT}/build/pins/ruff.env"
mkdir -p "$(dirname "$DOTENV_FILE")"

PAF_USERNAME=$(id -u -n)
PAF_UID=$(id -u)
PAF_GID=$(id -g)
RUFF_VERSION=""

if [ -f "${RUFF_VERSION_FILE}" ]; then
  # shellcheck source=/dev/null
  source "${RUFF_VERSION_FILE}"
fi

cat >"$DOTENV_FILE" <<EOF
PAF_USERNAME=${PAF_USERNAME}
PAF_UID=${PAF_UID}
PAF_GID=${PAF_GID}
EOF

echo "Wrote $DOTENV_FILE with current user info:"
echo "PAF_USERNAME=${PAF_USERNAME}"
echo "PAF_UID=${PAF_UID}"
echo "PAF_GID=${PAF_GID}"

if [ -n "${RUFF_VERSION}" ]; then
  echo "RUFF_VERSION=${RUFF_VERSION}" >>"$DOTENV_FILE"
  echo "RUFF_VERSION=${RUFF_VERSION}"
else
  echo "Warning: RUFF_VERSION not found in ${RUFF_VERSION_FILE}; run scripts/update-dotenv.sh after adding build/pins/ruff.env to keep Compose and tooling in sync."
fi
