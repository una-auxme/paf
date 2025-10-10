#!/usr/bin/env bash
set -euo pipefail

DOTENV_FILE="${PWD}/build/.env"

PAF_USERNAME=$(id -u -n)
PAF_UID=$(id -u)
PAF_GID=$(id -g)

cat > "$DOTENV_FILE" <<EOF
PAF_USERNAME=${PAF_USERNAME}
PAF_UID=${PAF_UID}
PAF_GID=${PAF_GID}
EOF

echo "Wrote $DOTENV_FILE with current user info:"
echo "PAF_USERNAME=${PAF_USERNAME}"
echo "PAF_UID=${PAF_UID}"
echo "PAF_GID=${PAF_GID}"
