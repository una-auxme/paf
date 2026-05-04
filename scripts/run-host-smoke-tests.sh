#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." &>/dev/null && pwd)"

exec "$REPO_ROOT/scripts/bootstrap-host-python.sh" \
  python -m pytest code/test -m unit "$@"
