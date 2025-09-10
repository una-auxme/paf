#!/usr/bin/env bash
set -euo pipefail

#!/usr/bin/env bash
set -euo pipefail

# Minimum required NVIDIA driver major version
REQUIRED=550

# Absolute path to this script (for problem matcher file reference)
SCRIPT_PATH=$(readlink -f "$0" 2>/dev/null || echo "$0")

diag_warn() {
  # GCC-like diagnostic for VS Code problem matchers
  echo "${SCRIPT_PATH}:1:1: warning: $1"
}

diag_error() {
  # GCC-like diagnostic for VS Code problem matchers
  echo "${SCRIPT_PATH}:1:1: error: $1"
}

have_smi() {
  command -v nvidia-smi >/dev/null 2>&1
}

version_ge() {
  # Compare two semantic-like versions (supports dotted, but we use numeric)
  # Returns 0 if $1 >= $2
  [ "$(printf '%s\n' "$2" "$1" | sort -V | head -n1)" = "$2" ]
}

if ! have_smi; then
  # Likely an Intel/AMD host or NVIDIA not installed; don't create a Problem entry.
  echo "[paf] NVIDIA check: nvidia-smi not found. Skipping check (only required for CUDA workflows)."
  exit 0
fi

DRV=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -n1 | tr -d '[:space:]')
if [ -z "$DRV" ]; then
  echo "[paf] NVIDIA check: could not determine driver version via nvidia-smi. Skipping."
  exit 0
fi

# Extract major version (before first dot)
DRV_MAJOR=${DRV%%.*}

if version_ge "$DRV_MAJOR" "$REQUIRED"; then
  echo "[paf] NVIDIA driver OK: $DRV (>= $REQUIRED)"
  exit 0
else
  MSG="NVIDIA driver too old: $DRV (< $REQUIRED). Please upgrade to R$REQUIRED+."
  echo "[paf] $MSG" >&2
  # Use warning severity to surface in Problems view without failing the task.
  diag_warn "$MSG"
  exit 0
fi
