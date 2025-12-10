#!/bin/bash
set -e

SCRIPT_DIR="$(cd -- "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../../../../" && pwd)"
RUFF_VERSION_FILE="${REPO_ROOT}/build/pins/ruff.env"
if [ -f "${RUFF_VERSION_FILE}" ]; then
  # shellcheck source=/dev/null
  # Share Ruff version pin with CI/Compose via build/pins/ruff.env so every install uses
  # the same version (avoids mismatched lint results).
  source "${RUFF_VERSION_FILE}"
fi

if [ -z "${PAF_ROS_WS}" ]; then
  echo PAF_ROS_WS is not set
  exit 1
fi

cd "${PAF_ROS_WS}/src"

REQUIREMENTS=()

# Based on https://stackoverflow.com/questions/23356779/how-can-i-store-the-find-command-results-as-an-array-in-bash/54561526#54561526
readarray -d '' REQUIREMENT_FILES < <(find . -name requirements.txt -print0)
for i in "${REQUIREMENT_FILES[@]}"; do
  REQUIREMENTS+=(-r "$i")
  echo Found python requirements: "$i"
done

if [ -n "$BASE_FLAVOUR" ]; then
  # Only rocm and cuda flavours use special requirement files.
  if [ "$BASE_FLAVOUR" == "generic" ]; then
    BASE_FLAVOUR="cpu"
  elif [ "$BASE_FLAVOUR" == "gpu" ]; then
    BASE_FLAVOUR="cpu"
  fi

  readarray -d '' REQUIREMENT_FILES < <(find . -name requirements."$BASE_FLAVOUR".txt -print0)
  for i in "${REQUIREMENT_FILES[@]}"; do
    REQUIREMENTS+=(-r "$i")
    echo Found "$BASE_FLAVOUR" python requirements: "$i"
  done
fi

# Install infrastructure requirements
readarray -d '' REQUIREMENT_FILES < <(find . -name requirements_infrastructure.txt -print0)
for i in "${REQUIREMENT_FILES[@]}"; do
  REQUIREMENTS+=(-r "$i")
  echo Found infrastructure python requirements: "$i"
done

if [ ${#REQUIREMENTS[@]} -gt 0 ]; then
  pip_tmpdir=~/.pip-tmpdir-docker
  mkdir -p ${pip_tmpdir}

  RUFF_CONSTRAINT_FILE="${pip_tmpdir}/ruff-constraint.txt"
  if [ -n "${RUFF_VERSION}" ]; then
    # Constrain pip to the pinned Ruff version from build/pins/ruff.env so developers,
    # CI (docker-compose.linter), and container builds all see identical Ruff behavior.
    printf "ruff==%s\n" "${RUFF_VERSION}" > "${RUFF_CONSTRAINT_FILE}"
    REQUIREMENTS+=(--constraint "${RUFF_CONSTRAINT_FILE}")
  else
    echo "Warning: RUFF_VERSION is not set; Ruff will not be pinned."
  fi

  TMPDIR=${pip_tmpdir} pip install --user "${REQUIREMENTS[@]}"
  rm -rf ${pip_tmpdir}
else
  echo No python requirements found.
fi
