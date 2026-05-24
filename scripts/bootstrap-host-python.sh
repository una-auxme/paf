#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." &>/dev/null && pwd)"
VENV_DIR="${PAF_HOST_VENV:-$REPO_ROOT/.venv-host}"
STAMP_FILE="$VENV_DIR/.paf-host-deps.sha256"

if [ -n "${PAF_HOST_PYTHON:-}" ]; then
  PYTHON_BIN="$PAF_HOST_PYTHON"
elif [ -x /usr/bin/python3 ]; then
  PYTHON_BIN="/usr/bin/python3"
else
  PYTHON_BIN="python3"
fi

if command -v uv >/dev/null 2>&1; then
  UV_BIN="$(command -v uv)"
else
  UV_BIN=""
fi

deps_hash="$(
  {
    cat "$REPO_ROOT/code/requirements_infrastructure.txt"
    printf '\n# extra host smoke test dependency\nnumpy==1.26.4\n'
  } | sha256sum | cut -d' ' -f1
)"

create_virtualenv() {
  rm -rf "$VENV_DIR"
  if [ -n "$UV_BIN" ]; then
    "$UV_BIN" venv --python "$PYTHON_BIN" "$VENV_DIR"
    return
  fi

  "$PYTHON_BIN" -m venv "$VENV_DIR"
}

if [ ! -x "$VENV_DIR/bin/python" ] \
  || ! "$VENV_DIR/bin/python" -c "import encodings" >/dev/null 2>&1 \
  || { [ -z "$UV_BIN" ] && ! "$VENV_DIR/bin/python" -m pip --version >/dev/null 2>&1; }; then
  create_virtualenv
fi

if [ ! -f "$STAMP_FILE" ] || [ "$(cat "$STAMP_FILE")" != "$deps_hash" ]; then
  if [ -n "$UV_BIN" ]; then
    "$UV_BIN" pip install \
      --python "$VENV_DIR/bin/python" \
      -r "$REPO_ROOT/code/requirements_infrastructure.txt" \
      numpy==1.26.4
  else
    "$VENV_DIR/bin/python" -m pip install --upgrade pip
    "$VENV_DIR/bin/python" -m pip install \
      -r "$REPO_ROOT/code/requirements_infrastructure.txt" \
      numpy==1.26.4
  fi
  printf '%s\n' "$deps_hash" >"$STAMP_FILE"
fi

if [ "$#" -gt 0 ]; then
  export VIRTUAL_ENV="$VENV_DIR"
  export PATH="$VENV_DIR/bin:$PATH"
  exec "$@"
fi

printf 'Host Python environment ready at %s\n' "$VENV_DIR"
printf 'Using interpreter: %s\n' "$PYTHON_BIN"
printf 'Run commands through it with:\n'
printf '  bash scripts/bootstrap-host-python.sh python -m pytest code/test -m unit\n'
