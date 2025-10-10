#!/usr/bin/env bash
# ==============================================================
# Secure Docker Credential Helper Setup for GitHub Container Registry (ghcr.io)
# Works on Ubuntu Desktop (secretservice) and Ubuntu Server (pass)
# ==============================================================

set -euo pipefail

echo "=== Secure Docker Credential Helper Setup for GHCR ==="

# --------------------------------------------------------------
# 1. Check prerequisites
# --------------------------------------------------------------
if ! command -v docker &>/dev/null; then
    echo "ERROR: Docker is not installed. Please install Docker first:"
    echo "  https://docs.docker.com/engine/install/ubuntu/"
    exit 1
fi

if ! command -v gh &>/dev/null; then
    echo "ERROR: GitHub CLI (gh) is not installed. Please install it:"
    echo "  https://cli.github.com/manual/installation"
    exit 1
fi

# --------------------------------------------------------------
# 2. Detect whether a GUI environment (Secret Service) is available
# --------------------------------------------------------------
if command -v gnome-keyring-daemon &>/dev/null || pgrep -x "gnome-keyring-daemon" &>/dev/null; then
    HELPER="secretservice"
    echo "Detected desktop environment → using 'secretservice' helper."
else
    HELPER="pass"
    echo "No desktop environment detected → using 'pass' helper."
fi

# --------------------------------------------------------------
# 3. Define helper binary URL according to architecture
# --------------------------------------------------------------
VER="v0.9.4"
ARCH=$(uname -m)
case "$ARCH" in
    x86_64)   SUF="linux-amd64" ;;
    aarch64)  SUF="linux-arm64" ;;
    armv7l)   SUF="linux-armv7" ;;
    *) echo "Unsupported architecture: $ARCH"; exit 1 ;;
esac

URL="https://github.com/docker/docker-credential-helpers/releases/download/${VER}/docker-credential-${HELPER}-${VER}.${SUF}"

# --------------------------------------------------------------
# 4. Download and install the credential helper binary
# --------------------------------------------------------------
echo "Downloading docker-credential-${HELPER} from official GitHub release..."
curl -fsSL -o /tmp/docker-credential-${HELPER} "$URL"

if [ ! -s /tmp/docker-credential-${HELPER} ]; then
    echo "ERROR: Download failed or file is empty."
    exit 1
fi

sudo install -m 0755 /tmp/docker-credential-${HELPER} /usr/local/bin/docker-credential-${HELPER}
rm -f /tmp/docker-credential-${HELPER}
echo "Installed /usr/local/bin/docker-credential-${HELPER}"

# --------------------------------------------------------------
# 5. Configure Docker to use the helper
# --------------------------------------------------------------
mkdir -p ~/.docker
CONFIG_FILE=~/.docker/config.json

echo "Configuring Docker to use '${HELPER}' credential store..."
cat > "$CONFIG_FILE" <<EOF
{
  "credsStore": "${HELPER}"
}
EOF

# --------------------------------------------------------------
# 6. Install helper dependencies
# --------------------------------------------------------------
if [ "$HELPER" = "secretservice" ]; then
    echo "Installing Secret Service dependencies (gnome-keyring)..."
    sudo apt update -qq
    sudo apt install -y gnome-keyring libsecret-1-0 dbus-user-session
else
    echo "Installing 'pass' and 'gnupg2' for encrypted storage..."
    sudo apt update -qq
    sudo apt install -y pass gnupg2
    # Initialize pass store if needed
    if ! pass show test &>/dev/null; then
        echo "Initializing pass store..."
        KEYID=$(gpg --list-secret-keys --keyid-format=long | awk '/^sec/ {print $2}' | sed 's#.*/##' | head -n1)
        if [ -z "$KEYID" ]; then
            echo "No GPG key found. Generating one..."
            gpg --batch --passphrase '' --quick-gen-key "Docker Credentials <docker@example.com>" default default 0
            KEYID=$(gpg --list-secret-keys --keyid-format=long | awk '/^sec/ {print $2}' | sed 's#.*/##' | head -n1)
        fi
        pass init "$KEYID"
    fi
fi

# --------------------------------------------------------------
# 7. Log in securely to GHCR
# --------------------------------------------------------------
echo "Logging in to ghcr.io using your GitHub CLI token..."
TOKEN=$(gh auth token)
USER=$(gh api user --jq .login)

if [ -z "$USER" ]; then
    echo "Could not determine GitHub username. Ensure 'gh auth login' completed successfully."
    exit 1
fi

echo "$TOKEN" | docker login ghcr.io -u "$USER" --password-stdin

# --------------------------------------------------------------
# 8. Validation step (robust)
# --------------------------------------------------------------
echo
echo "=== Validation ==="
CONFIG=~/.docker/config.json

# We use jq if available for safe JSON parsing; otherwise a grep fallback.
if ! command -v jq >/dev/null 2>&1; then
  echo "Note: jq not found; using grep-based checks. For stricter validation: sudo apt install -y jq"
fi

VALID=1

# 8.1 Ensure a credsStore is actually set
HELPER=""
if command -v jq >/dev/null 2>&1; then
  HELPER="$(jq -r '.credsStore // empty' "$CONFIG")"
else
  HELPER="$(grep -oE '"credsStore"\s*:\s*"[^"]+"' "$CONFIG" | sed -E 's/.*"credsStore"\s*:\s*"([^"]+)".*/\1/')"
fi

if [[ -z "$HELPER" ]]; then
  echo "❌ No 'credsStore' configured in $CONFIG."
  VALID=0
else
  echo "credsStore: $HELPER"
fi

# 8.2 Helper binary must exist and be executable
HELPER_PATH=$(command -v "docker-credential-$HELPER" 2>/dev/null || echo "")
if [[ -z "$HELPER_PATH" ]]; then
  echo "❌ Helper binary docker-credential-$HELPER not found in PATH."
  VALID=0
else
  # Test if the helper binary is executable and responds to version command
  if "docker-credential-$HELPER" version >/dev/null 2>&1; then
    echo "✅ Helper binary docker-credential-$HELPER is present and runnable."
  else
    echo "❌ Helper binary docker-credential-$HELPER exists but is not responding to 'version' command."
    VALID=0
  fi
fi

# 8.3 Detect plaintext creds: look for any .auths[].auth fields
if command -v jq >/dev/null 2>&1; then
  if jq -e '.auths | to_entries | map(select(.value | has("auth"))) | length > 0' "$CONFIG" >/dev/null; then
    echo "❌ Plaintext credentials found in $CONFIG (.auths[].auth present)."
    VALID=0
  else
    echo "No plaintext tokens detected in $CONFIG."
  fi
else
  if grep -qE '"auth"\s*:\s*"' "$CONFIG"; then
    echo "❌ Plaintext credentials found in $CONFIG (.auths[].auth present)."
    VALID=0
  else
    echo "No plaintext tokens detected in $CONFIG (grep check)."
  fi
fi

# 8.4 Ask the helper directly if it can return creds for ghcr.io
#    Docker sometimes stores the key under either 'ghcr.io' or 'https://ghcr.io'
CAN_GET=0
if printf '{"ServerURL":"ghcr.io"}' | "docker-credential-$HELPER" get >/dev/null 2>&1; then
  echo "Helper returned credentials for 'ghcr.io'."
  CAN_GET=1
elif printf '{"ServerURL":"https://ghcr.io"}' | "docker-credential-$HELPER" get >/dev/null 2>&1; then
  echo "Helper returned credentials for 'https://ghcr.io'."
  CAN_GET=1
else
  echo "⚠️  Helper did not return stored creds for ghcr.io (maybe not logged in yet or keyring locked)."
fi

# 8.5 Functional auth test (public image still exercises auth path)
echo "Testing GHCR access by pulling a public image..."
if docker pull ghcr.io/github/super-linter:latest >/dev/null 2>&1; then
  echo "✅ GHCR pull test: OK"
else
  echo "⚠️  GHCR pull test failed (network/auth). If private repos fail, ensure token scopes & org SSO."
  VALID=0
fi

# 8.6 Final result
echo
if [[ $VALID -eq 1 && $CAN_GET -eq 1 ]]; then
  echo "✅ Secure storage validated: helper active, no plaintext in config, helper can serve credentials."
elif [[ $VALID -eq 1 ]]; then
  echo "⚠️  Setup mostly OK, but helper couldn't retrieve creds (keyring may be locked or no login stored yet)."
else
  echo "❌ Validation failed. See messages above for exact reason(s)."
  exit 1
fi

echo
echo "Setup complete. Docker credentials are now encrypted via '${HELPER}'."
