#!/usr/bin/env bash

echo "Info: This file tests, if you can push a simple docker image to your GHCR."
echo "It does NOT build or push any CARLA images."
echo "Remember to delete the test package in your organization."

set -euo pipefail

# Config: set your org. GH_USER will be auto-detected if possible.
ORG="una-auxme"
IMAGE="hello-world"
TAG="latest"

# Resolve GH_TOKEN
if [ -z "${GH_TOKEN:-}" ]; then
  if command -v gh >/dev/null 2>&1 && gh auth status >/dev/null 2>&1; then
    GH_TOKEN="$(gh auth token)"
    echo "[info] Using token from gh CLI."
  else
    echo "[error] GH_TOKEN not set and gh CLI not logged in."
    echo "        Export GH_TOKEN (PAT with write:packages) or run: gh auth login"
    exit 1
  fi
fi

# Resolve GH_USER (token owner)
if [ -z "${GH_USER:-}" ]; then
  if command -v jq >/dev/null 2>&1; then
    GH_USER="$(curl -fsSL -H "Authorization: Bearer $GH_TOKEN" https://api.github.com/user | jq -r .login)"
  else
    GH_USER="$(curl -fsSL -H "Authorization: Bearer $GH_TOKEN" https://api.github.com/user | sed -n 's/.*"login": *"\([^"]*\)".*/\1/p')"
  fi
  if [ -z "$GH_USER" ] || [ "$GH_USER" = "null" ]; then
    echo "[error] Could not determine token owner username via GitHub API."
    echo "        Set GH_USER explicitly: export GH_USER=<your-github-username>"
    exit 1
  fi
fi

# Basic checks
command -v docker >/dev/null 2>&1 || { echo "[error] Docker not found."; exit 1; }

# Optional scope hint (classic PATs only)
if command -v curl >/dev/null 2>&1; then
  SCOPES="$(curl -sI -H "Authorization: Bearer $GH_TOKEN" https://api.github.com/user | tr -d '\r' | sed -n 's/^x-oauth-scopes: *//Ip')"
  if [ -n "$SCOPES" ] && ! echo "$SCOPES" | grep -q 'write:packages'; then
    echo "[warn] Token appears to be missing write:packages (scopes: $SCOPES)"
    echo "       If your org enforces SSO, ensure the token is SSO-granted."
  fi
fi

# Login
echo "$GH_TOKEN" | docker login ghcr.io -u "$GH_USER" --password-stdin
echo "[info] Logged in to ghcr.io as $GH_USER"

# Prepare and push
echo "[info] Pulling $IMAGE:$TAG"
docker pull "$IMAGE:$TAG"

TARGET="ghcr.io/$ORG/$IMAGE:$TAG"
echo "[info] Tagging as $TARGET"
docker tag "$IMAGE:$TAG" "$TARGET"

echo "[info] Pushing $TARGET"
docker push "$TARGET"

echo "[ok] Pushed $TARGET"
echo "If access is denied in the UI, adjust package visibility under your organization’s Packages settings."
