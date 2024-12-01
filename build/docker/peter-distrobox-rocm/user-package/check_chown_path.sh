#!/bin/bash
set -e

CHECK_DIR="$1"

# Root not allowed
[[ ! "$CHECK_DIR" =~ ^/$ ]]
# Relative path not allowed
[[ ! "$CHECK_DIR" =~ ^\./ ]]
# Home not allowed
[[ ! "$CHECK_DIR" =~ ^/home/ ]]
