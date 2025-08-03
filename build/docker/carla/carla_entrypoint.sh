#!/bin/bash
set -e

_term() {
  echo "Caught SIGTERM signal!"
  kill -TERM "$(jobs -p)" 2>/dev/null
}
trap _term SIGTERM SIGINT

"$@" &
wait "$!"
