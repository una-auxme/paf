#!/bin/bash

# enable xhost
./xhost_enable.sh

# run docker compose
if [ $# -eq 0 ]; then
    echo "Usage: $0 <path-to-docker-compose-file>"
    exit 1
fi

docker compose -f "$1" up
