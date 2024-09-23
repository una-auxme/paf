#!/bin/bash
# run docker compose file specified as argument and located in the build directory

# enable xhost for the current user to allow docker to display graphics
./xhost_enable.sh

# run docker compose
if [ $# -eq 0 ]; then
    echo "Usage: $0 <path-to-docker-compose-file>"
    exit 1
fi

docker compose -f "$1" up
