#!/bin/bash
set -e

source /.env.bash

python3 /wait_for_carla.py

exec "$@"
