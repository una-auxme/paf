#!/bin/bash
set -e

${CARLA_ROOT}/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping CarlaUE4 -quality-level=Epic -world-port=${CARLA_PORT} -resx=800 -resy=600 -nosound -carla-settings="${USER_PACKAGE_DIR}/CustomCarlaSettings.ini"
