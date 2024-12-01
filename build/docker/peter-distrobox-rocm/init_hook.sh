#!/bin/bash
chmod +x ${USER_PACKAGE_DIR}/init.sh
username=$(id -u peter -n)
groupname=$(id -g peter -n)

CHECK_CHOWN = ${USER_PACKAGE_DIR}/check_chown_path.sh
chown ${username}:${groupname} ${CHECK_CHOWN}

${CHECK_CHOWN} ${USER_PACKAGE_DIR}
chown -R ${username}:${groupname} ${USER_PACKAGE_DIR}
${CHECK_CHOWN} ${INTERNAL_WORKSPACE_DIR}
mkdir -p ${INTERNAL_WORKSPACE_DIR}
chown ${username}:${groupname} ${INTERNAL_WORKSPACE_DIR}
${CHECK_CHOWN} ${PIP_CACHE_DIR}
chown -R ${username}:${groupname} ${PIP_CACHE_DIR}
${CHECK_CHOWN} ${ROOT_CATKIN_WS}
chown -R ${username}:${groupname} ${ROOT_CATKIN_WS}

runuser -u ${username} -g ${groupname} /${USER_PACKAGE_DIR}/init.sh
