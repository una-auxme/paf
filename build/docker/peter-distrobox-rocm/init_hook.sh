#!/bin/bash
chmod +x "${USER_PACKAGE_DIR}"/init.sh

if [ -z "${DISTROBOX_USERNAME}" ] || [ -z "${USER_PACKAGE_DIR}" ]; then
  echo DISTROBOX_USERNAME and USER_PACKAGE_DIR have to be set for this container to initialize.
  exit 1
fi

username=$(id -u "${DISTROBOX_USERNAME}" -n)
groupname=$(id -g "${DISTROBOX_USERNAME}" -n)

CHECK_CHOWN=${USER_PACKAGE_DIR}/check_chown_path.sh

${CHECK_CHOWN} "${INTERNAL_WORKSPACE_DIR}"
chown -R "${username}":"${groupname}" "${INTERNAL_WORKSPACE_DIR}"
${CHECK_CHOWN} "${ROOT_CATKIN_WS}"
chown -R "${username}":"${groupname}" "${ROOT_CATKIN_WS}"

runuser -u "${username}" -g "${groupname}" "${USER_PACKAGE_DIR}"/init.sh
