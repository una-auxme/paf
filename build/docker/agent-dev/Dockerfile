# syntax = edrevo/dockerfile-plus

INCLUDE+ ./build/docker/agent/Dockerfile

# Add development tools
RUN sudo apt-get update && sudo apt-get install -y \
    vim \
    nano \
    tmux \
    git \
    curl \
    htop

# Create a development entry point script
# RUN echo '#!/bin/bash\n\
# source /opt/ros/noetic/setup.bash\n\
# source /catkin_ws/devel/setup.bash\n\
# exec "$@"' > /dev_entrypoint.sh && \
#     chmod +x /dev_entrypoint.sh

ADD ./build/docker/agent-dev/dev_entrypoint.sh /dev_entrypoint.sh

# Set the development entry point
ENTRYPOINT ["/dev_entrypoint.sh"]

# Default to bash if no command is provided
CMD ["bash"]