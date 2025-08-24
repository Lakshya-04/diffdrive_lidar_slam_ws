#-------------------- Builder Stage --------------------#
FROM osrf/ros:jazzy-simulation AS builder

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Install build dependencies
RUN apt-get update && apt-get install -y \
    git \
    sudo \
    python3-pip \
    ros-dev-tools \
    ros-jazzy-slam-toolbox \
    && rm -rf /var/lib/apt/lists/*

ARG USERNAME=rosuser
ARG USER_UID=1001
ARG USER_GID=1001
ARG WORKSPACE_DIR=/home/${USERNAME}/diff_drive_ws

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -ms /bin/bash $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && mkdir -p ${WORKSPACE_DIR}/src \
    && chown -R $USERNAME:$USERNAME /home/$USERNAME

USER $USERNAME
WORKDIR ${WORKSPACE_DIR}

COPY --chown=$USERNAME:$USERNAME src/ ./src/
RUN . /opt/ros/jazzy/setup.bash && \
    rosdep update && \
    rosdep install -i --from-path src -y --rosdistro jazzy --skip-keys "xacro slam_toolbox"

RUN . /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install --event-handlers console_direct+

#-------------------- Final Stage --------------------#
FROM osrf/ros:jazzy-simulation

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Install BOTH runtime AND build dependencies for development
RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    vim \
    ros-dev-tools \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-localization \
    ros-jazzy-interactive-markers \
    ros-jazzy-rviz2 \
    && python3 -m pip install transformations --break-system-packages \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

ARG USERNAME=rosuser
ARG USER_UID=1001
ARG USER_GID=1001
ARG WORKSPACE_DIR=/home/${USERNAME}/diff_drive_ws

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -ms /bin/bash $USERNAME \
    && mkdir -p ${WORKSPACE_DIR} \
    && chown -R $USERNAME:$USERNAME /home/$USERNAME

USER $USERNAME
WORKDIR ${WORKSPACE_DIR}

# Copy ALL workspace directories for development
COPY --chown=$USERNAME:$USERNAME --from=builder ${WORKSPACE_DIR}/install ./install
COPY --chown=$USERNAME:$USERNAME --from=builder ${WORKSPACE_DIR}/build ./build
COPY --chown=$USERNAME:$USERNAME --from=builder ${WORKSPACE_DIR}/log ./log
COPY --chown=$USERNAME:$USERNAME --from=builder ${WORKSPACE_DIR}/src ./src

COPY --chown=$USERNAME:$USERNAME entrypoint.sh /home/$USERNAME/
RUN chmod +x /home/$USERNAME/entrypoint.sh

ENTRYPOINT ["/home/rosuser/entrypoint.sh"]
CMD ["bash"]
