# Tracailer ROS 2 Jazzy development image
# Base: official OSRF ROS 2 Jazzy desktop image (Ubuntu 24.04)
FROM osrf/ros:jazzy-desktop

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

SHELL ["/bin/bash", "-c"]

# ---------------------------------------------------------------------------
# System packages (build tools + Tracailer runtime deps)
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        sudo \
        git \
        curl \
        wget \
        ca-certificates \
        gnupg2 \
        lsb-release \
        locales \
        build-essential \
        cmake \
        ninja-build \
        pkg-config \
        gdb \
        vim \
        nano \
        less \
        python3-pip \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        libeigen3-dev \
        libompl-dev \
        libpcl-dev \
        libboost-all-dev \
        liblapack-dev \
        libblas-dev \
        ros-jazzy-ackermann-msgs \
        ros-jazzy-tf2-geometry-msgs \
        ros-jazzy-pcl-conversions \
        ros-jazzy-pcl-ros \
        ros-jazzy-rviz2 \
        ros-jazzy-rmw-cyclonedds-cpp \
        mesa-utils \
        libgl1-mesa-dri \
        libglu1-mesa \
        libxkbcommon-x11-0 \
        libxcb-cursor0 \
        x11-apps \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

# ---------------------------------------------------------------------------
# CasADi (required by mpc_node) built with LAPACK + qpOASES
# ---------------------------------------------------------------------------
RUN git clone --depth 1 https://github.com/casadi/casadi.git /tmp/casadi \
    && cd /tmp/casadi \
    && sed -i 's/WITH_LAPACK_DEF\s\+OFF/WITH_LAPACK_DEF ON/'     CMakeLists.txt \
    && sed -i 's/WITH_QPOASES_DEF\s\+OFF/WITH_QPOASES_DEF ON/'   CMakeLists.txt \
    && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release .. \
    && make -j"$(nproc)" \
    && make install \
    && ldconfig \
    && rm -rf /tmp/casadi

# ---------------------------------------------------------------------------
# Non-root user with sudo (needed for X11 forwarding + rosdep)
# ---------------------------------------------------------------------------
RUN if getent passwd ${USER_UID} >/dev/null; then \
        existing_user=$(getent passwd ${USER_UID} | cut -d: -f1); \
        userdel -r "${existing_user}" 2>/dev/null || true; \
    fi \
    && groupadd --gid ${USER_GID} ${USERNAME} 2>/dev/null || true \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# ---------------------------------------------------------------------------
# Shell setup: auto-source ROS 2 + workspace overlay if present
# ---------------------------------------------------------------------------
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "if [ -f /workspaces/tracailer/install/setup.bash ]; then source /workspaces/tracailer/install/setup.bash; fi" >> /home/${USERNAME}/.bashrc \
    && chown ${USERNAME}:${USERNAME} /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR /workspaces/tracailer

CMD ["/bin/bash"]
