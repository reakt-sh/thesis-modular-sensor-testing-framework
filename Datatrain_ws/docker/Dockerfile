FROM ros:jazzy-ros-base
# Note: use official multi-arch base (arm64-native on Apple Silicon); add desktop-full via apt

ENV DEBIAN_FRONTEND=noninteractive

# Base tools, ROS dev tools, lightweight desktop + VNC + noVNC
RUN apt-get clean && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get update --fix-missing && \
    apt-get install -y --no-install-recommends \
    bash-completion locales apt-transport-https ca-certificates curl gnupg lsb-release \
    python3-pip python3-venv git build-essential cmake \
    python3-colcon-common-extensions python3-rosdep python3-vcstool \
    xfce4 xfce4-terminal xfce4-goodies tigervnc-standalone-server tigervnc-tools novnc websockify \
    dbus-x11 x11-xserver-utils mesa-utils fonts-noto \
    arc-theme papirus-icon-theme \
 && rm -rf /var/lib/apt/lists/*

# Full ROS desktop (RViz, demos, etc.) while staying on the official multi-arch base
RUN apt-get clean && rm -rf /var/lib/apt/lists/* && \
    apt-get update --fix-missing && \
    apt-get install -y --no-install-recommends \
    ros-jazzy-desktop-full \
 && rm -rf /var/lib/apt/lists/*

# Gazebo Harmonic (gz CLI) from OSRF repo
RUN set -eux; \
    apt-get clean && rm -rf /var/lib/apt/lists/*; \
    apt-get update && apt-get install -y --no-install-recommends curl gnupg lsb-release; \
    curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg; \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(. /etc/os-release && echo $VERSION_CODENAME) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list; \
    apt-get clean && rm -rf /var/lib/apt/lists/*; \
    apt-get update; \
    apt-get install -y --no-install-recommends gz-harmonic; \
    rm -rf /var/lib/apt/lists/*

# ROS ↔ Gazebo bridge
RUN apt-get clean && rm -rf /var/lib/apt/lists/* && \
    apt-get update --fix-missing && \
    apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz \
 && rm -rf /var/lib/apt/lists/*

# rosdep setup
RUN rosdep init || true && rosdep update

# Python venv that can see system ROS packages
RUN python3 -m venv --system-site-packages /root/venvs/ros \
 && printf '%s\n' \
    'source /opt/ros/jazzy/setup.bash' \
    'source /root/venvs/ros/bin/activate' \
    'test -f /ws/install/setup.bash && source /ws/install/setup.bash' \
    >> /root/.bashrc

WORKDIR /ws
# (Keep upstream ENTRYPOINT ["/ros_entrypoint.sh"] and CMD ["bash"])