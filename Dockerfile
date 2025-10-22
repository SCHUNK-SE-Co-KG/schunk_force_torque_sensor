FROM ros:humble-ros-core

ENV DEBIAN_FRONTEND=noninteractive

# Install essentials
RUN apt-get update -qq && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    curl \
    libcurl4-openssl-dev \
    libexpat1-dev \
    gettext \
    libz-dev \
    libssl-dev \
    openssh-client \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-rosinstall-generator \
    python3-vcstool \
    git \
    socat \
    ros-humble-example-interfaces \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
RUN mkdir -p src

# Initialize rosdep and update database
RUN rosdep init || true
RUN rosdep update

# upgrade pip globally
RUN python3 -m pip install --upgrade pip

# Ensure automatic sourcing for any terminal
RUN echo 'if [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash; fi' >> /root/.bashrc

# Set bash as default shell
SHELL ["/bin/bash", "-c"]

# Default command
CMD ["bash"]
