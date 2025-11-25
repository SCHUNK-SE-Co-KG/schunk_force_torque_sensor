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

WORKDIR /schunk_force_torque_sensor
RUN mkdir -p src

# Copy ROS packages
COPY schunk_fts_driver /schunk_force_torque_sensor/src/schunk_fts_driver
COPY schunk_fts_dummy /schunk_force_torque_sensor/src/schunk_fts_dummy
COPY schunk_fts_interfaces /schunk_force_torque_sensor/src/schunk_fts_interfaces
COPY schunk_fts_library /schunk_force_torque_sensor/src/schunk_fts_library

# Initialize rosdep
RUN rosdep init || true &&\
    rosdep update

# Install Python packages
RUN pip install -e /schunk_force_torque_sensor/src/schunk_fts_library &&\
    pip install -e /schunk_force_torque_sensor/src/schunk_fts_driver

# Build ROS workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Ensure workspace is sourced in every shell
RUN echo 'source /opt/ros/humble/setup.bash' >> /etc/bash.bashrc && \
    echo 'source /schunk_force_torque_sensor/install/setup.bash' >> /etc/bash.bashrc

# Clean up apt cache
RUN rm -rf /var/lib/apt/lists/*

# Set bash as default shell
SHELL ["/bin/bash", "-c"]
CMD ["bash"]
