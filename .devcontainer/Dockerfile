# Use the official ROS2 Humble base image
FROM ros:humble-ros-base

# Install necessary packages
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install additional tools if needed
RUN pip3 install \
    argcomplete \
    flake8 \
    pytest \
    pytest-cov \
    pytest-mock

# Set up ROS2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

# Create and set the workspace
WORKDIR /workspace

# Set up entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]
