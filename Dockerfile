# Use the official ROS 2 Jazzy image
FROM ros:jazzy-ros-base

# Set the working directory
WORKDIR /ros_ws

# Copy your source code
COPY . /ros_ws/src

# Install dependencies and build
RUN apt update && apt install -y \
    ros-jazzy-ros-base \
    && rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

# Source the workspace when starting the container
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && source /ros_ws/install/setup.bash && bash"]
