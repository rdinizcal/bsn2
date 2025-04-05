#!/bin/bash

# Update rosdep
sudo rosdep update

# Install ROS dependencies
sudo rosdep install --from-paths src --ignore-src -y

# Change ownership of /ros_ws
sudo chown -R $(whoami) /ros_ws

# Source ROS 2 setup and workspace setup
echo "source /opt/ros/jazzy/setup.bash" >> /home/DockerUser/.bashrc
echo "source /ros_ws/install/setup.bash" >> /home/DockerUser/.bashrc

# Optional: Build the workspace (if needed)
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

source /ros_ws/install/setup.bash

echo "Setup Complete!"