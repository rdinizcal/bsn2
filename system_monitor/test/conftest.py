import pytest
import rclpy
import os
import sys

# Make Python find the package modules
sys.path.insert(0, os.path.abspath(os.path.dirname(os.path.dirname(__file__))))

# Explicitly import modules to ensure coverage
import system_monitor.node_monitor
import system_monitor.logger

@pytest.fixture(scope="session", autouse=True)
def ros_context():
    """ROS context for the entire test session"""
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()