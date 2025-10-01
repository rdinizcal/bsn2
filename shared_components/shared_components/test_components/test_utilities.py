"""
Shared test utilities for ROS2 testing across packages
"""

import os
import threading
import yaml
import time
import rclpy
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


class ExecutorThread(threading.Thread):
    """Thread-safe executor for managing multiple ROS2 nodes in tests"""

    def __init__(self, nodes):
        super().__init__(daemon=True)
        self.executor = SingleThreadedExecutor()
        self.nodes = nodes
        for node in self.nodes:
            self.executor.add_node(node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)

    def run(self):
        """Start the executor thread"""
        self.executor_thread.start()

    def clean_up(self):
        """Clean up all nodes and shutdown executor"""
        # Remove nodes from executor first
        for node in self.nodes:
            try:
                if hasattr(node, "lifecycle_manager"):
                    node.lifecycle_manager.shutdown_node()
                self.executor.remove_node(node)
            except Exception as e:
                print(f"Error removing node from executor: {e}")

        # Shutdown executor
        try:
            self.executor.shutdown()
        except Exception as e:
            print(f"Error shutting down executor: {e}")

        # Destroy nodes
        for node in self.nodes:
            try:
                node.destroy_node()
            except Exception as e:
                print(f"Error destroying node: {e}")

        # Join executor thread with timeout
        if self.executor_thread.is_alive():
            self.executor_thread.join(timeout=2.0)


def get_param(package, node_name, yaml_file):
    """
    Load ROS2 parameters from YAML file

    Args:
        package: ROS2 package name
        node_name: Node name in the YAML file
        yaml_file: YAML configuration file name

    Returns:
        List of Parameter objects
    """
    params_path = os.path.join(
        get_package_share_directory(package), "config", yaml_file
    )

    if not os.path.exists(params_path):
        raise FileNotFoundError(f"Parameter file not found: {params_path}")

    with open(params_path, "r") as f:
        full_params = yaml.safe_load(f)

    if node_name not in full_params:
        raise KeyError(f"Node '{node_name}' not found in {yaml_file}")

    if "ros__parameters" not in full_params[node_name]:
        raise KeyError(
            f"'ros__parameters' not found for node '{node_name}' in {yaml_file}"
        )

    ros_params = full_params[node_name]["ros__parameters"]
    return [Parameter(name=k, value=v) for k, v in ros_params.items()]


def ensure_ros_init():
    """Ensure ROS2 is initialized, initialize if not"""
    if not rclpy.ok():
        rclpy.init()


def create_mock_service_node(node_name="mock_service_provider"):
    """Create a basic mock service node for testing"""
    ensure_ros_init()
    return Node(node_name)


def shutdown_ros_init():
    """Shutdown ROS2 if initialized"""
    if rclpy.ok():
        rclpy.shutdown()


def wait_for_service_registration(duration=1.0):
    """Wait for services to be registered"""
    time.sleep(duration)
