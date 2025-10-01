"""
Shared fixtures for ROS2 testing
"""

import pytest
import rclpy
from .test_utilities import (
    ExecutorThread,
    get_param,
    ensure_ros_init,
    create_mock_service_node,
    wait_for_service_registration,
    TestNodeManager,
)


@pytest.fixture(scope="session", autouse=True)
def ros_context():
    """Session-wide ROS2 context initialization"""
    ensure_ros_init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def mock_service_node():
    """Create a mock service node for testing"""
    node = create_mock_service_node()

    with TestNodeManager([node]):
        yield node


@pytest.fixture
def executor_thread():
    """Provide ExecutorThread class for manual node management"""
    return ExecutorThread


@pytest.fixture
def param_loader():
    """Provide get_param function for loading parameters"""
    return get_param
