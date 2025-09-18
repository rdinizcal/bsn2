import rclpy
import pytest
@pytest.fixture(scope="module")
def ros_context():
    """Initialize ROS once for all tests in this module."""
    # Check if ROS is already initialized to avoid errors
    try:
        if not rclpy.ok():
            rclpy.init()
    except:
        # Already initialized
        pass
    
    yield