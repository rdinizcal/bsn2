import pytest
import rclpy

@pytest.fixture(scope="session", autouse=True)
def ros_setup():
    rclpy.init()
    yield
    rclpy.shutdown()
