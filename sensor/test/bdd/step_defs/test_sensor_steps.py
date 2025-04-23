import pytest
from pytest_bdd import given, when, then
from sensor import Sensor
import rclpy

@pytest.fixture(scope="module")
def sensor_node():
    rclpy.init()
    node = Sensor(node_name='test_sensor')
    yield node
    node.destroy_node()
    rclpy.shutdown()

@given("the Sensor node is running")
def sensor_node_running(sensor_node):
    assert sensor_node is not None

@when("the Sensor collects a data point")
def collect_data(sensor_node):
    # Mock the service call response here
    sensor_node.req.vital_sign = "heart_rate"
    class MockResponse:
        datapoint = 85.0
    sensor_node.cli.call = lambda req: MockResponse()
    sensor_node.data_window.clear()  # Clear to ensure predictable behavior
    sensor_node.collect()

@then("the moving average is calculated")
def check_average(sensor_node):
    for i in range(4):  # Fill the window to get 5 items
        sensor_node.process(80.0)
    result = sensor_node.process(90.0)
    assert result == pytest.approx(83.0)

@then("the data is published")
def check_transfer(sensor_node):
    published = []
    # Add logic to verify data publication