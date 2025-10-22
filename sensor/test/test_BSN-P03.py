import time
import rclpy
from pytest_bdd import scenarios, given, when, then
from central_hub.central_hub_tests import SharedCentralHubTests
from sensor.sensor_tests import SharedSensorTests

# Import fixture
from fixtures import context

scenarios("../../features/BSN-P03.feature")


@given("that nodes thermometer and central hub are online")
def nodes_online(context):
    assert context.sensor_node is not None, "Sensor node should exist"
    assert context.central_hub_node is not None, "Central hub node should exist"
    # Basic component checks
    SharedSensorTests.assert_processor_components_exist(context.sensor_node)
    SharedCentralHubTests.assert_components_exist(context.central_hub_node)


@when("I listen to thermometer")
def listen_to_thermometer(context):
    context.listen_start_time = time.monotonic()
    # Clear any previous state
    if hasattr(context.central_hub_node, 'sensor_handler'):
        context.central_hub_node.sensor_handler.latest_data = {}
        context.central_hub_node.sensor_handler.latest_risk = {}


@when("thermometer sends data with high risk")
def thermometer_sends_high_risk(context):
    # Send a single high-risk datapoint directly to the hub and record publish time
    send_time = time.monotonic()
    SharedCentralHubTests.publish_and_process_sensor_data(
        context.central_hub_node,
        sensor_type="thermometer",
        value=38.0,
        risk_level="high",
        risk_percentage=90.0,
    )
    context.last_send_time = send_time


@then("Central hub will detect an emergency in less than 250 ms")
def central_hub_detects_fast(context):
    # Measure detection time by invoking detect and timing it
    t0 = time.monotonic()
    context.central_hub_node.detect()
    t1 = time.monotonic()
    elapsed = t1 - t0
    assert elapsed <= 0.250, f"Detection took too long: {elapsed:.3f}s"


@when("thermometer sends low-risk data with high frequency")
def thermometer_sends_high_freq_low_risk(context):
    # Simulate an overload: many low-risk messages in quick succession
    for i in range(60):
        SharedCentralHubTests.publish_and_process_sensor_data(
            context.central_hub_node,
            sensor_type="thermometer",
            value=36.0,
            risk_level="low",
            risk_percentage=5.0,
        )


@when("But thermometer sends data with high risk")
def then_send_high_risk_after_overload(context):
    # After overload, send the high-risk datapoint and measure detection time
    send_time = time.monotonic()
    SharedCentralHubTests.publish_and_process_sensor_data(
        context.central_hub_node,
        sensor_type="thermometer",
        value=38.0,
        risk_level="high",
        risk_percentage=90.0,
    )
    context.last_send_time = send_time


@then("Central Hub will experience delayed emergency detection")
def central_hub_detects_slow(context):
    t0 = time.monotonic()
    context.central_hub_node.detect()
    t1 = time.monotonic()
    elapsed = t1 - t0
    # We expect the system to be delayed due to prior load; assert detection > 250ms
    assert elapsed > 0.250, f"Detection was not delayed as expected: {elapsed:.3f}s"
