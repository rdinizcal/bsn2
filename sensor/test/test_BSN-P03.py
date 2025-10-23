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
    # Record the exact time when thermometer starts sending high-risk data
    context.transmission_start_time = time.monotonic()
    
    # Send high-risk datapoint and process through the full pipeline
    SharedCentralHubTests.publish_and_process_sensor_data(
        context.central_hub_node,
        sensor_type="thermometer",
        value=38.0,
        risk_level="high",
        risk_percentage=90.0,
    )
    
    # Record when data transmission and processing is complete
    context.data_ready_time = time.monotonic()


@then("Central hub will detect an emergency in less than 250 ms")
def central_hub_detects_fast(context):
    # Measure complete pipeline: thermometer send → central hub detection
    detection_start_time = time.monotonic()
    context.central_hub_node.detect()
    detection_end_time = time.monotonic()
    
    # Calculate total pipeline time (transmission + processing + detection)
    total_pipeline_time = detection_end_time - context.transmission_start_time
    
    # Calculate just the detection phase time  
    detection_only_time = detection_end_time - detection_start_time
    
    # Store timing data for analysis
    context.test_data = {
        'total_pipeline_ms': total_pipeline_time * 1000,
        'detection_only_ms': detection_only_time * 1000,
        'transmission_processing_ms': (context.data_ready_time - context.transmission_start_time) * 1000
    }
    
    # Assert BSN-P03 requirement: complete pipeline under 250ms
    assert total_pipeline_time <= 0.250, (
        f"Complete thermometer→central_hub pipeline took {total_pipeline_time*1000:.2f}ms, "
        f"exceeds 250ms limit. Breakdown: "
        f"transmission+processing={context.test_data['transmission_processing_ms']:.2f}ms, "
        f"detection={context.test_data['detection_only_ms']:.2f}ms"
    )


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
    # After overload, send high-risk datapoint and measure complete timing
    context.overload_transmission_start_time = time.monotonic()
    
    SharedCentralHubTests.publish_and_process_sensor_data(
        context.central_hub_node,
        sensor_type="thermometer",
        value=38.0,
        risk_level="high",
        risk_percentage=90.0,
    )
    
    context.overload_data_ready_time = time.monotonic()


@then("Central Hub will experience delayed emergency detection")
def central_hub_detects_slow(context):
    detection_start_time = time.monotonic()
    context.central_hub_node.detect()
    detection_end_time = time.monotonic()
    
    # Calculate overloaded pipeline timing
    total_overload_time = detection_end_time - context.overload_transmission_start_time
    detection_only_time = detection_end_time - detection_start_time
    
    # Store overload timing data
    context.overload_test_data = {
        'total_pipeline_ms': total_overload_time * 1000,
        'detection_only_ms': detection_only_time * 1000,
        'transmission_processing_ms': (context.overload_data_ready_time - context.overload_transmission_start_time) * 1000
    }
    
    # Under overload conditions, we expect delayed detection (> 250ms)
    assert total_overload_time > 0.250, (
        f"Overloaded pipeline should exceed 250ms but took {total_overload_time*1000:.2f}ms. "
        f"Breakdown: transmission+processing={context.overload_test_data['transmission_processing_ms']:.2f}ms, "
        f"detection={context.overload_test_data['detection_only_ms']:.2f}ms"
    )
