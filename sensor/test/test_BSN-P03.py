from pytest_bdd import scenarios, given, when, then
from fixtures import context
from shared_test_methods import SharedSensorTests
from central_hub.test.shared_test_methods import SharedCentralHubTests
import time
import rclpy
from bsn_interfaces.msg import SensorData

scenarios("../../features/BSN-P03.feature")


@given("that nodes thermometer and central hub are online")
def nodes_online(context):
    """Ensure both thermometer sensor and central hub nodes are active."""
    assert context.sensor_node is not None
    assert context.central_hub_node is not None
    
    # Use shared sensor methods to verify sensor is working
    SharedSensorTests.assert_collect_works(context.sensor_node)
    SharedSensorTests.assert_processor_components_exist(context.sensor_node)
    
    # Use shared central hub methods to verify central hub is working
    SharedCentralHubTests.assert_components_exist(context.central_hub_node)
    SharedCentralHubTests.assert_receive_datapoint_works(context.central_hub_node)


@when("I listen to thermometer")
def listen_to_thermometer(context):
    """Set up listening to thermometer data."""
    # Clear any existing messages
    context.sensor_node.received_messages = []
    context.central_hub_node.received_messages = []
    # Store start time for timing measurements
    context.listen_start_time = time.time()


@when("thermometer sends data with high risk")
def thermometer_sends_high_risk_data(context):
    """Send high-risk temperature data that should trigger emergency detection."""
    # Use shared sensor method to verify risk evaluation for high temp
    SharedSensorTests.assert_risk_evaluation_works(context.sensor_node, 42.0, "high")
    
    # Use shared sensor method to verify processing with filled window works
    processed_value = SharedSensorTests.assert_process_with_filled_window_works(context.sensor_node)
    
    # Send high-risk temperature (e.g., 42°C fever)
    high_risk_temp = 42.0
    context.emergency_trigger_time = time.time()

    processed = context.sensor_node.processor.process(high_risk_temp)
    
    # Use shared sensor method to verify transfer works
    SharedSensorTests.assert_transfer_works(context.sensor_node, processed)

    # Spin nodes to ensure message delivery
    for _ in range(20):
        rclpy.spin_once(context.sensor_node, timeout_sec=0.01)
        rclpy.spin_once(context.central_hub_node, timeout_sec=0.01)
        time.sleep(0.005)


@when("thermometer sends low-risk data with high frequency")
def thermometer_sends_low_risk_high_frequency(context):
    """Send multiple low-risk readings rapidly to simulate system load."""
    # Use shared sensor method to verify normal temperature risk evaluation
    SharedSensorTests.assert_risk_evaluation_works(context.sensor_node, 36.5, "low")
    
    # Use shared sensor method to verify data flow works
    SharedSensorTests.assert_sensor_data_flow_works(context.sensor_node)
    
    # Fill data window
    for _ in range(context.sensor_node.config.window_size - 1):
        context.sensor_node.processor.data_window.append(37.0)

    # Send 50 low-risk readings rapidly
    for i in range(50):
        low_risk_temp = 36.5 + (i % 3) * 0.1  # 36.5-36.7°C range
        processed = context.sensor_node.processor.process(low_risk_temp)
        context.sensor_node.processor.transfer(processed)

        # Minimal spinning to process messages
        rclpy.spin_once(context.sensor_node, timeout_sec=0.001)
        rclpy.spin_once(context.central_hub_node, timeout_sec=0.001)
        time.sleep(0.001)  # High frequency = 1ms delay


@when("thermometer sends data with high risk", target_fixture="second_high_risk")
def thermometer_sends_second_high_risk_data(context):
    """Send high-risk data after the system has been overloaded."""
    # Use shared sensor method to verify risk evaluation for high temp
    SharedSensorTests.assert_risk_evaluation_works(context.sensor_node, 41.5, "high")
    
    high_risk_temp = 41.5
    context.emergency_trigger_time = time.time()

    processed = context.sensor_node.processor.process(high_risk_temp)
    
    # Use shared sensor method to verify transfer works
    SharedSensorTests.assert_transfer_works(context.sensor_node, processed)

    # Spin nodes to process the high-risk message
    for _ in range(30):
        rclpy.spin_once(context.sensor_node, timeout_sec=0.01)
        rclpy.spin_once(context.central_hub_node, timeout_sec=0.01)
        time.sleep(0.005)


@then("Central hub will detect an emergency in less than 250 ms")
def central_hub_detects_emergency_quickly(context):
    """Verify emergency detection happens within 250ms."""
    # Use shared central hub methods to verify detection works
    SharedCentralHubTests.assert_detect_abnormal_conditions_works(context.central_hub_node)
    SharedCentralHubTests.assert_data_fusion_works(context.central_hub_node)
    
    # Check central hub received emergency notification
    emergency_messages = [
        msg
        for msg in context.central_hub_node.received_messages
        if hasattr(msg, "risk_level") and msg.risk_level == "high"
    ]

    assert len(emergency_messages) > 0, "No emergency messages received by central hub"

    # Calculate detection time
    emergency_msg = emergency_messages[0]
    detection_time = (
        getattr(emergency_msg, "timestamp", time.time())
        - context.emergency_trigger_time
    )
    detection_time_ms = detection_time * 1000

    assert (
        detection_time_ms <= 250
    ), f"Emergency detection took {detection_time_ms:.2f}ms, exceeds 250ms limit"

    # Log the actual detection time for verification
    print(f"Emergency detected in {detection_time_ms:.2f}ms")


@then("Central Hub will experience delayed emergency detection")
def central_hub_delayed_emergency_detection(context):
    """Verify that system overload causes delayed emergency detection."""
    # Use shared central hub methods to verify detection still works under load
    SharedCentralHubTests.assert_detect_abnormal_conditions_works(context.central_hub_node)
    SharedCentralHubTests.assert_data_fusion_works(context.central_hub_node)
    
    # Use shared central hub method to verify components remain functional
    SharedCentralHubTests.assert_components_exist(context.central_hub_node)
    
    # Check that emergency was eventually detected
    emergency_messages = [
        msg
        for msg in context.central_hub_node.received_messages
        if hasattr(msg, "risk_level") and msg.risk_level == "high"
    ]

    assert (
        len(emergency_messages) > 0
    ), "No emergency messages received despite high-risk data"

    # Calculate detection time
    emergency_msg = emergency_messages[-1]  # Get the latest emergency message
    detection_time = (
        getattr(emergency_msg, "timestamp", time.time())
        - context.emergency_trigger_time
    )
    detection_time_ms = detection_time * 1000

    # In overloaded scenario, detection should be delayed (> 250ms)
    assert (
        detection_time_ms > 250
    ), f"Emergency detection was too fast ({detection_time_ms:.2f}ms), expected delay due to system overload"

    # Log the actual detection time
    print(
        f"Emergency detected in {detection_time_ms:.2f}ms (delayed due to system overload)"
    )


def wait_for_emergency_detection(context, timeout=1.0):
    """Helper function to wait for emergency detection with timeout."""
    start_time = time.time()
    while time.time() - start_time < timeout:
        # Spin both nodes
        rclpy.spin_once(context.sensor_node, timeout_sec=0.01)
        rclpy.spin_once(context.central_hub_node, timeout_sec=0.01)

        # Check for emergency messages
        emergency_messages = [
            msg
            for msg in context.central_hub_node.received_messages
            if hasattr(msg, "risk_level") and msg.risk_level == "high"
        ]

        if emergency_messages:
            return emergency_messages[0]

        time.sleep(0.01)

    return None
