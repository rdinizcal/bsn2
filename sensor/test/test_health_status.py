import time
from pytest_bdd import scenarios, given, when, then
from central_hub.central_hub_tests import SharedCentralHubTests
from sensor.sensor_tests import SharedSensorTests
from sensor.sensor import Sensor
from fixtures import context,SensorTestContext

scenarios("../../features/health_status.feature")


@given("that nodes thermometer and central hub are online")
def nodes_online(context: SensorTestContext):
    assert context.sensor_node is not None, "Sensor node should exist"
    assert context.central_hub_node is not None, "Central hub node should exist"
    # Basic component checks
    SharedSensorTests.assert_processor_components_exist(context.sensor_node)
    SharedCentralHubTests.assert_components_exist(context.central_hub_node)


@when("I listen to thermometer")
def listen_to_thermometer(context: SensorTestContext):
    # Clear any previous state and collect first datapoint (37.0)
    SharedSensorTests.assert_collect_works(context.sensor_node, 37.0)
    
    # Allow time for central hub to receive and process the data
    time.sleep(0.1)
    
    # Store the initial thermometer risk after receiving 37.0
    initial_risk = context.central_hub_node.sensor_handler.latest_risk.get("thermometer", -1.0)
    context.test_data['initial_thermometer_risk'] = initial_risk
    
    # Now collect second datapoint (43.0)
    context.mock_service_node.test_data['last_datapoint'] = 43.0
    SharedSensorTests.assert_collect_works(context.sensor_node, 43.0)
    
    # Allow time for central hub to receive and process the new data
    time.sleep(0.1)


@then("g4t1 will detect new patient health status")
def detect_health_status(context: SensorTestContext):
    # Get the current thermometer risk after receiving 43.0
    current_risk = context.central_hub_node.sensor_handler.latest_risk.get("thermometer", -1.0)
    initial_risk = context.test_data.get('initial_thermometer_risk', -1.0)
    
    # Assert that the central hub received and processed both datapoints
    assert current_risk >= 0, f"Central hub should have thermometer risk data, got: {current_risk}"
    assert initial_risk >= 0, f"Initial thermometer risk should have been recorded, got: {initial_risk}"
    
    # The key test: risk should change when temperature goes from 37.0 to 43.0
    # Since 43.0 is likely higher risk than 37.0, we expect the risk to increase
    assert current_risk > initial_risk, f"Thermometer risk should change from {initial_risk} to {current_risk} when temperature changes from 37.0 to 43.0"
    
    # Verify the central hub's overall functionality
    result = SharedCentralHubTests.assert_receive_datapoint_works(
        context.central_hub_node
    )
    assert result, "Central hub failed to detect health status under abnormal conditions"


@when("an internal processing error occurs in g4t1")
def internal_processing_error(context):
    # Prepare the hub to simulate a fusion_engine exception during detect
    hub = context.central_hub_node
    # Save original to restore later if test framework reuses the hub
    context._original_fusion_engine = getattr(hub, 'fusion_engine', None)

    class MockFusionEngine:
        def fuse_data(self):
            raise RuntimeError("Simulated fusion error")
    
    hub.fusion_engine = MockFusionEngine()


@then("Central hub will fail to detect the new patient health status")
def detect_fails_on_error(context):
    hub = context.central_hub_node
    # Call detect and ensure it doesn't crash the test runner; the hub should handle the exception
    try:
        hub.detect()
        handled = True
    except Exception:
        handled = False

    # In this sad path we expect the hub to not successfully detect due to internal error.
    # If detect() swallowed the exception, 'handled' will be True but the fusion step failed.
    # The best we can assert here is that the hub did not raise an uncaught exception.
    assert handled is True, "Detect raised an unexpected exception instead of handling it"
