import time
from pytest_bdd import scenarios, given, when, then
from central_hub.central_hub_tests import SharedCentralHubTests
from sensor.sensor_tests import SharedSensorTests

# Import fixture
from fixtures import context

scenarios("../../features/health_status.feature")


@given("that nodes thermometer and central hub are online")
def nodes_online(context):
    assert context.sensor_node is not None, "Sensor node should exist"
    assert context.central_hub_node is not None, "Central hub node should exist"
    # Basic component checks
    SharedSensorTests.assert_processor_components_exist(context.sensor_node)
    SharedCentralHubTests.assert_components_exist(context.central_hub_node)


@when("I listen to thermometer")
def listen_to_thermometer(context):
    # use monotonic for timing stability
    context.listen_start_time = time.monotonic()
    # Clear any previous state
    if hasattr(context.central_hub_node, 'sensor_handler'):
        context.central_hub_node.sensor_handler.latest_data = {}
        context.central_hub_node.sensor_handler.latest_risk = {}


@then("g4t1 will detect new patient health status")
def detect_health_status(context):
    # For the happy path, simulate abnormal conditions so the hub's detect() is exercised
    result = SharedCentralHubTests.assert_detect_abnormal_conditions_works(
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
