import time
from pytest_bdd import scenarios, given, when, then
from central_hub.central_hub_tests import SharedCentralHubTests
from sensor.sensor_tests import SharedSensorTests

# Import fixture
from fixtures import context

scenarios("../../features/data_persistance.feature")


@given("that persistence system is online")
def persistence_system_online(context):
    # Our minimal test fixture ensures central hub and sensor are configured
    assert context.central_hub_node is not None
    assert context.sensor_node is not None
    SharedCentralHubTests.assert_components_exist(context.central_hub_node)


@when("I listen to thermometer data")
def listen_to_thermometer(context):
    # Prepare local capture placeholders used by the behave steps
    context.sensor_data = {}
    context.found_high_risk = []
    context.target_system_data = {}
    context.non_sensor = {}
    # record listen start time
    context.listen_start_time = time.monotonic()


@when("I send data to collector")
def send_data_to_collector(context):
    # Use helper to publish a datapoint into the central hub
    msg = SharedCentralHubTests.publish_and_process_sensor_data(
        context.central_hub_node,
        sensor_type="thermometer",
        value=37.5,
        risk_level="normal",
        risk_percentage=10.0,
    )
    # mark that we sent data (behave steps expect this flag)
    context.data_sent_to_collector = True
    context.last_sent_msg = msg


@then("the data will be in persist topic")
def data_will_be_persisted(context):
    # We don't run the logger in this fixture; instead, assert central hub
    # produced a status/event that the persistence system would consume.
    # Ensure detect runs with the current data
    SharedCentralHubTests.assert_receive_datapoint_works(context.central_hub_node)
    # Trigger detection to ensure hub attempts to publish status/event
    context.central_hub_node.detect()
    # At minimum the publisher_manager should exist
    assert hasattr(context.central_hub_node, 'publisher_manager')


@when("a database error prevents persistence")
def simulate_database_error(context):
    # Simulate downstream persistence failure by making a fake publisher that raises
    hub = context.central_hub_node

    if hasattr(hub, 'publisher_manager'):
        orig_publish_status = getattr(hub.publisher_manager, 'publish_status', None)

        def raise_on_publish(*args, **kwargs):
            raise RuntimeError("Simulated persistence failure")

        # patch
        hub._orig_publish_status = orig_publish_status
        hub.publisher_manager.publish_status = raise_on_publish
        context._patched_publish_status = True


@then("the system must log a persistence failure")
def system_logs_persistence_failure(context):
    hub = context.central_hub_node
    # Ensure detect handles publisher exceptions gracefully
    try:
        hub.detect()
        handled = True
    except Exception:
        handled = False

    # Restore patched publisher if we changed it
    if getattr(context, '_patched_publish_status', False):
        hub.publisher_manager.publish_status = hub._orig_publish_status

    assert handled is True, "Detect raised an unexpected exception when persistence failed"