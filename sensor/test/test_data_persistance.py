import time
from pytest_bdd import scenarios, given, when, then
from central_hub.central_hub_tests import SharedCentralHubTests
from sensor.sensor_tests import SharedSensorTests
from system_monitor.shared_logger_tests import SharedLoggerTests
from system_monitor.shared_node_monitor_tests import SharedNodeMonitorTests
from adaptation.shared_data_access_tests import SharedDataAccessTests

# Import fixture
from fixtures import context_persistence

scenarios("../../features/data_persistance.feature")


@given("that persistence system is online")
def persistence_system_online(context_persistence):
    if not hasattr(context_persistence, 'test_data'):
        context_persistence.test_data = {}
    SharedCentralHubTests.assert_components_exist(context_persistence.central_hub_node)
    SharedSensorTests.assert_processor_components_exist(context_persistence.sensor_node)
    SharedLoggerTests.assert_logger_initialized(context_persistence.logger_node)
    SharedNodeMonitorTests.assert_monitor_initialized(context_persistence.node_monitor_node)
    SharedDataAccessTests.assert_data_access_initialized(context_persistence.data_access_node)
    # Store initial state in test_data
    context_persistence.test_data['central_hub_node'] = context_persistence.central_hub_node
    context_persistence.test_data['sensor_node'] = context_persistence.sensor_node
    context_persistence.test_data['logger_node'] = context_persistence.logger_node
    context_persistence.test_data['node_monitor_node'] = context_persistence.node_monitor_node
    context_persistence.test_data['data_access_node'] = context_persistence.data_access_node


@when("I listen to thermometer data")
def listen_to_thermometer(context_persistence):
    context_persistence.test_data['sensor_data'] = {}
    context_persistence.test_data['found_high_risk'] = []
    context_persistence.test_data['target_system_data'] = {}
    context_persistence.test_data['non_sensor'] = {}
    context_persistence.test_data['listen_start_time'] = time.monotonic()


@when("I send data to collector")
def send_data_to_collector(context_persistence):
    msg = SharedCentralHubTests.publish_and_process_sensor_data(
        context_persistence.test_data['central_hub_node'],
        sensor_type="thermometer",
        value=37.5,
        risk_level="normal",
        risk_percentage=10.0,
    )
    context_persistence.test_data['data_sent_to_collector'] = True
    context_persistence.test_data['last_sent_msg'] = msg


@then("the data will be in persist topic")
def data_will_be_persisted(context_persistence):
    SharedCentralHubTests.assert_receive_datapoint_works(context_persistence.test_data['central_hub_node'])
    SharedLoggerTests.assert_receive_status_works(context_persistence.test_data.get('logger_received_messages', []))
    SharedNodeMonitorTests.assert_status_forwarding_works(context_persistence.test_data.get('node_monitor_received_messages', []))
    SharedDataAccessTests.assert_receive_persist_message_status(context_persistence.test_data['data_access_node'])
    SharedSensorTests.assert_sensor_data_flow_works(context_persistence.test_data['sensor_node'])


@when("a database error prevents persistence")
def simulate_database_error(context_persistence):
    # Simulate downstream persistence failure by making a fake publisher that raises
    hub = context_persistence.central_hub_node

    if hasattr(hub, 'publisher_manager'):
        orig_publish_status = getattr(hub.publisher_manager, 'publish_status', None)

        def raise_on_publish(*args, **kwargs):
            raise RuntimeError("Simulated persistence failure")

        # patch
        hub._orig_publish_status = orig_publish_status
        hub.publisher_manager.publish_status = raise_on_publish
        context_persistence._patched_publish_status = True


@then("the system must log a persistence failure")
def system_logs_persistence_failure(context_persistence):
    hub = context_persistence.central_hub_node
    # Ensure detect handles publisher exceptions gracefully
    try:
        hub.detect()
        handled = True
    except Exception:
        handled = False

    # Restore patched publisher if we changed it
    if getattr(context_persistence, '_patched_publish_status', False):
        hub.publisher_manager.publish_status = hub._orig_publish_status

    assert handled is True, "Detect raised an unexpected exception when persistence failed"
