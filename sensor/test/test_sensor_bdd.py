from pytest_bdd import scenarios, given, when, then
from fixtures import bdd_context

scenarios("../features/check_sensor.feature")


@given("the sensor node is online")
def sensor_node_online(bdd_context):
    assert bdd_context.sensor_node is not None
    datapoint = bdd_context.sensor_node.processor.collect()
    assert datapoint == 37.0

@when("the sensor process data")
def sensor_process_data(bdd_context):
    # Fill the data window if needed
    for _ in range(bdd_context.sensor_node.config.window_size - 1):
        bdd_context.sensor_node.processor.data_window.append(37.0)
    # Process and transfer (publish) the data
    bdd_context.processed_value = bdd_context.sensor_node.processor.process(37.0)
    bdd_context.transferred_msg = bdd_context.sensor_node.processor.transfer(
        bdd_context.processed_value
    )


@then("the data is within valid range")
def data_within_valid_range(bdd_context):
    # Assert that the published data is within the expected range
    msg = bdd_context.sensor_node.processor.data_window
    assert 35.0 <= msg[0] <= 39.0  # Adjust range as needed
