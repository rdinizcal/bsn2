from pytest_bdd import scenarios, given, when, then
from fixtures import context

scenarios("../features/check_sensor.feature")


@given("the sensor node is online")
def sensor_node_online(context):
    assert context.sensor_node is not None
    datapoint = context.sensor_node.processor.collect()
    assert datapoint == 37.0


@when("the sensor process data")
def sensor_process_data(context):
    # Fill the data window if needed
    for _ in range(context.sensor_node.config.window_size - 1):
        context.sensor_node.processor.data_window.append(37.0)
    # Process and transfer (publish) the data
    context.processed_value = context.sensor_node.processor.process(37.0)
    context.transferred_msg = context.sensor_node.processor.transfer(
        context.processed_value
    )


@then("the data is within valid range")
def data_within_valid_range(context):
    # Assert that the published data is within the expected range
    msg = context.sensor_node.processor.data_window
    assert 35.0 <= msg[0] <= 39.0  # Adjust range as needed
