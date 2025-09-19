from pytest_bdd import scenarios, given, when, then
from fixtures import sensor_node
# Link to your feature file
scenarios('../features/check_sensor.feature')

@given('the sensor node is online')
def sensor_node_online(sensor_node, request):
    # Setup code to ensure sensor node is running
    assert sensor_node is not None
    assert request is not None

@when('the sensor collects data')
def sensor_collects_data(sensor_node, request):
    # Simulate or trigger data collection
    sensor_node = request.sensor_node
    assert 37.0 == sensor_node.data_processor.collect()

@then('the sensor publishes the data')
def sensor_publishes_data(sensor_node, request):
    # Assert that data was published (mock or check topic)
    assert 37.0 == sensor_node.data_processor.process(37.0)

@then('the data is within valid range')
def data_within_valid_range(sensor_node, request):
    # Assert that published data is valid
    assert sensor_node.data_processor.transfer()