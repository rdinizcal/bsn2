from pytest_bdd import scenarios, given, when, then

# Link to your feature file
scenarios('../features/check_sensor.feature')

@given('the sensor node is online')
def sensor_node_online():
    # Setup code to ensure sensor node is running
    assert sensor_node is not None

@when('the sensor collects data')
def sensor_collects_data():
    # Simulate or trigger data collection
    pass

@then('the sensor publishes the data')
def sensor_publishes_data():
    # Assert that data was published (mock or check topic)
    pass

@then('the data is within valid range')
def data_within_valid_range():
    # Assert that published data is valid
    pass