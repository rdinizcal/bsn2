from pytest_bdd import scenarios, given, when, then
from fixtures import context
from shared_test_methods import SharedSensorTests

scenarios("../features/check_sensor.feature")


@given("the sensor node is online")
def sensor_node_online(context):
    assert context.sensor_node is not None

    # Use shared test method to verify collect works
    SharedSensorTests.assert_collect_works(context.sensor_node)

    # Also verify all components exist
    SharedSensorTests.assert_processor_components_exist(context.sensor_node)


@when("the sensor process data")
def sensor_process_data(context):
    # Use shared test method to verify processing works with filled window
    context.processed_value = SharedSensorTests.assert_process_with_filled_window_works(
        context.sensor_node
    )

    # Use shared test method to verify transfer works
    SharedSensorTests.assert_transfer_works(
        context.sensor_node, context.processed_value
    )

    # Store the transferred message result (if any)
    context.transferred_msg = context.sensor_node.processor.transfer(
        context.processed_value
    )


@then("the data is within valid range")
def data_within_valid_range(context):
    # Use shared test method to verify data range
    SharedSensorTests.assert_data_within_valid_range(context.sensor_node, 35.0, 39.0)

    # Use shared test method to verify complete data flow
    SharedSensorTests.assert_sensor_data_flow_works(context.sensor_node)


# Additional BDD steps that leverage the shared methods
@then("the sensor can handle incomplete data window")
def sensor_handles_incomplete_window(context):
    """Test that sensor properly handles incomplete data window"""
    SharedSensorTests.assert_process_with_incomplete_window_fails(context.sensor_node)


@then("the risk assessment works for normal temperature")
def risk_assessment_normal_temp(context):
    """Test risk assessment for normal temperature values"""
    SharedSensorTests.assert_risk_evaluation_works(context.sensor_node, 37.0, "low")


@then("the risk assessment works for high temperature")
def risk_assessment_high_temp(context):
    """Test risk assessment for high temperature values"""
    SharedSensorTests.assert_risk_evaluation_works(context.sensor_node, 41.0, "high")


@then("the risk assessment works for low temperature")
def risk_assessment_low_temp(context):
    """Test risk assessment for low temperature values"""
    SharedSensorTests.assert_risk_evaluation_works(
        context.sensor_node, 33.0, "moderate"
    )
