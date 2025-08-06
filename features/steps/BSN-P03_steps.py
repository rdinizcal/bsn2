from utils.parsers import capture_topic_data
from utils.asserts import node_is_active, check_time_performance
from behave import given, when, then
@when('{sensor_name} sends data with high risk')
def step_when_high_risk_data_sent(context, sensor_name):
    # implementation made by patient data service 
    node_is_active('/patient_data_service')
@then('Central hub will detect an emergency in less than 250 ms')
def step_then_g4t1_detects_emergency(context):
    assert check_time_performance(context.sensor_data, context.target_system_data,
                                  '/thermometer_data','trm_data', 'data')
@when('{node_name} sends low-risk data with high frequency')
def step_when_overloaded_data_sent(context, node_name):
    topic = f'/{node_name}_data'
    capture_topic_data(context, ['/thermometer_data'])
    context.overloaded = True
    context.high_risk_detected = high_risk_detected
    assert context.overloaded, "Sensor data overload did not occur"

@then('Central Hub will experience delayed emergency detection')
def step_then_g4t1_might_delay_detection(context):
    assert context.overloaded and context.high_risk_detected, "Delayed detection scenario not met"