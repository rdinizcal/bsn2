from utils.parsers import capture_topic_data, capture_csv_data, restart_central_hub_node
from utils.asserts import node_is_active, check_time_performance
from behave import given, when, then
import time
@when('{sensor_name} sends data with high risk')
def step_when_high_risk_data_sent(context, sensor_name):
    # implementation made by patient data service 
    node_is_active('/patient_node')
    # Convert strings to floats, then find max
    max_temp = max(float(x) for x in context.topic_data['/sensor_data/thermometer']['sensor_datapoint'])
    assert max_temp > 39.0, f'topic sensor_data/thermometer does not have high risk data: {context.topic_data["/sensor_data/thermometer"]["sensor_datapoint"]}'
@then('Central hub will detect an emergency in less than 250 ms')
def step_then_g4t1_detects_emergency(context):
    max_temp = max(float(x) for x in context.topic_data['/sensor_data/thermometer']['sensor_datapoint'])
    assert max_temp > 39.0, f'topic sensor_data/thermometer does not have high risk data: {context.topic_data["/sensor_data/thermometer"]["sensor_datapoint"]}'
    assert check_time_performance(context.topic_data, context.topic_data['/target_system_data'],
                                  '/sensor_data/thermometer','trm_data', 'sensor_datapoint')
@when('{node_name} sends low-risk data with high frequency')
def step_when_overloaded_data_sent(context, node_name):
    topic = f'/{node_name}_data'
    capture_topic_data(context, ['sensor_data/thermometer'])

    
    assert False

@then('Central Hub will experience delayed emergency detection')
def step_then_g4t1_might_delay_detection(context):
    assert False