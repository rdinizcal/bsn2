import sys
import os

# Add the steps directory to Python path
steps_dir = os.path.dirname(__file__)
if steps_dir not in sys.path:
    sys.path.insert(0, steps_dir)

# Now use absolute imports
from utils.parsers import capture_topic_data, capture_csv_data
from utils.asserts import node_is_active, check_time_performance

from behave import given, when, then
import time

@when('{sensor_name} sends data with high risk')
def step_when_high_risk_data_sent(context, sensor_name):
    # implementation made by patient data service 
    node_is_active('/patient_node')
    # Convert strings to floats, then find max
    time.sleep(10)
    ('high' in context.topic_data['/sensor_data/thermometer']['risk_level'] 
           or 'moderate' in context.topic_data['/sensor_data/thermometer']['risk_level'])

@then('Central hub will detect an emergency in less than 250 ms')
def step_then_g4t1_detects_emergency(context):
    max_temp = max(float(x) for x in context.topic_data['/sensor_data/thermometer']['sensor_datapoint'])
    assert check_time_performance(context.topic_data, context.topic_data['/target_system_data'],
                                  '/sensor_data/thermometer','trm_data', 'sensor_datapoint')

@when('{node_name} sends low-risk data with high frequency')
def step_when_overloaded_data_sent(context, node_name):
    thermometer_topic = capture_csv_data('/sensor_data/thermometer', 10, 30)
    assert thermometer_topic['sensor_datapoint'], f'no risk found: {thermometer_topic}'

@then('Central Hub will experience delayed emergency detection')
def step_then_g4t1_might_delay_detection(context):
    #detects no high emergency risk

    target_system_topic = capture_csv_data('/target_system_data', line_limit=10, timeout=30)
    assert any(50.0 > float(x) for x in target_system_topic['patient_status']), f'no low or moderate risk data found: {target_system_topic}'
