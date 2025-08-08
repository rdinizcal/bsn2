from behave import given, when, then
import subprocess
from utils.parsers import process_real_time_topics, capture_topic_data,capture_csv_data, deactivate_node, restart_central_hub_node # , format_debug_data,,
from utils.constants import PERSISTENCE_NODES, PERSISTANCE_TOPICS
from utils.asserts import node_is_active
import time

REDUCED_SYSTEM_NODES = [
    "thermometer",
    "central_hub"
]

def check_nodes_online(node_list):
    """Check if all required nodes are online."""
    for node in node_list:
        assert node_is_active(node), f"Node {node} is not online"

@given('that persistence system is online')
def step_given_reduced_system_nodes_online(context):
    node_is_active(PERSISTENCE_NODES)

@given('nodes are online')
def step_given_all_persistence_nodes_online(context):
    node_is_active(PERSISTENCE_NODES)

@when('I listen to thermometer data')
def step_when_i_listen_to_thermometer(context):
    context.sensor_data = {}
    context.found_high_risk = []
    context.target_system_data = {}
    context.non_sensor = {}
    topics = [
        
        '/sensor_data/thermometer',
        '/collect_energy_status/thermometer',
        '/persist',
        '/log_energy_status',
        '/target_system_data'
    ]

    process_real_time_topics(context, capture_csv_data, topics)
    #print(f'non sensor data: {format_debug_data(context.non_sensor)}')
    print(context.non_sensor)
@when('I send data to collector')
def step_when_send_data_to_collector(context):
    """Simulate sending data to the collector."""
    assert 'thermometer_node' in context.non_sensor['/collect_energy_status/thermometer']['source'], f'No data detected in /collect_energy_status. {context.non_sensor['/collect_energy_status/thermometer']['source']}'

@when('collector receives collect_? topic')
def step_when_collector_receives_collect_topic(context):
    """Simulate collector receiving the collect topic."""
    assert context.data_sent_to_collector, "Collector has not received any data."

@then('the data will be in persist topic')
def step_then_data_persisted(context):
    """Simulate data persistence."""
    assert 'Status' in context.non_sensor['/persist']['type'], f"Data was not sent to collector, so it cannot be persisted. {context.non_sensor['/persist']['type']}"
    print(context.non_sensor['/persist']['type'])


@when('a database error prevents persistence')
def step_when_database_error_occurs(context):
    """Simulate a database error preventing persistence."""
    subprocess.run(['ros2', 'lifecycle', 'set', '/central_hub_node', 'shutdown'])
    #deactivate_node('central_hub_node') 
    time.sleep(5)

@then('the system must log a persistence failure')
def step_then_system_logs_failure(context):
    """Ensure the system logs a persistence failure."""
    energyStatus = capture_csv_data('/log_energy_status')
    print(f'energyStatus: {energyStatus}')
    assert 'central_hub_node' not in energyStatus['source'], f'Energy status source should be empty due to persistence failure. {energyStatus}'
    persist_topic = capture_csv_data('/persist')
    print(f'persist topic: {persist_topic}')
    assert 'central_hub_node' not in persist_topic['source'], f'Persist topic source should be empty due to persistence failure. {persist_topic}'
    success = restart_central_hub_node(context)
    assert success, "Failed to restart the central hub node after processing error"
