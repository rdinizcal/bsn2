from behave import given, when, then
from utils.parsers import capture_csv_data, format_entity
import subprocess
import concurrent.futures


@given("that nodes thermometer and central hub are online")
def step_given_nodes_online(context):
    # Check if the thermometer and central hub nodes are online
    result = subprocess.run(['ros2', 'node', 'list'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    node_list = result.stdout.decode('utf-8').splitlines()
    required_nodes = ['thermometer_node', 'central_hub_node']
    for node in required_nodes:
        assert node in node_list, f"Node {node} is not online"


@when("I listen to thermometer")
def step_when_listen_to_thermometer(context):
    # Capture data from the thermometer topic
    topic = '/sensor_data/thermometer'
    line_limit = 10  # Optional line limit per topic
    context.topic_data = context.topic_data if hasattr(context, 'topic_data') else {}
    with concurrent.futures.ThreadPoolExecutor() as executor:
        future = executor.submit(capture_csv_data, topic, line_limit)
        try:
            context.topic_data[topic] = future.result()
        except Exception as e:
            print(f"Error while capturing data from {topic}: {e}")


@then("g4t1 will detect new patient health status")
def step_then_detect_health_status(context):
    # Check if the TargetSystemData topic is receiving the health status
    topic_name = '/TargetSystemData'
    assert topic_name in context.topic_data, f"Target system topic {topic_name} is missing"
    target_data = context.topic_data[topic_name]

    # Validate that the thermometer data flows to the target system
    assert 'trm_data' in target_data, f"Target system topic {topic_name} does not have 'trm_data' field"
    assert any(sensor_value == target_value for sensor_value in context.topic_data['/sensor_data/thermometer']['sensor_datapoint']
               for target_value in target_data['trm_data']), (
        f"Mismatch: No matching values found between Sensor data ({context.topic_data['/sensor_data/thermometer']['sensor_datapoint']}) "
        f"and TargetSystemData trm_data ({target_data['trm_data']})"
    )

    # Validate that the patient status is updated
    assert 'patient_status' in target_data, f"Target system topic {topic_name} does not have 'patient_status' field"
    assert target_data['patient_status'], "Patient status is not being provided"
    assert all((x.replace('.', '', 1).isdigit() and 0 <= float(x) <= 100)
               for x in target_data['patient_status']), "Patient status contains invalid values"


@then("Central hub will fail to detect the new patient health status")
def step_then_fail_to_detect_health_status(context):
    # Simulate a failure in detecting health status
    print("Simulating a failure in detecting health status in the central hub.")
    assert 'internal_error' in context, "Internal processing error was not simulated"
    assert context.internal_error, "Central hub did not fail as expected"