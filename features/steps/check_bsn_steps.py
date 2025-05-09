from behave import given, when, then
from utils.parsers import capture_topic_data
from utils.asserts import count_matching_elements
import subprocess


@given("that all sensors and central hub nodes are online")
def step_given_all_nodes_online(context):
    # Check if all sensor and central hub nodes are online
    result = subprocess.run(
        ["ros2", "node", "list"], stdout=subprocess.PIPE, stderr=subprocess.PIPE
    )
    node_list = result.stdout.decode("utf-8").splitlines()
    required_nodes = ["/thermometer_node", "/ecg_node", "/central_hub_node"]
    for node in required_nodes:
        assert node in node_list, f"Node {node} is not online"


@when("I listen to sensors data")
def step_when_listen_to_sensors_data(context):
    # Capture data from sensor topics
    topics = ["/sensor_data/thermometer", "/sensor_data/ecg", "/target_system_data"]
    line_limit = 10  # Optional line limit per topic
    context.topic_data = context.topic_data if hasattr(context, "topic_data") else {}
    capture_topic_data(context, topics, line_limit)


@then("Sensors will process the risks")
def step_then_sensors_process_risks(context):
    # Check if sensors process the risks
    assert any(context.topic_data.values()), "No risk data found in sensor topics."
    for topic, data in context.topic_data.items():
        assert (
            "risk" in data and data["risk"]
        ), f"No risk data detected in topic {topic}"


@then("Central hub will process the risk")
def step_then_central_hub_process_risk(context):
    # Check if the central hub processes the risks
    topic_name = "/target_system_data"
    assert (
        topic_name in context.topic_data
    ), f"Central hub topic {topic_name} is missing"
    target_data = context.topic_data[topic_name]
    assert (
        "patient_status" in target_data
    ), f"Central hub topic {topic_name} does not have 'patient_status' field"
    assert target_data[
        "patient_status"
    ], "Central hub did not process the patient status"


@then("Sensors will process the data")
def step_then_sensors_process_data(context):
    # Check if sensors process the data
    assert any(context.topic_data.values()), "No data found in sensor topics."
    for topic, data in context.topic_data.items():
        if topic != "/target_system_data":
            assert (
                "sensor_datapoint" in data and data["sensor_datapoint"]
            ), f"No sensor data detected in topic {topic}"


@then("Central hub will receive data from sensors")
def step_then_central_hub_receive_data(context):
    # Check if the central hub receives data from sensors
    topic_name = "/target_system_data"
    assert (
        topic_name in context.topic_data
    ), f"Central hub topic {topic_name} is missing"
    target_data = context.topic_data[topic_name]
    assert count_matching_elements(
        context.topic_data["/sensor_data/thermometer"]["sensor_datapoint"],
        target_data["trm_data"],
    ), f"Mismatch: No matching values found between Sensor data and TargetSystemData"

    print(f"Central hub received data: {target_data}")
