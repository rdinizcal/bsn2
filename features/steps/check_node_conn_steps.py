import subprocess
from utils.parsers import get_rosnode_info_ros2
from behave import given, when, then


@given("the {node_name} node is online")
def step_impl(context, node_name):
    # Initialize node_info_dict if it doesn't exist
    if not hasattr(context, "node_info_dict"):
        context.node_info_dict = {}

    # Check if the node is online
    result = subprocess.run(
        ["ros2", "node", "list"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=5,
    )
    node_list = result.stdout.decode("utf-8").splitlines()
    assert node_name in node_list, f"{node_name} is not online"

    # Get node info and store it in context.node_info_dict with the node_name as key
    result = subprocess.run(
        ["ros2", "node", "info", node_name],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=5,
    )
    if result.returncode != 0:
        raise Exception(f"Error getting node info: {result.stderr.decode('utf-8')}")

    output = result.stdout.decode("utf-8")
    lines = output.splitlines()
    context.node_info_dict[node_name] = get_rosnode_info_ros2(
        lines
    )  # Store node info for each node


@when("I check if node {node_name} publishes to {topics}")
def step_impl(context, node_name, topics):
    # Access the node_info for the specific node from context.node_info_dict
    node_info = context.node_info_dict[node_name]

    # Split topics into a list
    topic_list = [t.strip() for t in topics.split(",")]

    for topic in topic_list:
        found = any(pub["topic"] == topic for pub in node_info["publishers"])
        assert found, f"{node_name} does not publish to {topic}"


@then("{node_name} should have {topics} subscribed")
def step_impl(context, node_name, topics):
    # Access the node_info for the specific node from context.node_info_dict
    node_info = context.node_info_dict[node_name]

    # Split topics into a list
    topic_list = [t.strip() for t in topics.split(",")]

    for topic in topic_list:
        found = any(sub["topic"] == topic for sub in node_info["subscribers"])
        assert found, f"{node_name} does not subscribe to {topic}"
