import os
import pytest
import yaml
import rclpy
from rclpy.parameter import Parameter
from ament_index_python.packages import get_package_share_directory
from patient.patient import Patient
@pytest.fixture(scope="session")
def patient_node_bdd():
    if not rclpy.ok():
        rclpy.init()

    # Load YAML params
    params_path = os.path.join(
        get_package_share_directory("patient"), "config", "patient_test_params.yaml"
    )
    with open(params_path, "r") as f:
        full_params = yaml.safe_load(f)

    ros_params = full_params["patient_node"]["ros__parameters"]
    params = [Parameter(name=k, value=v) for k, v in ros_params.items()]

    # Create node and assign to class
    node = Patient(parameters=params)
    #request.cls.patient_node = node
    yield node
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
        
@pytest.fixture(scope="class")
def patient_node(request):
    import rclpy
    if not rclpy.ok():
        rclpy.init()

    # Load YAML params
    params_path = os.path.join(
        get_package_share_directory("patient"), "config", "patient_test_params.yaml"
    )
    with open(params_path, "r") as f:
        full_params = yaml.safe_load(f)

    ros_params = full_params["patient_node"]["ros__parameters"]
    params = [Parameter(name=k, value=v) for k, v in ros_params.items()]

    # Create node and assign to class
    node = Patient(parameters=params)
    request.cls.patient_node = node
    yield node
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
