import pytest
from patient.patient import Patient
import yaml
from rclpy.parameter import Parameter
from rclpy.parameter import ParameterValue
from ament_index_python.packages import get_package_share_directory
from rclpy.parameter import Parameter
import os

@pytest.fixture
def patient_node():
    import rclpy
    rclpy.init()
    params_path = os.path.join(
        get_package_share_directory("patient"), "config", "patient_test_params.yaml"
    )
    # Create parameter overrides
    with open(params_path, 'r') as f:
        full_params = yaml.safe_load(f)

    ros_params = full_params['patient_node']['ros__parameters']
    params = [Parameter(name=k, value=v) for k, v in ros_params.items()]

    node = Patient(parameters=params)
    yield node
    node.destroy_node()
    rclpy.shutdown()



def test_patient_initial_states(patient_node):
    
    for vital in patient_node.vital_signs:
        assert patient_node.vital_states[vital] == 2


def test_calculate_datapoint_valid_range(patient_node):
    vital = patient_node.vital_signs[0]
    patient_node.risk_ranges[vital][2] = [30.0, 40.0]  # simulate a safe risk range
    patient_node._calculate_datapoint(vital, 2)
    assert 30.0 <= patient_node.vital_datapoints[vital] <= 40.0