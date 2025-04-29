import pytest
from patient.patient import Patient

@pytest.fixture
def patient_node():
    import rclpy
    rclpy.init()
    node = Patient()
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
