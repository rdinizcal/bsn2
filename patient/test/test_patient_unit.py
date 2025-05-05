# sensor/test/test_patient_params.py
import os
import pytest
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from patient.patient import Patient
import yaml
from ament_index_python.packages import get_package_share_directory

@pytest.fixture(scope="module")
def loaded_test_params():
    """Loads parameters from the YAML file."""
    param_file_path = os.path.join(
        get_package_share_directory("patient"), "config", "patient_test_params.yaml"
    )
    with open(param_file_path, 'r') as f:
        return yaml.safe_load(f)

@pytest.fixture
def patient_node_with_params(loaded_test_params):
    """Creates a Patient node with parameters overridden from YAML."""
    import rclpy

    #rclpy.init()
    
    # Extract parameters from YAML
    node_params = loaded_test_params.get('patient_node', {}).get('ros__parameters', {})
    
    # Create the node
    node = Patient()

    # Create parameter objects for each parameter you want to override
    params_to_set = []
    for param_name, param_value in node_params.items():
        # Determine appropriate parameter type based on value
        if isinstance(param_value, list):
            if all(isinstance(x, float) for x in param_value):
                param_type = Parameter.Type.DOUBLE_ARRAY
            elif all(isinstance(x, int) for x in param_value):
                param_type = Parameter.Type.INTEGER_ARRAY
            else:
                param_type = Parameter.Type.STRING_ARRAY
        elif isinstance(param_value, bool):
            param_type = Parameter.Type.BOOL
        elif isinstance(param_value, int):
            param_type = Parameter.Type.INTEGER
        elif isinstance(param_value, float):
            param_type = Parameter.Type.DOUBLE
        else:
            param_type = Parameter.Type.STRING
        
        # Create parameter with determined type
        params_to_set.append(Parameter(param_name, param_type, param_value))

    # Set all parameters at once
    node.set_parameters(params_to_set)
    
    # IMPORTANT: Update object attributes to match the new parameter values
    
    # Update frequency 
    node.frequency = node.get_parameter("frequency").value
    
    # Update vital signs
    node.vital_signs = node.get_parameter("vitalSigns").value
    
    # Re-initialize data structures 
    for vital in node.vital_signs:
        # Update change rates if parameter exists
        change_param = f"{vital}_Change"
        if change_param in node_params:
            if vital not in node.change_rates:
                node.change_rates[vital] = 0
            node.change_rates[vital] = 1.0 / node.get_parameter(change_param).value
        
        # Update offsets if parameter exists
        offset_param = f"{vital}_Offset"
        if offset_param in node_params:
            if vital not in node.offsets:
                node.offsets[vital] = 0
            node.offsets[vital] = node.get_parameter(offset_param).value
        
        # Generate state matrix and risk ranges (these will use the updated parameters)
        if vital not in node.transition_matrix_states:
            node.transition_matrix_states[vital] = {}
        node.transition_matrix_states[vital] = node._gen_sensor_state_matrix(vital)
        
        if vital not in node.risk_ranges:
            node.risk_ranges[vital] = {}
        node.risk_ranges[vital] = node._set_up_sensor_risk_ranges(vital)
    
    yield node
    node.destroy_node()
    #rclpy.shutdown()

def test_patient_frequency_override(patient_node_with_params):
    """Test if the frequency parameter is loaded from YAML."""
    expected_frequency = 5.0  # Value from your patient_test_params.yaml
    assert patient_node_with_params.frequency == expected_frequency

def test_patient_vital_signs_override(patient_node_with_params):
    """Test if the vitalSigns parameter is loaded from YAML."""
    expected_vital_signs = ["oxigenation", "heart_rate", "temperature", "abps", "abpd", "glucose"]
    assert patient_node_with_params.vital_signs == expected_vital_signs

def test_patient_change_rate_override(patient_node_with_params):
    """Test if a specific change rate parameter is loaded from YAML."""
    expected_temp_change = 0.1
    assert patient_node_with_params.change_rates["temperature"] == 1 / expected_temp_change

def test_patient_offset_override(patient_node_with_params):
    """Test if a specific offset parameter is loaded from YAML."""
    expected_abps_offset = 10.0
    assert patient_node_with_params.offsets["abps"] == expected_abps_offset

def test_patient_state_matrix_override(patient_node_with_params):
    """Test if a specific state matrix parameter is loaded from YAML."""
    expected_oxigenation_state2 = [0.0, 0.0, 0.9, 0.08, 0.02]
    assert patient_node_with_params.transition_matrix_states["oxigenation"][2] == expected_oxigenation_state2

def test_patient_risk_range_override(patient_node_with_params):
    """Test if a specific risk range parameter is loaded from YAML."""
    expected_heart_rate_low_risk = [85.0, 97.0]
    assert patient_node_with_params.risk_ranges["heart_rate"][2] == expected_heart_rate_low_risk