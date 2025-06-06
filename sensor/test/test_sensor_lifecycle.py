import rclpy
from rclpy.executors import SingleThreadedExecutor
import pytest
import time
import threading
from unittest.mock import MagicMock
from bsn_interfaces.srv import PatientData
from sensor.sensor import Sensor
from rclpy.lifecycle import TransitionCallbackReturn

@pytest.fixture
def setup_sensor():
    rclpy.init()
    
    # Create sensor node with test parameters
    node = Sensor("test_sensor")
    
    # Create mock service node
    mock_service_node = rclpy.create_node("mock_service")
    
    # Set test parameters
    node.declare_parameter("sensor", "thermometer")
    node.declare_parameter("vital_sign", "temperature")
    node.declare_parameter("frequency", "1.0")
    node.declare_parameter("battery_id", "test_battery")
    node.declare_parameter("battery_capacity", 100.0)
    node.declare_parameter("battery_level", 100.0)
    node.declare_parameter("battery_unit", 0.01)
    
    # Risk parameters
    node.declare_parameter("lowrisk_percent", "0,20")
    node.declare_parameter("midrisk_percent", "21,65")
    node.declare_parameter("highrisk_percent", "66,100")
    node.declare_parameter("HighRisk0", "0,33")
    node.declare_parameter("MidRisk0", "33,36")
    node.declare_parameter("LowRisk", "36,38")
    node.declare_parameter("MidRisk1", "38,40")
    node.declare_parameter("HighRisk1", "40,100")
    
    def mock_service_callback(request, response):
        # You can make this return different values based on request
        if request.vital_sign == "temperature":
            response.datapoint = 37.0
        elif request.vital_sign == "pulse":
            response.datapoint = 72.0
        else:
            response.datapoint = 100.0
        return response
        
    mock_service = mock_service_node.create_service(
        PatientData, 
        "/get_sensor_reading", 
        mock_service_callback
    )
    
    # Setup executor for spinning
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(mock_service_node)
    
    # Configure the node
    node.trigger_configure()
    time.sleep(0.1)
    
    # Setup threading
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    
    # Store mock service node reference
    node.mock_service_node = mock_service_node
    
    yield node
    
    # Cleanup
    node.destroy_node()
    mock_service_node.destroy_node()
    rclpy.shutdown()

# Pure pytest-style tests (without TestCase class)
def test_configuration(setup_sensor):
    """Test sensor configuration"""
    node = setup_sensor
    
    # Test basic configuration
    assert node.config.sensor == "thermometer"
    assert node.config.vital_sign == "temperature"
    assert node.processor is not None
    assert node.battery_manager is not None
    assert node.publisher_manager is not None
    assert node.risk_manager is not None

def test_lifecycle_transitions(setup_sensor):
    """Test lifecycle state transitions"""
    node = setup_sensor
    
    # Activate
    result = node.trigger_activate()
    assert result == TransitionCallbackReturn.SUCCESS
    assert node.active
    
    # Deactivate
    result = node.trigger_deactivate()
    assert result == TransitionCallbackReturn.SUCCESS
    assert not node.active
    
    # Back to active for other tests
    node.trigger_activate()

def test_data_collection(setup_sensor):
    """Test data collection through processor"""
    node = setup_sensor
    node.trigger_activate()
    
    # Call collect through processor
    datapoint = node.processor.collect()
    assert datapoint == 37.0

def test_data_processing(setup_sensor):
    """Test data processing"""
    node = setup_sensor
    node.trigger_activate()
    
    # Clear and fill data window with test values
    node.processor.data_window.clear()
    for i in range(5):
        node.processor.data_window.append(36.5 + i * 0.1)
    
    # Process a value
    result = node.processor.process(37.0)
    
    # Should be average of window values
    assert result > 36.0
    assert result < 38.0

def test_risk_evaluation(setup_sensor):
    """Test risk evaluation"""
    node = setup_sensor
    node.trigger_activate()
    
    # Test risk values for different temperatures
    test_cases = [
        (32.0, "high"),    # Too low
        (34.0, "moderate"),  # Below normal
        (37.0, "low"),     # Normal
        (39.0, "moderate"),  # Above normal
        (41.0, "high")     # Too high
    ]
    
    for value, expected_label in test_cases:
        risk_value = node.risk_manager.evaluate_risk(value)
        risk_label = node.risk_manager.get_risk_label(risk_value)
        assert risk_label == expected_label, \
            f"Expected risk label {expected_label} for value {value}, got {risk_label}"

def test_battery_management(setup_sensor):
    """Test battery management"""
    node = setup_sensor
    node.trigger_activate()
    
    # Get initial level
    initial_level = node.battery_manager.battery.current_level
    
    # Consume
    node.battery_manager.consume(10.0)
    
    # Should be lower
    assert node.battery_manager.battery.current_level < initial_level