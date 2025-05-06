import pytest
import yaml
import os
import time
import numpy as np
from ament_index_python.packages import get_package_share_directory
from rclpy.parameter import Parameter
from bsn_interfaces.msg import SensorData
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


@pytest.fixture(scope="class")
def sensor_node(request):
    import rclpy
    from sensor.sensor import Sensor

    rclpy.init()

    # Load params from YAML file - we'll use thermometer for testing
    params_path = os.path.join(
        get_package_share_directory("sensor"), "config", "thermometer.yaml"
    )
    with open(params_path, "r") as f:
        full_params = yaml.safe_load(f)

    # Prepare parameters
    ros_params = full_params["thermometer_node"]["ros__parameters"]
    params = [Parameter(name=k, value=v) for k, v in ros_params.items()]

    # Create node and assign to class
    node = Sensor("sensor_test")
    for param in params:
        node.declare_parameter(param.name, param.value)

    # Initialize the risk evaluator
    node._configure_risk_evaluator()

    # Setup a mock patient data service
    def mock_patient_service(request, response):
        response.datapoint = 37.0  # Normal body temperature
        return response

    node.cli = node.create_client(request.cls.PatientData, "get_sensor_reading")
    node.patient_service = node.create_service(
        request.cls.PatientData, "get_sensor_reading", mock_patient_service
    )

    # Create a subscription to capture published data
    request.cls.received_messages = []

    def listener_callback(msg):
        request.cls.received_messages.append(msg)

    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,
    )

    node.create_subscription(
        SensorData, f"sensor_data/{node.sensor}", listener_callback, qos_profile
    )

    request.cls.sensor_node = node
    yield node

    node.destroy_node()
    rclpy.shutdown()


@pytest.mark.usefixtures("sensor_node")
class TestSensorBehavior:
    sensor_node = None  # Will be set by fixture
    received_messages = []  # Will store published messages

    # Import the service type at class level
    from bsn_interfaces.srv import PatientData

    def test_initial_configuration(self):
        """Test that sensor is properly initialized with configuration"""
        assert self.sensor_node.sensor == "thermometer"
        assert self.sensor_node.vital_sign == "temperature"
        assert isinstance(self.sensor_node.frequency, float)
        assert self.sensor_node.risk_evaluator is not None

    def test_risk_ranges_loaded(self):
        """Test that risk ranges are properly loaded from parameters"""
        ranges = self.sensor_node.risk_evaluator.sensor_ranges[self.sensor_node.sensor]

        # Check that ranges are properly loaded
        assert len(ranges) == 5
        assert "low_risk" in ranges
        assert "mid_risk0" in ranges
        assert "mid_risk1" in ranges
        assert "high_risk0" in ranges
        assert "high_risk1" in ranges

        # Check that values are in expected format (tuples of floats)
        for range_name, range_values in ranges.items():
            assert isinstance(range_values, tuple)
            assert len(range_values) == 2
            assert all(isinstance(v, float) for v in range_values)

    def test_collect_process_transfer_cycle(self, monkeypatch):
        """Test the entire data processing cycle"""
        import rclpy

        # Run one cycle and check the result
        self.sensor_node.collect()

        # Spin to get the response processed
        for _ in range(10):
            rclpy.spin_once(self.sensor_node, timeout_sec=0.1)
            time.sleep(0.1)

        # Check that a message was published
        assert len(self.received_messages) > 0

        # Verify the message has the right fields
        msg = self.received_messages[-1]
        assert msg.sensor_type == self.sensor_node.sensor
        assert isinstance(msg.sensor_datapoint, float)
        assert isinstance(msg.risk, float)
        assert msg.risk_level in ["low", "moderate", "high", "unknown"]

    def test_risk_evaluation_low_risk(self):
        """Test risk evaluation for values in low risk range"""
        # Get the low risk range from sensor
        ranges = self.sensor_node.risk_evaluator.sensor_ranges[self.sensor_node.sensor]
        low_range = ranges["low_risk"]

        # Create a value in the middle of low risk range
        test_value = (low_range[0] + low_range[1]) / 2

        # Evaluate risk
        risk_value = self.sensor_node.risk_evaluator.evaluate_risk(
            self.sensor_node.sensor, test_value
        )

        # Check that risk is in low risk percentage range
        assert self.sensor_node.risk_evaluator.is_low_risk(risk_value)
        assert self.sensor_node.risk_evaluator.risk_label(risk_value) == "low"

    def test_risk_evaluation_medium_risk(self):
        """Test risk evaluation for values in medium risk range"""
        # Get the medium risk range from sensor
        ranges = self.sensor_node.risk_evaluator.sensor_ranges[self.sensor_node.sensor]
        mid_range = ranges["mid_risk1"]  # Use mid_risk1 as an example

        # Create a value in the middle of medium risk range
        test_value = (mid_range[0] + mid_range[1]) / 2

        # Evaluate risk
        risk_value = self.sensor_node.risk_evaluator.evaluate_risk(
            self.sensor_node.sensor, test_value
        )

        # Check that risk is in medium risk percentage range
        assert self.sensor_node.risk_evaluator.is_medium_risk(risk_value)
        assert self.sensor_node.risk_evaluator.risk_label(risk_value) == "moderate"

    def test_risk_evaluation_high_risk(self):
        """Test risk evaluation for values in high risk range"""
        # Get the high risk range from sensor
        ranges = self.sensor_node.risk_evaluator.sensor_ranges[self.sensor_node.sensor]
        high_range = ranges["high_risk1"]  # Use high_risk1 as an example

        # Create a value in the middle of high risk range
        test_value = (high_range[0] + high_range[1]) / 2

        # Evaluate risk
        risk_value = self.sensor_node.risk_evaluator.evaluate_risk(
            self.sensor_node.sensor, test_value
        )

        # Check that risk is in high risk percentage range
        assert self.sensor_node.risk_evaluator.is_high_risk(risk_value)
        assert self.sensor_node.risk_evaluator.risk_label(risk_value) == "high"

    def test_risk_at_boundaries(self):
        """Test risk evaluation at the boundaries of risk ranges"""
        ranges = self.sensor_node.risk_evaluator.sensor_ranges[self.sensor_node.sensor]

        # Test at lower boundary of low risk
        low_boundary = ranges["low_risk"][0]
        risk_at_low_boundary = self.sensor_node.risk_evaluator.evaluate_risk(
            self.sensor_node.sensor, low_boundary
        )
        assert risk_at_low_boundary >= 0

        # Test at upper boundary of low risk
        high_boundary = ranges["low_risk"][1]
        risk_at_high_boundary = self.sensor_node.risk_evaluator.evaluate_risk(
            self.sensor_node.sensor, high_boundary
        )
        assert risk_at_high_boundary <= 100

    def test_data_window_processing(self):
        """Test moving average calculation with data window"""
        # Clear the window
        self.sensor_node.data_window.clear()

        # Add test values
        test_values = [36.5, 36.7, 36.9, 37.1, 37.3]
        for val in test_values:
            self.sensor_node.data_window.append(val)

        # Calculate expected moving average
        expected_avg = sum(test_values) / len(test_values)

        # Process the data
        result = self.sensor_node.process(test_values[-1])

        # Check the result
        assert abs(result - expected_avg) < 0.0001

    def test_incomplete_data_window(self):
        """Test behavior with incomplete data window"""
        # Clear the window
        self.sensor_node.data_window.clear()

        # Add just one value
        self.sensor_node.data_window.append(36.5)

        # Process should return -1 for insufficient data
        result = self.sensor_node.process(36.5)
        assert result == -1.0

    def test_risk_displacement_calculation(self):
        """Test displacement calculation for risk evaluation"""
        evaluator = self.sensor_node.risk_evaluator

        # Test crescent displacement (0 at min, 1 at max)
        disp_crescent = evaluator.get_displacement(10.0, 20.0, 15.0, "crescent")
        assert 0.45 < disp_crescent < 0.55  # Should be around 0.5

        # Test decrescent displacement (0 at max, 1 at min)
        disp_decrescent = evaluator.get_displacement(10.0, 20.0, 15.0, "decrescent")
        assert 0.45 < disp_decrescent < 0.55  # Should be around 0.5

        # Test medium displacement (0 at middle, 1 at extremes)
        disp_medium = evaluator.get_displacement(10.0, 20.0, 15.0, "medium")
        assert disp_medium < 0.1  # Should be close to 0

        disp_medium_extreme = evaluator.get_displacement(10.0, 20.0, 20.0, "medium")
        assert disp_medium_extreme > 0.9  # Should be close to 1

    def test_percentage_conversion(self):
        """Test percentage range conversion"""
        evaluator = self.sensor_node.risk_evaluator

        # Test converting displacement to percentage range
        low_range = (0.0, 20.0)
        result = evaluator.convert_percentage(low_range[0], low_range[1], 0.5)
        assert result == 10.0

        # Test edge cases
        assert evaluator.convert_percentage(0.0, 100.0, 0.0) == 0.0
        assert evaluator.convert_percentage(0.0, 100.0, 1.0) == 100.0
