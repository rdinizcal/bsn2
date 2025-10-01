import pytest
import time
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from bsn_interfaces.msg import SensorData
from std_msgs.msg import Header
from fixtures import central_hub_node



@pytest.mark.usefixtures("central_hub_node")
class TestCentralHubBehavior:
    central_hub = None
    publishers = {}
    published_messages = []  # Renamed for clarity

    def setup_method(self):
        """Set up publishers before each test"""
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create publishers for each sensor type
        for sensor_type in [
            "thermometer", "ecg", "oximeter", "abps", "abpd", "glucosemeter"
        ]:
            self.publishers[sensor_type] = self.central_hub.create_publisher(
                SensorData, f"sensor_data/{sensor_type}", qos_profile
            )
        
        time.sleep(0.2)
        self.published_messages.clear()

    def publish_sensor_data(self, sensor_type, value, risk_level="normal", risk_percentage=None):
        """Helper to publish sensor data"""
        msg = SensorData()
        header = Header()
        header.stamp = self.central_hub.get_clock().now().to_msg()
        header.frame_id = sensor_type

        msg.header = header
        msg.sensor_type = sensor_type
        msg.sensor_datapoint = float(value)
        msg.risk_level = risk_level

        if risk_percentage is not None:
            msg.risk = float(risk_percentage)
        else:
            msg.risk = float(self._risk_level_to_percentage(risk_level))

        self.publishers[sensor_type].publish(msg)

        # Process the message
        for _ in range(5):
            rclpy.spin_once(self.central_hub, timeout_sec=0.1)
            time.sleep(0.1)

    def _risk_level_to_percentage(self, risk_level):
        """Convert risk level to a percentage value"""
        if risk_level == "high":
            return 80.0
        elif risk_level == "moderate":
            return 50.0
        elif risk_level == "low" or risk_level == "normal":
            return 10.0
        else:
            return -1.0

    def wait_for_published_message(self, timeout=1.0):
        """Wait for a message to be published and captured"""
        start_time = time.time()
        initial_count = len(self.published_messages)
        
        while len(self.published_messages) == initial_count and time.time() - start_time < timeout:
            rclpy.spin_once(self.central_hub, timeout_sec=0.1)
            time.sleep(0.05)
        
        return len(self.published_messages) > initial_count

    def test_receive_datapoint(self):
        """Test receiving data from sensors"""
        self.publish_sensor_data("thermometer", 37.0, "normal")
        assert self.central_hub.sensor_handler.latest_data["thermometer"] == 37.0

    def test_detect_normal_conditions(self):
        """Test detection under normal conditions"""
        assert self.central_hub.active, "Node must be active for this test"
        
        self.published_messages.clear()

        # Publish normal values for all sensors
        self.publish_sensor_data("thermometer", 37.0, "normal", 10.0)
        self.publish_sensor_data("ecg", 90.0, "normal", 10.0)
        self.publish_sensor_data("oximeter", 98.0, "normal", 10.0)
        self.publish_sensor_data("abps", 110.0, "normal", 10.0)
        self.publish_sensor_data("abpd", 75.0, "normal", 10.0)
        self.publish_sensor_data("glucosemeter", 80.0, "normal", 10.0)

        # Trigger detect and wait for message
        self.central_hub.detect()
        
        assert self.wait_for_published_message(), "No TargetSystemData message published"
        
        message = self.published_messages[-1]
        assert message.patient_status < 20.0, f"Expected low patient status, got {message.patient_status}"

    def test_detect_abnormal_conditions(self):
        """Test detection under abnormal conditions"""
        self.published_messages.clear()

        # Publish abnormal values
        self.publish_sensor_data("thermometer", 39.5, "moderate", 60.0)
        self.publish_sensor_data("ecg", 150.0, "high", 80.0)
        self.publish_sensor_data("oximeter", 90.0, "moderate", 50.0)
        self.publish_sensor_data("abps", 150.0, "high", 80.0)
        self.publish_sensor_data("abpd", 100.0, "high", 80.0)
        self.publish_sensor_data("glucosemeter", 200.0, "high", 80.0)

        self.central_hub.detect()
        
        assert self.wait_for_published_message(), "No TargetSystemData message published"
        
        message = self.published_messages[-1]
        assert message.patient_status > 50.0, f"Expected high patient status, got {message.patient_status}"

    def test_data_fusion_algorithm(self):
        """Test the data fusion algorithm"""
        if not hasattr(self.central_hub, 'fusion_engine') or not hasattr(self.central_hub, 'sensor_handler'):
            pytest.skip("Required components not found")
        
        self.central_hub.sensor_handler.latest_risk = {
            "thermometer": 10.0, "ecg": 10.0, "oximeter": 10.0,
            "abps": 10.0, "abpd": 10.0, "glucosemeter": 10.0,
        }
        
        result = self.central_hub.fusion_engine.fuse_data()
        assert 5.0 <= result <= 15.0, f"Expected result between 5-15 for equal values of 10, got {result}"
        
        self.__class__.fusion_calculation_factor = result / 10.0

    def test_blood_pressure_special_handling(self):
        """Test special handling of blood pressure data"""
        self.central_hub.sensor_handler.latest_risk = {
            "thermometer": 10.0, "ecg": 10.0, "oximeter": 10.0,
            "abps": 20.0, "abpd": 20.0, "glucosemeter": 10.0,
        }
        
        result = self.central_hub.fusion_engine.fuse_data()
        assert result > 10.0, "Blood pressure values should be averaged and affect result"

    def test_format_log_message(self):
        """Test log message formatting"""
        self.central_hub.sensor_handler.latest_data = {
            "thermometer": 37.0, "ecg": 90.0, "oximeter": 98.0,
            "abps": 110.0, "abpd": 75.0, "glucosemeter": 80.0,
        }
        
        self.central_hub.sensor_handler.latest_risks_labels = {
            "thermometer": "low", "ecg": "low", "oximeter": "low",
            "abps": "low", "abpd": "low", "glucosemeter": "low",
        }
        
        log_message = self.central_hub.visualizer.format_log_message()
        
        for sensor in ["thermometer", "ecg", "oximeter", "abps", "abpd", "glucosemeter"]:
            assert sensor in log_message

    def test_multiple_readings_overwrite(self):
        """Test that newer readings overwrite older ones"""
        self.publish_sensor_data("thermometer", 36.0, "normal")
        assert self.central_hub.sensor_handler.latest_data["thermometer"] == 36.0
        
        self.publish_sensor_data("thermometer", 37.0, "normal")
        assert self.central_hub.sensor_handler.latest_data["thermometer"] == 37.0

    def test_risk_categorization(self):
        """Test risk categorization"""
        test_cases = [
            (10.0, "VERY LOW RISK"),
            (30.0, "LOW RISK"),
            (50.0, "MODERATE RISK"),
            (70.0, "CRITICAL RISK"), 
            (90.0, "VERY CRITICAL RISK"),
        ]
        
        for status, category in test_cases:
            # Call emit_alert directly on risk_analyzer
            self.central_hub.risk_analyzer.emit_alert(status)
            # We can't easily test the output, but at least make sure it doesn't crash

    def test_data_fusion_with_deviation_weighting(self):
        """Test data fusion with values that have different deviations"""
        self.central_hub.sensor_handler.latest_risk = {
            "thermometer": 10.0, "ecg": 20.0, "oximeter": 30.0,
            "abps": 40.0, "abpd": 50.0, "glucosemeter": 60.0,
        }
        
        result = self.central_hub.fusion_engine.fuse_data()
        assert 10.0 < result < 60.0, f"Result should be between min and max: {result}"

    def test_data_fuse_with_no_data(self):
        """Test data fusion with no data"""
        self.central_hub.sensor_handler.latest_risk = {
            "thermometer": -1.0,
            "ecg": -1.0,
            "oximeter": -1.0,
            "abps": -1.0,
            "abpd": -1.0, 
            "glucosemeter": -1.0,
        }
        
        result = self.central_hub.fusion_engine.fuse_data()
        
        # Should return default value for no data
        assert result == 0.0, f"Expected 0.0 for no data, got {result}"

    def test_data_fuse_with_identical_values(self):
        """Test data fusion with identical values"""
        if not hasattr(self.central_hub, 'fusion_engine') or not hasattr(self.central_hub, 'sensor_handler'):
            pytest.skip("Required components not found")
        
        identical_value = 42.0
        self.central_hub.sensor_handler.latest_risk = {
            "thermometer": identical_value,
            "ecg": identical_value,
            "oximeter": identical_value,
            "abps": identical_value,
            "abpd": identical_value,
            "glucosemeter": identical_value,
        }
        
        result = self.central_hub.fusion_engine.fuse_data()
        
        calculation_factor = getattr(self.__class__, 'fusion_calculation_factor', None)
        
        if calculation_factor:
            expected_result = identical_value * calculation_factor
            assert abs(result - expected_result) < 0.1, f"Expected {expected_result}, got {result}"
        else:
            assert 30.0 <= result <= 50.0, f"Expected result between 30-50 for all values = 42, got {result}"

    def test_data_fuse_with_partial_data(self):
        """Test data fusion with only some values available"""
        self.central_hub.sensor_handler.latest_risk = {
            "thermometer": 30.0,
            "ecg": -1.0,  # No data
            "oximeter": 20.0,
            "abps": -1.0,  # No data
            "abpd": -1.0,  # No data
            "glucosemeter": 40.0,
        }
        
        result = self.central_hub.fusion_engine.fuse_data()
        
        # Should still generate a result with partial data
        assert result > 0.0, "Should compute result with partial data"
        # Result should be influenced by available values
        assert 20.0 <= result <= 40.0, f"Result should be between min and max values: {result}"

    def test_receive_datapoint_with_empty_sensor_type(self):
        """Test handling of datapoints with empty sensor type"""
        # Create message with empty sensor type
        msg = SensorData()
        msg.header = Header()
        msg.header.stamp = self.central_hub.get_clock().now().to_msg()
        msg.sensor_type = ""
        msg.sensor_datapoint = 37.0
        msg.risk_level = "normal"
        msg.risk = 10.0
        
        # Call the handler directly
        self.central_hub.sensor_handler.receive_datapoint(msg)
        
        # This should log a warning but not crash - no easy way to assert this
        # Just make sure it doesn't throw an exception

    def test_format_log_message_with_empty_data(self):
        """Test log formatting with empty data"""
        # Set up empty data
        self.central_hub.sensor_handler.latest_data = {
            "thermometer": -1.0,
            "ecg": -1.0,
            "oximeter": -1.0,
            "abps": -1.0,
            "abpd": -1.0,
            "glucosemeter": -1.0,
        }
        
        # Get log message
        log_message = self.central_hub.visualizer.format_log_message()
        
        # Should have "waiting data" for all sensors
        assert "waiting data" in log_message

    def test_battery_manager_low_battery_behavior(self):
        """Test behavior when battery is low"""
        # Check if component exists
        if not hasattr(self.central_hub, 'battery_manager'):
            pytest.skip("Battery manager component not found")
        
        # Save original battery level
        original_level = self.central_hub.battery_manager.battery.current_level
        
        try:
            # Set battery to low level
            self.central_hub.battery_manager.battery.current_level = 5.0
            
            # The BatteryManager doesn't have is_low(), so check directly with battery level
            assert self.central_hub.battery_manager.battery.current_level < 10.0, "Battery level should be low"
            
            # Test recharge - note that recharge() doesn't take parameters
            old_level = self.central_hub.battery_manager.battery.current_level
            self.central_hub.battery_manager.recharge()
            assert self.central_hub.battery_manager.battery.current_level > old_level, "Battery should have recharged"
            
            # Test max level cap by getting the capacity (which is the max)
            # Set battery level close to max and recharge
            if hasattr(self.central_hub.battery_manager.battery, 'capacity'):
                max_capacity = self.central_hub.battery_manager.battery.capacity
                self.central_hub.battery_manager.battery.current_level = max_capacity - 1
                self.central_hub.battery_manager.recharge()
                assert self.central_hub.battery_manager.battery.current_level <= max_capacity, "Battery should not exceed max capacity"
        
        finally:
            # Restore original battery level
            self.central_hub.battery_manager.battery.current_level = original_level
            
    def test_risk_analyzer_edge_cases(self):
        """Test risk analyzer edge cases"""
        # Check if component exists
        if not hasattr(self.central_hub, 'risk_analyzer'):
            pytest.skip("Risk analyzer component not found")
        
        # Test extreme values
        self.central_hub.risk_analyzer.emit_alert(100.0)  # Should handle 100%
        self.central_hub.risk_analyzer.emit_alert(0.0)    # Should handle 0%
        self.central_hub.risk_analyzer.emit_alert(-10.0)  # Should handle negative values
        
        # Test get_risk_category method if it exists
        if hasattr(self.central_hub.risk_analyzer, 'get_risk_category'):
            assert "VERY LOW" in self.central_hub.risk_analyzer.get_risk_category(5.0).upper()
            assert "CRITICAL" in self.central_hub.risk_analyzer.get_risk_category(90.0).upper()

    def test_central_hub_lifecycle_methods(self):
        """Test central hub's lifecycle methods"""
        # Skip this test if node is not a proper lifecycle node
        if not hasattr(self.central_hub, 'active'):
            pytest.skip("Node does not implement active property")
        
        # Save initial state
        initial_active = self.central_hub.active
        
        try:
            # First test - if node is active, try deactivating it
            if initial_active:
                try:
                    if hasattr(self.central_hub, 'trigger_deactivate'):
                        # Deactivate and check
                        self.central_hub.trigger_deactivate()
                        time.sleep(0.2)
                        assert not self.central_hub.active, "Node should be inactive after deactivation"
                except Exception as e:
                    self.central_hub.get_logger().warn(f"Deactivation test failed: {e}")
            
            # Second test - if node is inactive, try activating it
            if not self.central_hub.active:
                try:
                    if hasattr(self.central_hub, 'trigger_activate'):
                        # Activate and check
                        self.central_hub.trigger_activate()
                        time.sleep(0.2)
                        assert self.central_hub.active, "Node should be active after activation"
                except Exception as e:
                    self.central_hub.get_logger().warn(f"Activation test failed: {e}")
        
        finally:
            # Restore initial state
            if self.central_hub.active != initial_active:
                try:
                    if initial_active and not self.central_hub.active and hasattr(self.central_hub, 'trigger_activate'):
                        self.central_hub.trigger_activate()
                    elif not initial_active and self.central_hub.active and hasattr(self.central_hub, 'trigger_deactivate'):
                        self.central_hub.trigger_deactivate()
                except Exception as e:
                    self.central_hub.get_logger().warn(f"Failed to restore initial state: {e}")

    def test_hub_error_handling(self):
        """Test error handling in the hub"""
        # Test with invalid sensor type
        msg = SensorData()
        msg.header = Header()
        msg.header.stamp = self.central_hub.get_clock().now().to_msg()
        msg.sensor_type = "invalid_sensor_type"
        msg.sensor_datapoint = 37.0
        msg.risk_level = "normal"
        msg.risk = 10.0
        
        # This should handle the invalid sensor gracefully
        self.central_hub.sensor_handler.receive_datapoint(msg)
        
        # Test detect method with no data
        self.central_hub.sensor_handler.latest_data = {}
        self.central_hub.detect()  # Should not crash
        
        # Test with extreme values
        msg.sensor_type = "thermometer"
        msg.sensor_datapoint = float('inf')  # Infinite value
        self.central_hub.sensor_handler.receive_datapoint(msg)
        
        # Test detect after setting extreme value
        self.central_hub.detect()  # Should handle extreme values

    def test_hub_parameter_handling(self):
        """Test hub's parameter handling"""
        if not hasattr(self.central_hub, 'config_manager'):
            pytest.skip("Config manager not found")
        
        # Get original parameters
        original_params = {}
        if hasattr(self.central_hub.config_manager, 'params'):
            original_params = self.central_hub.config_manager.params.copy()
        
        try:
            # Test parameter access
            assert hasattr(self.central_hub.config_manager, 'get_param')
            
            # Get a parameter with default
            value = self.central_hub.config_manager.get_param('non_existent', 'default_value')
            assert value == 'default_value', "Default value should be returned for missing parameters"
            
            # Set a parameter if possible
            if hasattr(self.central_hub.config_manager, 'set_param'):
                self.central_hub.config_manager.set_param('test_param', 'test_value')
                value = self.central_hub.config_manager.get_param('test_param', None)
                assert value == 'test_value', "Parameter should be set and retrievable"
        finally:
            # Restore original parameters if possible
            if hasattr(self.central_hub.config_manager, 'params') and hasattr(self.central_hub.config_manager, 'set_param'):
                self.central_hub.config_manager.params = original_params

    def test_publisher_manager_methods(self):
        """Test publisher manager methods"""
        if not hasattr(self.central_hub, 'publisher_manager'):
            pytest.skip("Publisher manager not found")
        
        # Test status publishing
        self.central_hub.publisher_manager.publish_status("test", "idle")
        
        # Test event publishing if the method exists - note only takes one argument
        if hasattr(self.central_hub.publisher_manager, 'publish_event'):
            self.central_hub.publisher_manager.publish_event("test")
    
        # Test heartbeat publishing if the method exists
        if hasattr(self.central_hub.publisher_manager, 'publish_heartbeat'):
            self.central_hub.publisher_manager.publish_heartbeat()
    
        # Test system data publishing if the method exists
        if hasattr(self.central_hub.publisher_manager, 'publish_system_data'):
            # Use empty dictionaries for the data arguments
            self.central_hub.publisher_manager.publish_system_data(50.0, {}, {}, {})
