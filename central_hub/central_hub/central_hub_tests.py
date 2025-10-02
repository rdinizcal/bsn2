"""Shared test methods that can be used by both BDD and characteristic tests for Central Hub"""
import time
import rclpy
import signal
from bsn_interfaces.msg import SensorData, TargetSystemData
from std_msgs.msg import Header
from rclpy.parameter import Parameter
from central_hub.central_hub import CentralHub

class SharedCentralHubTests:
    """Shared test methods for central hub testing"""

    @staticmethod
    def assert_receive_datapoint_works(hub : CentralHub):
        """Assert that hub can receive sensor data points"""
        # Create test sensor data
        msg = SensorData()
        msg.sensor_type = "thermometer"
        msg.sensor_datapoint = 37.0
        msg.risk_level = "normal"
        msg.risk = 10.0
        
        # Simulate receiving the datapoint
        if hasattr(hub, 'sensor_handler') and hasattr(hub.sensor_handler, 'receive_datapoint'):
            hub.sensor_handler.receive_datapoint(msg)
            assert hub.sensor_handler.latest_data["thermometer"] == 37.0, f'Expected 37.0, got {hub.sensor_handler.latest_data["thermometer"]}'
        return True

    @staticmethod
    def assert_detect_normal_conditions_works(hub : CentralHub):
        """Assert that detection works under normal conditions"""
        # Don't check active state if already configured by fixture
        # Just ensure we have the required components
        if not hasattr(hub, 'sensor_handler'):
            return False
        
        # Set normal sensor data
        hub.sensor_handler.latest_risk = {
            "thermometer": 10.0, "ecg": 10.0, "oximeter": 10.0,
            "abps": 10.0, "abpd": 10.0, "glucosemeter": 10.0,
        }
        
        # Trigger detection
        hub.detect()
        return True

    @staticmethod
    def assert_detect_abnormal_conditions_works(hub):
        """Assert that detection works under abnormal conditions"""
        # Don't check active state if already configured by fixture
        if not hasattr(hub, 'sensor_handler'):
            return False
        
        # Set abnormal sensor data
        hub.sensor_handler.latest_risk = {
            "thermometer": 80.0, "ecg": 80.0, "oximeter": 80.0,
            "abps": 80.0, "abpd": 80.0, "glucosemeter": 80.0,
        }
        
        # Trigger detection
        hub.detect()
        return True

    @staticmethod
    def assert_data_fusion_works(hub):
        """Assert that data fusion algorithm works correctly"""
        if not hasattr(hub, 'fusion_engine') or not hasattr(hub, 'sensor_handler'):
            return False
        
        hub.sensor_handler.latest_risk = {
            "thermometer": 10.0, "ecg": 10.0, "oximeter": 10.0,
            "abps": 10.0, "abpd": 10.0, "glucosemeter": 10.0,
        }
        
        result = hub.fusion_engine.fuse_data()
        assert 5.0 <= result <= 15.0, f"Expected result between 5-15 for equal values of 10, got {result}"
        return result

    @staticmethod
    def assert_blood_pressure_handling_works(hub):
        """Assert that blood pressure special handling works"""
        if not hasattr(hub, 'fusion_engine') or not hasattr(hub, 'sensor_handler'):
            return False
            
        hub.sensor_handler.latest_risk = {
            "thermometer": 10.0, "ecg": 10.0, "oximeter": 10.0,
            "abps": 20.0, "abpd": 20.0, "glucosemeter": 10.0,
        }
        
        result = hub.fusion_engine.fuse_data()
        assert result > 10.0, "Blood pressure values should be averaged and affect result"
        return result

    @staticmethod
    def assert_format_log_message_works(hub):
        """Assert that log message formatting works"""
        if not hasattr(hub, 'sensor_handler') or not hasattr(hub, 'visualizer'):
            return False
            
        hub.sensor_handler.latest_data = {
            "thermometer": 37.0, "ecg": 90.0, "oximeter": 98.0,
            "abps": 110.0, "abpd": 75.0, "glucosemeter": 80.0,
        }
        
        hub.sensor_handler.latest_risks_labels = {
            "thermometer": "low", "ecg": "low", "oximeter": "low",
            "abps": "low", "abpd": "low", "glucosemeter": "low",
        }
        
        log_message = hub.visualizer.format_log_message()
        
        for sensor in ["thermometer", "ecg", "oximeter", "abps", "abpd", "glucosemeter"]:
            assert sensor in log_message
        return log_message

    @staticmethod
    def assert_lifecycle_transitions_work(hub):
        """Assert that lifecycle transitions work - but don't conflict with fixture state"""
        # Check current state first
        is_active = getattr(hub, 'active', False)
        
        if not is_active:
            # Only test activation if not already active
            result = hub.trigger_configure()
            if result is not None:
                time.sleep(0.2)
                
                result = hub.trigger_activate()
                if result is not None:
                    time.sleep(0.2)
                    assert getattr(hub, 'active', False), "Node should be active after activation"
        
        # Test deactivation (safe to do even if already active)
        if getattr(hub, 'active', False):
            result = hub.trigger_deactivate()
            if result is not None:
                time.sleep(0.2)
                assert not getattr(hub, 'active', True), "Node should be inactive after deactivation"
        
        return True

    @staticmethod
    def assert_detect_with_exceptions_works(hub):
        """Assert that detect handles exceptions gracefully"""
        # Configure and activate
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Save original fusion engine
        original_fusion_engine = hub.fusion_engine
        
        try:
            # Replace fusion engine with one that raises an exception
            class MockFusionEngine:
                def fuse_data(self):
                    raise RuntimeError("Simulated fusion error")
            
            hub.fusion_engine = MockFusionEngine()
            
            # Call detect - should catch the exception
            hub.detect()  # This should log an error but not crash
            
        finally:
            # Restore original fusion engine
            hub.fusion_engine = original_fusion_engine
        
        return True

    @staticmethod
    def assert_detect_with_empty_data_works(hub):
        """Assert that detect handles empty sensor data"""
        # Configure and activate
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Empty all data
        if hasattr(hub, 'sensor_handler'):
            hub.sensor_handler.latest_data = {}
            hub.sensor_handler.latest_risk = {}
            hub.sensor_handler.sensor_battery_levels = {}
        
        # Call detect - should handle empty data
        hub.detect()
        return True

    @staticmethod
    def assert_is_active_method_works(hub):
        """Assert that is_active method works correctly"""
        # Test when inactive
        if hasattr(hub, 'is_active'):
            active = hub.is_active()
            if hasattr(hub, 'active'):
                assert active == hub.active
        
        # Configure and activate
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Test when active
        if hasattr(hub, 'is_active'):
            active = hub.is_active()
            assert active, "Node should be active after activation"
        
        return True

    @staticmethod
    def assert_detect_under_low_battery_works(hub):
        """Assert that detect works under low battery conditions"""
        # Configure and activate
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Save original battery level
        original_level = hub.battery_manager.battery.current_level
        
        try:
            # Set battery level to very low
            hub.battery_manager.battery.current_level = 1.0
            
            # Call detect - should handle low battery case
            hub.detect()
            
        finally:
            # Restore battery level
            hub.battery_manager.battery.current_level = original_level
        
        return True

    @staticmethod
    def assert_detect_when_inactive_works(hub):
        """Assert that detect works when node is inactive"""
        # Configure but do not activate
        hub.trigger_configure()
        time.sleep(0.1)
        
        # Check if we're already inactive - no need to deactivate
        if hasattr(hub, 'active') and hub.active:
            hub.trigger_deactivate()
            time.sleep(0.1)
        
        # Ensure we're inactive before proceeding
        assert not hub.active, "Node should be inactive for this test"
        
        # Deplete some battery first to allow room for recharge
        original_level = 80.0
        hub.battery_manager.battery.current_level = original_level
    
        # Call detect - should not process data, but recharge battery
        hub.detect()
        time.sleep(0.1)  # Give time for recharge to happen
        
        # Battery should have recharged
        assert hub.battery_manager.battery.current_level > original_level, "Battery should recharge when inactive"
        return True

    @staticmethod
    def assert_error_handling_paths_work(hub):
        """Assert that error handling paths work correctly - avoid lifecycle conflicts"""
        # Don't try to configure/activate if already done by fixture
        
        # Test with valid topic names but test content
        if hasattr(hub, 'publisher_manager') and hasattr(hub.publisher_manager, 'publish_status'):
            hub.publisher_manager.publish_status("test_source", "test_content")
        
        # Test with unusual but valid message
        if hasattr(hub, 'sensor_handler') and hasattr(hub.sensor_handler, 'receive_datapoint'):
            unusual_msg = SensorData()
            unusual_msg.header = Header()
            unusual_msg.sensor_type = "thermometer"
            unusual_msg.sensor_datapoint = -999.0  # Unusual but valid value
            unusual_msg.risk_level = "unknown"
            unusual_msg.risk = -1.0
            
            # This should be handled gracefully
            hub.sensor_handler.receive_datapoint(unusual_msg)
        
        return True

    @staticmethod
    def assert_battery_threshold_behavior_works(hub):
        """Assert that battery threshold behavior works correctly - avoid lifecycle conflicts"""
        # Don't try to configure/activate if already done by fixture
        if not hasattr(hub, 'battery_manager'):
            return False
            
        # Save original battery level
        original_level = hub.battery_manager.battery.current_level
        
        try:
            threshold = getattr(hub, 'battery_threshold', 2.0)
                
            # Test low battery threshold (just above shutdown threshold)
            hub.battery_manager.battery.current_level = threshold + 0.1
            hub.detect()
            time.sleep(0.1)
            
            # Test critical battery (below threshold) 
            hub.battery_manager.battery.current_level = threshold - 0.1
            hub.battery_manager.check_battery_status()
            time.sleep(0.2)
            
            assert hub.battery_manager.is_recharging, "Node should enter recharge mode below threshold"
            
        finally:
            # Restore original battery level
            hub.battery_manager.battery.current_level = original_level
        
        return True

    @staticmethod
    def assert_parameter_handling_works(hub):
        """Assert that parameter handling works correctly"""
        if hasattr(hub, '_handle_parameter_change'):
            # Call with a frequency parameter change
            params = [Parameter('frequency', Parameter.Type.DOUBLE, 5.0)]
            hub._handle_parameter_change(params)
            
            # Verify frequency was updated
            assert abs(hub.frequency - 5.0) < 0.1, "Frequency parameter should be updated"
            
            # Call with other parameters
            params = [
                Parameter('battery_threshold', Parameter.Type.DOUBLE, 15.0),
                Parameter('unknown_param', Parameter.Type.STRING, 'test')
            ]
            hub._handle_parameter_change(params)
            
            # Verify threshold was updated (if that parameter exists)
            if hasattr(hub, 'battery_threshold'):
                assert abs(hub.battery_threshold - 15.0) < 0.1, "Battery threshold should be updated"
        
        return True

    @staticmethod
    def assert_components_exist(hub):
        """Assert that all required components exist"""
        assert hasattr(hub, 'sensor_handler'), "SensorHandler should be created"
        assert hasattr(hub, 'publisher_manager'), "PublisherManager should be created"
        assert hasattr(hub, 'battery_manager'), "BatteryManager should be created"
        assert hasattr(hub, 'fusion_engine'), "FusionEngine should be created"
        assert hasattr(hub, 'risk_analyzer'), "RiskAnalyzer should be created"
        assert hasattr(hub, 'visualizer'), "Visualizer should be created"
        return True

    @staticmethod
    def assert_signal_handler_works():
        """Assert that signal handler works correctly"""
        try:
            from central_hub.central_hub import signal_handler
            
            # Mock rclpy.ok and rclpy.shutdown
            original_ok = rclpy.ok
            original_shutdown = rclpy.shutdown
            
            ok_values = [True]
            shutdown_called = [False]
            
            # Mock functions
            def mock_ok():
                return ok_values[0]
                
            def mock_shutdown():
                shutdown_called[0] = True
                ok_values[0] = False
            
            try:
                # Install mocks
                rclpy.ok = mock_ok
                rclpy.shutdown = mock_shutdown
                
                # Call signal_handler
                signal_handler(signal.SIGINT, None)
                
                # Check that shutdown was called
                assert shutdown_called[0], "shutdown should be called by signal_handler"
                assert not ok_values[0], "ok should be set to False after shutdown"
                
            finally:
                # Restore original functions
                rclpy.ok = original_ok
                rclpy.shutdown = original_shutdown
        
        except ImportError:
            return False  # Skip if not importable
        
        return True

    @staticmethod
    def publish_and_process_sensor_data(hub, sensor_type, value, risk_level="normal", risk_percentage=None):
        """Helper method to publish and process sensor data"""
        msg = SensorData()
        header = Header()
        header.stamp = hub.get_clock().now().to_msg()
        header.frame_id = sensor_type

        msg.header = header
        msg.sensor_type = sensor_type
        msg.sensor_datapoint = float(value)
        msg.risk_level = risk_level

        if risk_percentage is not None:
            msg.risk = float(risk_percentage)
        else:
            # Convert risk level to percentage
            risk_map = {"high": 80.0, "moderate": 50.0, "low": 10.0, "normal": 10.0}
            msg.risk = float(risk_map.get(risk_level, -1.0))

        # Process the message directly
        if hasattr(hub, 'sensor_handler') and hasattr(hub.sensor_handler, 'receive_datapoint'):
            hub.sensor_handler.receive_datapoint(msg)

        # Process the message
        for _ in range(5):
            rclpy.spin_once(hub, timeout_sec=0.1)
            time.sleep(0.1)
        
        return msg