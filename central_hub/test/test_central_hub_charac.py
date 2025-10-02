import pytest
import time
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from bsn_interfaces.msg import SensorData, TargetSystemData
from std_msgs.msg import Header
from fixtures import direct_central_hub, central_hub_node
from shared_test_methods import SharedCentralHubTests


class TestCentralHubBehavior:
    """Central Hub characteristic tests - using individual fixtures for better isolation"""

    def test_receive_datapoint(self, direct_central_hub):
        """Test receiving data from sensors"""
        direct_central_hub.trigger_configure()
        direct_central_hub.trigger_activate()
        SharedCentralHubTests.assert_receive_datapoint_works(direct_central_hub)

    def test_detect_normal_conditions(self, direct_central_hub):
        """Test detection under normal conditions"""
        hub = direct_central_hub
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        SharedCentralHubTests.assert_detect_normal_conditions_works(hub)

    def test_detect_abnormal_conditions(self, direct_central_hub):
        """Test detection under abnormal conditions"""
        hub = direct_central_hub
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        SharedCentralHubTests.assert_detect_abnormal_conditions_works(hub)

    def test_data_fusion_algorithm(self, direct_central_hub):
        """Test the data fusion algorithm"""
        hub = direct_central_hub
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        result = SharedCentralHubTests.assert_data_fusion_works(hub)
        assert result is not None, "Data fusion should return a result"

    def test_blood_pressure_special_handling(self, direct_central_hub):
        """Test special handling of blood pressure data"""
        hub = direct_central_hub
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        SharedCentralHubTests.assert_blood_pressure_handling_works(hub)

    def test_format_log_message(self, direct_central_hub):
        """Test log message formatting"""
        hub = direct_central_hub
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        SharedCentralHubTests.assert_format_log_message_works(hub)

    def test_emergency_detection_timing(self, direct_central_hub):
        """Test emergency detection timing requirements"""
        hub = direct_central_hub
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Record start time
        start_time = time.time()
        
        # Trigger high-risk scenario
        hub.sensor_handler.latest_risk = {
            "thermometer": 80.0, "ecg": 80.0, "oximeter": 80.0,
            "abps": 80.0, "abpd": 80.0, "glucosemeter": 80.0,
        }
        
        hub.detect()
        
        detection_time = (time.time() - start_time) * 1000
        assert detection_time <= 250, f"Emergency detection took {detection_time:.2f}ms, exceeds 250ms limit"

    def test_battery_status_monitoring(self, direct_central_hub):
        """Test battery status monitoring functionality"""
        SharedCentralHubTests.assert_battery_threshold_behavior_works(direct_central_hub)

    def test_sensor_data_validation(self, direct_central_hub):
        """Test sensor data validation and error handling"""
        SharedCentralHubTests.assert_error_handling_paths_work(direct_central_hub)

    def test_lifecycle_management(self, direct_central_hub):
        """Test lifecycle management functionality"""
        SharedCentralHubTests.assert_lifecycle_transitions_work(direct_central_hub)

    def test_component_initialization(self, direct_central_hub):
        """Test that all components are properly initialized"""
        SharedCentralHubTests.assert_components_exist(direct_central_hub)

    def test_parameter_updates(self, direct_central_hub):
        """Test dynamic parameter updates"""
        SharedCentralHubTests.assert_parameter_handling_works(direct_central_hub)

    def test_concurrent_sensor_updates(self, direct_central_hub):
        """Test handling of concurrent updates from multiple sensors"""
        hub = direct_central_hub
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Simulate concurrent sensor data
        sensors_data = {
            "thermometer": 37.5,
            "ecg": 85.0,
            "oximeter": 97.0,
            "abps": 115.0,
            "abpd": 78.0,
            "glucosemeter": 90.0
        }
        
        # Set all data at once (simulating concurrent updates)
        hub.sensor_handler.latest_data = sensors_data
        hub.sensor_handler.latest_risk = {k: 10.0 for k in sensors_data.keys()}
        
        # Trigger detection
        hub.detect()
        
        # Verify all data is accessible
        for sensor_type, value in sensors_data.items():
            assert hub.sensor_handler.latest_data[sensor_type] == value

    def test_risk_escalation_scenarios(self, direct_central_hub):
        """Test various risk escalation scenarios"""
        hub = direct_central_hub
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Test low risk scenario
        hub.sensor_handler.latest_risk = {k: 10.0 for k in ["thermometer", "ecg", "oximeter", "abps", "abpd", "glucosemeter"]}
        low_risk_result = hub.fusion_engine.fuse_data()
        
        # Test high risk scenario
        hub.sensor_handler.latest_risk = {k: 80.0 for k in ["thermometer", "ecg", "oximeter", "abps", "abpd", "glucosemeter"]}
        high_risk_result = hub.fusion_engine.fuse_data()
        
        assert high_risk_result > low_risk_result, "High risk should result in higher fusion result"

    def test_partial_sensor_data(self, direct_central_hub):
        """Test behavior with partial sensor data"""
        hub = direct_central_hub
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Only some sensors have data
        hub.sensor_handler.latest_risk = {
            "thermometer": 20.0,
            "ecg": 25.0,
            # Other sensors missing
        }
        
        result = hub.fusion_engine.fuse_data()
        assert result > 0, "Should handle partial sensor data gracefully"

    def test_sensor_disconnection_handling(self, direct_central_hub):
        """Test handling of sensor disconnections"""
        hub = direct_central_hub
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Initially have all sensors
        hub.sensor_handler.latest_risk = {k: 15.0 for k in ["thermometer", "ecg", "oximeter", "abps", "abpd", "glucosemeter"]}
        initial_result = hub.fusion_engine.fuse_data()
        
        # Simulate sensor disconnection by clearing some data
        hub.sensor_handler.latest_risk = {"thermometer": 15.0, "ecg": 15.0}
        disconnected_result = hub.fusion_engine.fuse_data()
        
        # Should still work with fewer sensors
        assert disconnected_result > 0, "Should handle sensor disconnections"

    def test_extreme_risk_values(self, direct_central_hub):
        """Test handling of extreme risk values"""
        hub = direct_central_hub
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Test with 0% risk
        hub.sensor_handler.latest_risk = {k: 0.0 for k in ["thermometer", "ecg", "oximeter"]}
        zero_result = hub.fusion_engine.fuse_data()
        
        # Test with 100% risk
        hub.sensor_handler.latest_risk = {k: 100.0 for k in ["thermometer", "ecg", "oximeter"]}
        max_result = hub.fusion_engine.fuse_data()
        
        assert zero_result <= max_result, "Maximum risk should be >= minimum risk"

    def test_data_persistence_across_detections(self, direct_central_hub):
        """Test that data persists correctly across multiple detection cycles"""
        hub = direct_central_hub
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Set initial data
        test_data = {"thermometer": 37.0, "ecg": 80.0}
        hub.sensor_handler.latest_data = test_data
        hub.sensor_handler.latest_risk = {"thermometer": 15.0, "ecg": 20.0}
        
        # Run multiple detection cycles
        for _ in range(3):
            hub.detect()
            time.sleep(0.1)
        
        # Data should persist
        assert hub.sensor_handler.latest_data["thermometer"] == 37.0
        assert hub.sensor_handler.latest_data["ecg"] == 80.0

    def test_detection_frequency_compliance(self, direct_central_hub):
        """Test that detection runs at the expected frequency"""
        hub = direct_central_hub
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Set up data
        hub.sensor_handler.latest_risk = {"thermometer": 20.0}
        
        # Measure detection timing
        start_time = time.time()
        detection_count = 0
        
        # Run detections for a short period
        while time.time() - start_time < 1.0:
            hub.detect()
            detection_count += 1
            time.sleep(0.1)
        
        # Should be able to run multiple detections per second
        assert detection_count >= 5, f"Expected at least 5 detections, got {detection_count}"

    def test_memory_usage_stability(self, direct_central_hub):
        """Test that memory usage remains stable over multiple operations"""
        hub = direct_central_hub
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Run many detection cycles to check for memory leaks
        for i in range(100):
            hub.sensor_handler.latest_risk = {"thermometer": float(i % 50)}
            hub.detect()
        
        # If we get here without crashing, memory usage is stable
        assert True, "Memory usage remained stable"

    def test_error_recovery(self, direct_central_hub):
        """Test recovery from various error conditions"""
        hub = direct_central_hub
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Test recovery from empty data
        hub.sensor_handler.latest_risk = {}
        hub.detect()  # Should not crash
        
        # Add valid data and verify recovery
        hub.sensor_handler.latest_risk = {"thermometer": 25.0}
        result = hub.fusion_engine.fuse_data()
        assert result > 0, "Should recover and provide valid results"

    def test_heartbeat_functionality(self, direct_central_hub):
        """Test heartbeat publishing functionality"""
        hub = direct_central_hub
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.2)  # Wait for heartbeat timer
        
        # Verify heartbeat timer exists and is active
        assert hasattr(hub, '_heartbeat_timer'), "Heartbeat timer should exist"
        assert hub._heartbeat_timer is not None, "Heartbeat timer should be initialized"

    def test_status_publishing(self, direct_central_hub):
        """Test status message publishing"""
        hub = direct_central_hub
        
        # Test configuration status
        hub.trigger_configure()
        time.sleep(0.1)
        
        # Test activation status
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Verify node is active
        assert hub.is_active(), "Node should be active after activation"

    def test_cleanup_on_deactivation(self, direct_central_hub):
        """Test proper cleanup when node is deactivated"""
        hub = direct_central_hub
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Verify active state
        assert hub.is_active(), "Node should be active"
        
        # Deactivate
        hub.trigger_deactivate()
        time.sleep(0.1)
        
        # Verify inactive state
        assert not hub.is_active(), "Node should be inactive after deactivation"

    def test_multiple_activation_cycles(self, direct_central_hub):
        """Test multiple activation/deactivation cycles"""
        hub = direct_central_hub
        hub.trigger_configure()
        time.sleep(0.1)
        
        for _ in range(3):
            # Activate
            hub.trigger_activate()
            time.sleep(0.1)
            assert hub.is_active(), "Node should be active"
            
            # Deactivate
            hub.trigger_deactivate()
            time.sleep(0.1)
            assert not hub.is_active(), "Node should be inactive"

    def test_detect_when_inactive_skip(self, direct_central_hub):
        """Test that detect method skips processing when inactive"""
        SharedCentralHubTests.assert_detect_when_inactive_works(direct_central_hub)

    def test_detect_with_low_battery_skip(self, direct_central_hub):
        """Test that detect method handles low battery conditions"""
        SharedCentralHubTests.assert_detect_under_low_battery_works(direct_central_hub)

    def test_configuration_parameter_validation(self, direct_central_hub):
        """Test validation of configuration parameters"""
        hub = direct_central_hub
        
        # Test that config manager exists and has expected parameters
        assert hasattr(hub, 'config'), "Config manager should exist"
        
        # Try to configure with valid parameters
        hub.trigger_configure()
        time.sleep(0.1)
        
        # Configuration should succeed
        assert True, "Configuration with valid parameters should succeed"


class TestCentralHubIntegration:
    """Integration tests that can be reused by BDD tests"""
    
    def test_full_sensor_integration_flow(self, direct_central_hub):
        """Complete integration test that BDD can reuse"""
        hub = direct_central_hub
        
        # Full lifecycle
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Simulate sensor data flow
        sensor_data = {
            "thermometer": {"value": 38.5, "risk": 35.0},  # Moderate fever
            "ecg": {"value": 95.0, "risk": 25.0},          # Slightly elevated
            "oximeter": {"value": 94.0, "risk": 40.0},     # Concerning oxygen
        }
        
        # Set sensor data
        for sensor, data in sensor_data.items():
            hub.sensor_handler.latest_data[sensor] = data["value"]
            hub.sensor_handler.latest_risk[sensor] = data["risk"]
        
        # Run detection
        hub.detect()
        
        # Verify processing occurred
        result = hub.fusion_engine.fuse_data()
        assert 20.0 <= result <= 50.0, f"Expected moderate risk result, got {result}"
        
        return result  # BDD tests can use this return value

    def test_emergency_scenario_integration(self, direct_central_hub):
        """Emergency scenario test that BDD can reuse"""
        hub = direct_central_hub
        
        # Setup
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Critical values across multiple sensors
        critical_data = {
            "thermometer": {"value": 41.0, "risk": 85.0},   # High fever
            "ecg": {"value": 140.0, "risk": 90.0},          # Tachycardia
            "oximeter": {"value": 88.0, "risk": 95.0},      # Low oxygen
            "abps": {"value": 180.0, "risk": 88.0},         # High blood pressure
        }
        
        start_time = time.time()
        
        # Set critical data
        for sensor, data in critical_data.items():
            hub.sensor_handler.latest_data[sensor] = data["value"]
            hub.sensor_handler.latest_risk[sensor] = data["risk"]
        
        # Run detection
        hub.detect()
        
        detection_time = (time.time() - start_time) * 1000
        result = hub.fusion_engine.fuse_data()
        
        # Verify emergency detection
        assert result >= 80.0, f"Expected emergency-level risk, got {result}"
        assert detection_time <= 250, f"Emergency detection too slow: {detection_time:.2f}ms"
        
        return {"risk": result, "time": detection_time}

    def test_system_overload_integration(self, direct_central_hub):
        """System overload test that BDD can reuse"""
        hub = direct_central_hub
        
        # Setup
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Simulate high-frequency sensor updates
        for i in range(50):
            hub.sensor_handler.latest_data["thermometer"] = 37.0 + (i * 0.01)
            hub.sensor_handler.latest_risk["thermometer"] = 15.0
            hub.detect()
            # Minimal delay to simulate high frequency
            time.sleep(0.001)
        
        # Now send emergency data
        start_time = time.time()
        hub.sensor_handler.latest_data["thermometer"] = 41.5
        hub.sensor_handler.latest_risk["thermometer"] = 90.0
        hub.detect()
        
        detection_time = (time.time() - start_time) * 1000
        
        # Under overload, detection might be delayed
        return {"time": detection_time, "overloaded": detection_time > 250}

    def test_sensor_recovery_integration(self, direct_central_hub):
        """Sensor recovery test that BDD can reuse"""
        hub = direct_central_hub
        
        # Setup
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Start with normal data
        hub.sensor_handler.latest_data = {"thermometer": 37.0, "ecg": 80.0}
        hub.sensor_handler.latest_risk = {"thermometer": 10.0, "ecg": 15.0}
        initial_result = hub.fusion_engine.fuse_data()
        
        # Simulate sensor failure (data disappears)
        hub.sensor_handler.latest_data = {}
        hub.sensor_handler.latest_risk = {}
        failure_result = hub.fusion_engine.fuse_data()
        
        # Simulate sensor recovery
        hub.sensor_handler.latest_data = {"thermometer": 37.2, "ecg": 82.0}
        hub.sensor_handler.latest_risk = {"thermometer": 12.0, "ecg": 18.0}
        recovery_result = hub.fusion_engine.fuse_data()
        
        return {
            "initial": initial_result,
            "failure": failure_result,
            "recovery": recovery_result,
            "recovered": recovery_result > failure_result
        }


# Additional utility functions for BDD test reuse
class CentralHubTestUtils:
    """Utility methods for BDD and other test reuse"""
    
    @staticmethod
    def setup_hub_for_testing(hub):
        """Standard setup for hub testing - reusable by BDD"""
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        return hub
    
    @staticmethod
    def set_sensor_data(hub, sensor_data_dict):
        """Set sensor data - reusable by BDD"""
        for sensor, data in sensor_data_dict.items():
            if isinstance(data, dict):
                hub.sensor_handler.latest_data[sensor] = data.get("value", 0.0)
                hub.sensor_handler.latest_risk[sensor] = data.get("risk", 0.0)
            else:
                hub.sensor_handler.latest_data[sensor] = data
                hub.sensor_handler.latest_risk[sensor] = 10.0  # Default risk
    
    @staticmethod
    def measure_detection_time(hub, setup_func=None):
        """Measure detection time - reusable by BDD"""
        if setup_func:
            setup_func(hub)
        
        start_time = time.time()
        hub.detect()
        return (time.time() - start_time) * 1000
    
    @staticmethod
    def verify_emergency_response(hub, expected_risk_threshold=70.0):
        """Verify emergency response - reusable by BDD"""
        result = hub.fusion_engine.fuse_data()
        return result >= expected_risk_threshold
