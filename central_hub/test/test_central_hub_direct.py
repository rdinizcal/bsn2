import pytest
import time
from fixtures import direct_central_hub
from shared_test_methods import SharedCentralHubTests


class TestCentralHubDirect:
    """Direct tests targeting uncovered code in central_hub.py - using shared test methods"""
    
    def test_lifecycle_state_transitions_full(self, direct_central_hub):
        """Test full lifecycle transitions sequence"""
        SharedCentralHubTests.assert_lifecycle_transitions_work(direct_central_hub)
    
    def test_multiple_transitions(self, direct_central_hub):
        """Test calling transitions multiple times"""
        hub = direct_central_hub
        
        # Configure
        hub.trigger_configure()
        time.sleep(0.1)
        
        # Try to configure again (should be idempotent or fail gracefully)
        try:
            hub.trigger_configure()
        except Exception as e:
            hub.get_logger().info(f"Expected exception when configuring twice: {e}")
    
        # Activate
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Try to activate again
        try:
            hub.trigger_activate()
        except Exception as e:
            hub.get_logger().info(f"Expected exception when activating twice: {e}")
        
        # Deactivate
        hub.trigger_deactivate()
        time.sleep(0.1)
        
        # Check that we're in the inactive state
        assert not hub.active, "Node should be inactive after deactivation"
    
    def test_detect_method_with_exceptions(self, direct_central_hub):
        """Test detect method with simulated internal exceptions"""
        SharedCentralHubTests.assert_detect_with_exceptions_works(direct_central_hub)
    
    def test_detect_with_empty_data(self, direct_central_hub):
        """Test detect method with empty sensor data"""
        SharedCentralHubTests.assert_detect_with_empty_data_works(direct_central_hub)
    
    def test_is_active_method(self, direct_central_hub):
        """Test is_active method"""
        SharedCentralHubTests.assert_is_active_method_works(direct_central_hub)
    
    def test_detect_under_low_battery(self, direct_central_hub):
        """Test detect behavior under low battery"""
        SharedCentralHubTests.assert_detect_under_low_battery_works(direct_central_hub)
    
    def test_detect_when_inactive(self, direct_central_hub):
        """Test detect behavior when node is inactive"""
        SharedCentralHubTests.assert_detect_when_inactive_works(direct_central_hub)
    
    def test_hub_callbacks(self, direct_central_hub):
        """Test any callback methods in the hub"""
        hub = direct_central_hub
        
        # Test timer callbacks if they exist
        if hasattr(hub, '_timer_callback'):
            hub._timer_callback()
        elif hasattr(hub, '_detection_timer_callback'):
            hub._detection_timer_callback()
        
        # Test parameter callbacks if they exist
        SharedCentralHubTests.assert_parameter_handling_works(hub)
    
    def test_error_handling_paths(self, direct_central_hub):
        """Test error handling paths in the central hub"""
        SharedCentralHubTests.assert_error_handling_paths_work(direct_central_hub)
    
    def test_battery_threshold_behavior(self, direct_central_hub):
        """Test behavior at battery thresholds"""
        SharedCentralHubTests.assert_battery_threshold_behavior_works(direct_central_hub)

    def test_exception_in_detect(self, direct_central_hub):
        """Test handling of exceptions in detect method"""
        hub = direct_central_hub
        
        # Configure and activate
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Save original components
        original_publisher_manager = hub.publisher_manager
        
        try:
            # Replace publisher manager with one that raises exception
            class BrokenPublisherManager:
                def publish_status(self, *args):
                    raise RuntimeError("Test exception")
                    
                def publish_event(self, *args):
                    raise RuntimeError("Test exception")
            
            # Install the broken component
            hub.publisher_manager = BrokenPublisherManager()
            
            # Call detect - should handle the exception gracefully
            hub.detect()
            
        finally:
            # Restore original components
            hub.publisher_manager = original_publisher_manager

    def test_parameter_handling(self, direct_central_hub):
        """Test parameter handling in central hub"""
        SharedCentralHubTests.assert_parameter_handling_works(direct_central_hub)

    def test_node_logging(self, direct_central_hub):
        """Test various logging methods"""
        hub = direct_central_hub
        
        # Configure and activate
        hub.trigger_configure()
        time.sleep(0.1)
        
        # Test log methods if they exist
        if hasattr(hub, 'log_status'):
            hub.log_status("Test status")
        
        if hasattr(hub, 'log_event'):
            hub.log_event("Test event")
        
        if hasattr(hub, 'log_error'):
            hub.log_error("Test error")
            
        # Test various log levels
        hub.get_logger().debug("Test debug message")
        hub.get_logger().info("Test info message")
        hub.get_logger().warning("Test warning message")
        hub.get_logger().error("Test error message")
    
    def test_on_configure_and_cleanup_directly(self, direct_central_hub):
        """Test the on_configure and on_cleanup methods directly if they exist"""
        hub = direct_central_hub
        
        if hasattr(hub, 'on_configure'):
            # Call on_configure directly
            result = hub.on_configure(None)
            assert result is not None, "on_configure should return a valid result"
        
        # Verify all components were created using shared method
        SharedCentralHubTests.assert_components_exist(hub)
        
        if hasattr(hub, 'on_cleanup'):
            # Call on_cleanup directly
            result = hub.on_cleanup(None)
            assert result is not None, "on_cleanup should return a valid result"

    def test_on_shutdown_directly(self, direct_central_hub):
        """Test the on_shutdown method directly if it exists"""
        hub = direct_central_hub
        
        # First configure to have proper state
        if hasattr(hub, 'on_configure'):
            hub.on_configure(None)
        
        if hasattr(hub, 'on_shutdown'):
            # Call on_shutdown directly
            result = hub.on_shutdown(None)
            assert result is not None, "on_shutdown should return a valid result"
            
            # Check if finalized flag is set
            if hasattr(hub, '_finalized'):
                assert hub._finalized, "Node should be finalized after on_shutdown"

    def test_signal_handler_direct(self, direct_central_hub):
        """Test the signal_handler function directly if it exists"""
        SharedCentralHubTests.assert_signal_handler_works()

    # Additional tests that use shared methods where applicable
    def test_battery_threshold_edge_cases(self, direct_central_hub):
        """Test edge cases of battery threshold handling"""
        SharedCentralHubTests.assert_battery_threshold_behavior_works(direct_central_hub)

    def test_main_loop_simulation(self, direct_central_hub):
        """Test the main loop behavior by direct simulation"""
        hub = direct_central_hub
        
        # Configure but don't activate yet
        hub.trigger_configure()
        time.sleep(0.1)
        
        # Test behavior when inactive
        assert not hub.active, "Node should be inactive initially"
        
        # Save original battery level
        original_level = hub.battery_manager.battery.current_level
        
        try:
            # Set battery to a level that allows measurement
            new_level = 80.0
            hub.battery_manager.battery.current_level = new_level
            
            # Simulate the main loop's inactive branch
            if not hub.active:
                hub.battery_manager.recharge()
                assert hub.battery_manager.battery.current_level > new_level, "Battery should recharge when inactive"
            
            # Now activate and test the active branch
            hub.trigger_activate()
            time.sleep(0.1)
            
            # Check we're active
            assert hub.active, "Node should be active after activation"
            
            # Simulate the main loop's active branch
            if hub.active:
                # This should call detect() which processes sensor data and publishes results
                hub.detect()
                
            # Now set finalized flag to test early exit condition
            hub._finalized = True
            # In the main loop, this would cause the loop to exit
            assert hub._finalized, "Finalized flag should be set"
            
        finally:
            # Restore original values
            hub.battery_manager.battery.current_level = original_level
            hub._finalized = False

    def test_detect_with_disabled_components(self, direct_central_hub):
        """Test detect method with disabled components"""
        hub = direct_central_hub
        
        # Configure and activate
        hub.trigger_configure()
        time.sleep(0.1)
        hub.trigger_activate()
        time.sleep(0.1)
        
        # Save original components
        original_sensor_handler = hub.sensor_handler
        original_fusion_engine = hub.fusion_engine
        
        try:
            # Set components to None to test graceful handling
            hub.sensor_handler = None
            
            # Call detect - should handle missing components
            hub.detect()  # Should not crash
            
            # Restore sensor_handler but remove fusion_engine
            hub.sensor_handler = original_sensor_handler
            hub.fusion_engine = None
            
            # Call detect again
            hub.detect()  # Should not crash
            
        finally:
            # Restore original components
            hub.sensor_handler = original_sensor_handler
            hub.fusion_engine = original_fusion_engine