#!/usr/bin/env python3
import pytest
import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
import threading
import time

# Import the LifecycleManager class
from sensor.components.lifecycle_manager import LifecycleManager

# Add wait_for mechanism to ensure operation completion
def wait_for(condition_function, timeout=5.0):
    """Wait for a condition function to become true or timeout."""
    start_time = time.time()
    while time.time() < start_time + timeout:
        if condition_function():
            return True
        time.sleep(0.1)
    return False

@pytest.fixture(scope="function")
def setup_ros():
    """Set up ROS context for tests"""
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()

# Mock Lifecycle Node
class MockLifecycleNode:
    def __init__(self):
        super().__init__('test_lifecycle_node')
        self.active = False
        self.configured = False
        self.cleaned_up = False
        self.shutdown = False
        
    def trigger_configure(self):
        self.configured = True
        return TransitionCallbackReturn.SUCCESS
        
    def trigger_activate(self):
        if not self.configured:
            self.node.get_logger().error("Node activation failed")
            return TransitionCallbackReturn.ERROR
            
        # Add this line to actually set the active flag
        self.active = True
        
        return TransitionCallbackReturn.SUCCESS
        
    def trigger_deactivate(self):
        self.active = False
        return TransitionCallbackReturn.SUCCESS
        
    def trigger_cleanup(self):
        self.configured = False
        self.cleaned_up = True
        return TransitionCallbackReturn.SUCCESS
        
    def trigger_shutdown(self):
        self.shutdown = True
        return TransitionCallbackReturn.SUCCESS

# Mock Battery for testing
class MockBattery:
    def __init__(self, level=100.0):
        self.current_level = level

@pytest.fixture(scope="function")
def lifecycle_node(setup_ros):
    """Create a mock lifecycle node for testing"""
    node = MockLifecycleNode()
    yield node
    node.destroy_node()

@pytest.fixture(scope="function")
def lifecycle_manager(lifecycle_node):
    """Create a LifecycleManager with a mock node"""
    manager = LifecycleManager(lifecycle_node)
    yield manager

class TestLifecycleManager:
    """Tests for the LifecycleManager class"""
    
    def test_initialization(self, lifecycle_manager, lifecycle_node):
        """Test initialization of LifecycleManager"""
        # Verify manager attributes
        assert hasattr(lifecycle_manager, 'node')
        assert lifecycle_manager.node == lifecycle_node
        
        assert hasattr(lifecycle_manager, 'auto_manage')
        assert lifecycle_manager.auto_manage is True
        
        assert hasattr(lifecycle_manager, 'monitoring_active')
        assert lifecycle_manager.monitoring_active is False
        
        # Verify thresholds
        assert hasattr(lifecycle_manager, 'battery_low_threshold')
        assert lifecycle_manager.battery_low_threshold == 5.0
        
        assert hasattr(lifecycle_manager, 'battery_recovery_threshold')
        assert lifecycle_manager.battery_recovery_threshold == 15.0
        
        # Verify auto-management flags
        assert lifecycle_manager.auto_configure_on_start is True
        assert lifecycle_manager.auto_activate_after_configure is True
        assert lifecycle_manager.battery_aware_deactivation is True
        assert lifecycle_manager.auto_recovery is True
    
    def test_configure_node(self, lifecycle_manager, lifecycle_node):
        """Test node configuration"""
        # Configure node
        result = lifecycle_manager.configure_node()
        
        # Verify results
        assert result is True
        assert lifecycle_node.configured is True
        assert hasattr(lifecycle_node, '_was_configured')
        assert getattr(lifecycle_node, '_was_configured') is True
    
    def test_activate_node(self, lifecycle_manager, lifecycle_node):
        """Test node activation"""
        # Configure first
        lifecycle_manager.configure_node()
        
        # Activate
        result = lifecycle_manager.activate_node()
        
        # Verify results
        assert result is True
        assert lifecycle_node.active is True
    
    def test_deactivate_node(self, lifecycle_manager, lifecycle_node):
        """Test node deactivation"""
        # Configure and activate first
        lifecycle_manager.configure_node()
        lifecycle_manager.activate_node()
        
        # Deactivate
        result = lifecycle_manager.deactivate_node()
        
        # Verify results
        assert result is True
        assert lifecycle_node.active is False
    
    def test_cleanup_node(self, lifecycle_manager, lifecycle_node):
        """Test node cleanup"""
        # Configure first to set _was_configured
        lifecycle_manager.configure_node()
        
        # Cleanup
        result = lifecycle_manager.cleanup_node()
        
        # Verify results
        assert result is True
        assert lifecycle_node.cleaned_up is True
        assert lifecycle_node.configured is False
        assert not hasattr(lifecycle_node, '_was_configured')
    
    def test_shutdown_node(self, lifecycle_manager, lifecycle_node):
        """Test node shutdown"""
        # Shutdown
        result = lifecycle_manager.shutdown_node()
        
        # Verify results
        assert result is True
        assert lifecycle_node.shutdown is True
        assert lifecycle_manager.auto_manage is False
        assert lifecycle_manager.monitoring_active is False
    
    def test_restart_node(self, lifecycle_manager, lifecycle_node, monkeypatch):
        """Test node restart sequence"""
        # Mock time.sleep to speed up test
        monkeypatch.setattr(time, 'sleep', lambda x: None)
        
        # Configure and activate first
        lifecycle_manager.configure_node()
        lifecycle_manager.activate_node()
        
        # Restart
        result = lifecycle_manager.restart_node()
        
        # Verify results
        assert result is True
        assert lifecycle_node.active is True  # Should end in active state
        
        # Should have gone through all transitions
        assert lifecycle_node.cleaned_up is True
    
    def test_get_battery_level(self, lifecycle_manager, lifecycle_node):
        """Test battery level retrieval"""
        # Create mock battery manager
        class MockBatteryManager:
            def __init__(self):
                self.battery = MockBattery(level=75.0)
        
        # Add battery manager to node
        lifecycle_node.battery_manager = MockBatteryManager()
        
        # Get battery level
        level = lifecycle_manager.get_battery_level()
        
        # Verify level
        assert level == 75.0
        
        # Test fallback when no battery
        delattr(lifecycle_node, 'battery_manager')
        level = lifecycle_manager.get_battery_level()
        assert level == 100.0
    
    def test_battery_threshold_activation(self, lifecycle_manager, lifecycle_node):
        """Test battery-aware activation"""
        # Create mock battery manager with low battery
        class MockBatteryManager:
            def __init__(self):
                self.battery = MockBattery(level=10.0)
        
        # Add battery manager to node
        lifecycle_node.battery_manager = MockBatteryManager()
        
        # Configure
        lifecycle_manager.configure_node()
        
        # Try to activate with low battery
        result = lifecycle_manager.activate_node()
        
        # Verify it failed due to battery
        assert result is False
        assert lifecycle_node.active is False
        
        # Now increase battery level
        lifecycle_node.battery_manager.battery.current_level = 20.0
        
        # Try again
        result = lifecycle_manager.activate_node()
        
        # Verify it works now
        assert result is True
        assert lifecycle_node.active is True
    
    def test_auto_management_start_stop(self, lifecycle_manager, lifecycle_node):
        """Test starting and stopping auto-management"""
        # Start auto-management
        lifecycle_manager.start_auto_management()
        
        # Verify timer creation
        assert hasattr(lifecycle_manager, 'monitor_timer')
        assert lifecycle_manager.monitoring_active is True
        
        # Stop auto-management
        lifecycle_manager.stop_auto_management()
        
        # Verify timer cancellation
        assert lifecycle_manager.monitoring_active is False
        assert lifecycle_manager.auto_manage is False
    
    def test_set_auto_management_flags(self, lifecycle_manager):
        """Test setting auto-management flags"""
        # Set flags
        lifecycle_manager.set_auto_management_flags(
            auto_configure=False,
            auto_activate=False,
            battery_aware=False,
            auto_recovery=False
        )
        
        # Verify changes
        assert lifecycle_manager.auto_configure_on_start is False
        assert lifecycle_manager.auto_activate_after_configure is False
        assert lifecycle_manager.battery_aware_deactivation is False
        assert lifecycle_manager.auto_recovery is False
        
        # Set only some flags
        lifecycle_manager.set_auto_management_flags(
            auto_configure=True,
            auto_recovery=True
        )
        
        # Verify selective changes
        assert lifecycle_manager.auto_configure_on_start is True
        assert lifecycle_manager.auto_activate_after_configure is False
        assert lifecycle_manager.battery_aware_deactivation is False
        assert lifecycle_manager.auto_recovery is True
    
    def test_configure_thresholds(self, lifecycle_manager):
        """Test configuring battery thresholds"""
        # Set thresholds
        lifecycle_manager.configure_thresholds(
            battery_low=10.0,
            battery_recovery=20.0
        )
        
        # Verify changes
        assert lifecycle_manager.battery_low_threshold == 10.0
        assert lifecycle_manager.battery_recovery_threshold == 20.0
        
        # Set only one threshold
        lifecycle_manager.configure_thresholds(
            battery_low=15.0
        )
        
        # Verify selective change
        assert lifecycle_manager.battery_low_threshold == 15.0
        assert lifecycle_manager.battery_recovery_threshold == 20.0
    
    def test_get_lifecycle_status(self, lifecycle_manager, lifecycle_node):
        """Test getting lifecycle status"""
        # Create mock battery manager
        class MockBatteryManager:
            def __init__(self):
                self.battery = MockBattery(level=80.0)
        
        # Add battery manager to node
        lifecycle_node.battery_manager = MockBatteryManager()
        
        # Configure and activate
        lifecycle_manager.configure_node()
        lifecycle_manager.activate_node()
        
        # Get status
        status = lifecycle_manager.get_lifecycle_status()
        
        # Verify status contents
        assert 'active' in status
        assert status['active'] is True
        
        assert 'battery_level' in status
        assert status['battery_level'] == 80.0
        
        assert 'auto_manage' in status
        assert 'monitoring_active' in status
        assert 'battery_low_threshold' in status
        assert 'battery_recovery_threshold' in status
    
    def test_error_handling(self, lifecycle_manager, lifecycle_node, monkeypatch):
        """Test error handling in lifecycle methods"""
        # Create a mock logger
        errors_logged = []
        
        class MockLogger:
            def error(self, msg):
                errors_logged.append(msg)
                
            def info(self, msg):
                pass
                
            def warn(self, msg):
                pass
                
            def debug(self, msg):
                pass
        
        monkeypatch.setattr(lifecycle_node, 'get_logger', lambda: MockLogger())
        
        # Create a function that raises an exception
        def trigger_error(*args, **kwargs):
            raise RuntimeError("Test error")
        
        # Replace trigger_configure with error function
        monkeypatch.setattr(lifecycle_node, 'trigger_configure', trigger_error)
        
        # Try to configure
        result = lifecycle_manager.configure_node()
        
        # Verify error handling
        assert result is False
        assert len(errors_logged) > 0
        assert "Error during configuration" in errors_logged[0]
        assert "Test error" in errors_logged[0]
    
    def test_monitor_lifecycle_state(self, lifecycle_manager, lifecycle_node, monkeypatch):
        """Test lifecycle state monitoring"""
        # Mock time functions
        monkeypatch.setattr(time, 'time', lambda: 1000.0)
        monkeypatch.setattr(time, 'sleep', lambda x: None)
        
        # Create mock battery manager with good battery
        class MockBatteryManager:
            def __init__(self):
                self.battery = MockBattery(level=90.0)
        
        # Add battery manager to node
        lifecycle_node.battery_manager = MockBatteryManager()
        
        # Make node inactive but configured
        lifecycle_node.active = False
        setattr(lifecycle_node, '_was_configured', True)
        
        # Enable auto-recovery
        lifecycle_manager.auto_recovery = True
        
        # Run monitor
        lifecycle_manager.monitor_lifecycle_state()
        
        # Verify auto-recovery
        assert lifecycle_node.active is True
        
        # Now test with unconfigured node
        lifecycle_node.active = False
        delattr(lifecycle_node, '_was_configured')
        
        # Mock logger to catch warnings
        warnings = []
        
        class MockLogger:
            def warn(self, msg):
                warnings.append(msg)
                
            def info(self, msg):
                pass
                
            def error(self, msg):
                pass
                
            def debug(self, msg):
                pass
        
        monkeypatch.setattr(lifecycle_node, 'get_logger', lambda: MockLogger())
        
        # Run monitor
        lifecycle_manager.monitor_lifecycle_state()
        
        # Verify it tried to configure
        assert any("Node needs configuration" in msg for msg in warnings)
        
        # Verify last_state_check was updated
        assert lifecycle_manager.last_state_check == 1000.0
    
    def test_initial_setup(self, lifecycle_manager, lifecycle_node, monkeypatch):
        """Test initial setup sequence"""
        # Create a flag to track activation timer creation
        timer_created = [False]
        timer_callback = [None]
        
        # Mock create_timer
        def mock_create_timer(delay, callback):
            timer_created[0] = True
            timer_callback[0] = callback
            
            class MockTimer:
                def __init__(self):
                    self.cancelled = False
                    
                def cancel(self):
                    self.cancelled = True
                    
                def reset(self):
                    pass
            
            return MockTimer()
        
        monkeypatch.setattr(lifecycle_node, 'create_timer', mock_create_timer)
        
        # Run initial setup
        lifecycle_manager.initial_setup()
        
        # Verify configuration happened
        assert lifecycle_node.configured is True
        
        # Verify activation timer was created
        assert timer_created[0] is True
        assert timer_callback[0] is not None
        
        # Call the activation callback
        timer_callback[0]()
        
        # Verify activation happened
        assert lifecycle_node.active is True