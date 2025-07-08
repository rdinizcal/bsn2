import pytest
import rclpy
from rclpy.node import Node
from bsn_interfaces.msg import AdaptationCommand
from bsn_interfaces.srv import EffectorRegister
import threading
import time

# Import the ParamAdapter class
from system_monitor.param_adapter import ParamAdapter

# Add wait_for mechanism to ensure message delivery
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

@pytest.fixture(scope="function")
def adapter_node(setup_ros):
    """Create a ParamAdapter node for testing"""
    adapter = ParamAdapter()
    adapter.get_logger().info("ParamAdapter started for testing")
    
    yield adapter
    
    adapter.destroy_node()

class TestParamAdapter:
    """Tests for the ParamAdapter class"""
    
    def test_initialization(self, adapter_node):
        """Test initialization of ParamAdapter"""
        # Verify the adapter has been properly initialized
        assert hasattr(adapter_node, 'registered_components')
        assert isinstance(adapter_node.registered_components, dict)
        
        # Check for _component_publishers instead of publishers
        assert hasattr(adapter_node, '_component_publishers')
        assert isinstance(adapter_node._component_publishers, dict)
        
        # Check for service and subscription
        assert hasattr(adapter_node, 'register_service')
        assert hasattr(adapter_node, 'command_sub')
        
        # Check for parameters
        assert hasattr(adapter_node, 'check_interval')
        assert adapter_node.check_interval > 0
        assert hasattr(adapter_node, 'debug')
    
    def test_component_registration(self, setup_ros):
        """Test component registration via service"""
        # Create test node and adapter
        test_node = rclpy.create_node("test_component")
        adapter = ParamAdapter()
        
        # Create client for registration service
        client = test_node.create_client(
            EffectorRegister,
            'EffectorRegister'
        )
        
        # Create executor and spin in separate thread
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(test_node)
        executor.add_node(adapter)
        
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        
        try:
            # Wait for service discovery
            result = wait_for(lambda: client.service_is_ready(), 5.0)
            assert result, "Service not available"
            
            # Create registration request
            request = EffectorRegister.Request()
            request.name = "test_component"
            request.connection = True  # Register
            
            # Send request
            future = client.call_async(request)
            
            # Wait for response
            result = wait_for(lambda: future.done(), 3.0)
            assert result, "Service call timed out"
            
            # Check response
            response = future.result()
            assert response.ack, "Registration failed"
            
            # Verify component is registered
            assert "test_component" in adapter.registered_components
            
            
            # Test deregistration
            request.connection = False  # Deregister
            future = client.call_async(request)
            
            # Wait for response
            result = wait_for(lambda: future.done(), 3.0)
            assert result, "Service call timed out"
            
            # Check response
            response = future.result()
            assert response.ack, "Deregistration failed"
            
            # Verify component is deregistered
            assert "test_component" not in adapter.registered_components
            
        finally:
            # Clean up
            executor.shutdown()
            if spin_thread.is_alive():
                spin_thread.join(timeout=1.0)
            test_node.destroy_node()
            adapter.destroy_node()
    
    def test_command_routing(self, setup_ros):
        """Test routing of adaptation commands"""
        # Create test node and adapter
        test_node = rclpy.create_node("test_command_router")
        adapter = ParamAdapter()
        
        # Create publisher for adaptation commands
        command_pub = test_node.create_publisher(
            AdaptationCommand,
            'reconfigure',
            10
        )
        
        # Create subscription to receive routed commands
        received_commands = []
        
        def command_callback(msg):
            test_node.get_logger().info(f"Received routed command: {msg.action}")
            received_commands.append(msg)
        
        # Manually register a component
        test_component = "test_target"
        adapter.registered_components[test_component] = {
            'registered_at': time.time(),
            'last_command': None
        }
        
        # Create a publisher for this component - use the updated attribute name
        adapter._component_publishers[test_component] = adapter.create_publisher(
            AdaptationCommand,
            f'reconfigure_{test_component}',
            10
        )
        
        # Subscribe to component's reconfigure topic
        command_sub = test_node.create_subscription(
            AdaptationCommand,
            f'reconfigure_{test_component}',
            command_callback,
            10
        )
        
        # Create executor and spin in separate thread
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(test_node)
        executor.add_node(adapter)
        
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        
        try:
            # Wait for discovery
            time.sleep(1.0)
            
            # Create and publish command
            msg = AdaptationCommand()
            msg.source = "test_source"
            msg.target = test_component
            msg.action = "test_action"
            
            # Publish message multiple times to increase chance of delivery
            for _ in range(5):
                command_pub.publish(msg)
                time.sleep(0.2)
            
            # Wait for command routing with timeout
            result = wait_for(lambda: len(received_commands) > 0, 3.0)
            
            # Verify command was routed
            assert len(received_commands) > 0, "No commands routed"
            assert received_commands[0].source == "test_source"
            assert received_commands[0].target == test_component
            assert received_commands[0].action == "test_action"
            
        finally:
            # Clean up
            executor.shutdown()
            if spin_thread.is_alive():
                spin_thread.join(timeout=1.0)
            test_node.destroy_node()
            adapter.destroy_node()
    
    def test_check_components(self, setup_ros, monkeypatch):
        """Test checking for stale components"""
        adapter = ParamAdapter()
        
        # Add test components with different timestamps
        adapter.registered_components = {
            "fresh_component": {
                'registered_at': time.time(),
                'last_command': None
            },
            "stale_component": {
                'registered_at': time.time() - 600.0,  # 10 minutes old
                'last_command': None
            }
        }
        
        # Enable debug mode
        adapter.debug = True
        
        # Mock the logger
        logged_info = []
        
        class MockLogger:
            def info(self, msg):
                logged_info.append(msg)
                
            def warn(self, msg):
                pass
                
            def error(self, msg):
                pass
                
            def debug(self, msg):
                pass
        
        monkeypatch.setattr(adapter, 'get_logger', lambda: MockLogger())
        
        try:
            # Run component check
            adapter.check_components()
            
            # Verify stale components were logged
            assert any("stale_component" in msg for msg in logged_info)
            assert any("Active components" in msg for msg in logged_info)
            
        finally:
            adapter.destroy_node()
    
    def test_error_handling_in_module_connect(self, setup_ros, monkeypatch):
        """Test error handling in module_connect method"""
        adapter = ParamAdapter()
        
        # Mock the logger
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
    
        monkeypatch.setattr(adapter, 'get_logger', lambda: MockLogger())
        
        # Create a mock request that will definitely cause an error
        # Use a request that will cause an error when accessing name attribute
        class BrokenRequest:
            @property
            def name(self):
                raise RuntimeError("Test error")
            
            connection = True
        
        request = BrokenRequest()
        response = type('MockResponse', (), {'ack': None})
        
        try:
            # Call module_connect with invalid request
            adapter.module_connect(request, response)
            
            # Verify error was logged and response was negative
            assert len(errors_logged) > 0
            assert "Error in module_connect" in errors_logged[0]
            assert "Test error" in errors_logged[0]
            assert response.ack is False
            
        finally:
            adapter.destroy_node()
    
    def test_main_function(self, setup_ros, monkeypatch):
        """Test the main function of param_adapter.py"""
        # Mock rclpy functions
        init_called = [False]
        shutdown_called = [False]
        node_created = [False]
        executor_add_called = [False]
        executor_spin_called = [False]
        
        # Save original functions
        original_init = rclpy.init
        original_shutdown = rclpy.shutdown
        original_ok = rclpy.ok
        
        # Mock the MultiThreadedExecutor
        class MockExecutor:
            def __init__(self):
                pass
                
            def add_node(self, node):
                executor_add_called[0] = True
                assert node.get_name() == 'param_adapter'
                
            def spin(self):
                executor_spin_called[0] = True
                
            def shutdown(self):
                pass
        
        # Import main and save original class
        from system_monitor.param_adapter import ParamAdapter, main
        original_adapter = ParamAdapter
        original_executor = rclpy.executors.MultiThreadedExecutor
        
        # Mock the ParamAdapter class
        class MockParamAdapter:
            def __init__(self):
                node_created[0] = True
                self.destroyed = False
                
            def get_name(self):
                return 'param_adapter'
                
            def destroy_node(self):
                self.destroyed = True
        
        try:
            # Apply mocks
            def mock_init(*args, **kwargs):
                init_called[0] = True
            
            def mock_shutdown():
                shutdown_called[0] = True
            
            monkeypatch.setattr(rclpy, 'init', mock_init)
            monkeypatch.setattr(rclpy, 'shutdown', mock_shutdown)
            monkeypatch.setattr(rclpy, 'ok', lambda: False)  # Make it exit immediately
            monkeypatch.setattr(rclpy.executors, 'MultiThreadedExecutor', MockExecutor)
            monkeypatch.setattr('system_monitor.param_adapter.ParamAdapter', MockParamAdapter)
            
            # Call main
            main()
            
            # Verify all parts of main were called
            assert init_called[0], "rclpy.init() should be called"
            assert node_created[0], "ParamAdapter should be instantiated"
            assert executor_add_called[0], "executor.add_node() should be called"
            assert executor_spin_called[0], "executor.spin() should be called"
            assert shutdown_called[0], "rclpy.shutdown() should be called"
            
        finally:
            # Restore original functions
            monkeypatch.setattr(rclpy, 'init', original_init)
            monkeypatch.setattr(rclpy, 'shutdown', original_shutdown)
            monkeypatch.setattr(rclpy, 'ok', original_ok)
            monkeypatch.setattr(rclpy.executors, 'MultiThreadedExecutor', original_executor)
            monkeypatch.setattr('system_monitor.param_adapter.ParamAdapter', original_adapter)