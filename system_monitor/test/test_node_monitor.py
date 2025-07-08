#!/usr/bin/env python3
import pytest
import rclpy
from rclpy.node import Node
from bsn_interfaces.msg import Status, Event, EnergyStatus
from std_msgs.msg import String
import threading
import time

# Import the SystemMonitor class
from system_monitor.node_monitor import SystemMonitor

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
def monitor_node(setup_ros):
    """Create a SystemMonitor node for testing"""
    monitor = SystemMonitor()
    monitor.get_logger().info("SystemMonitor started for testing")
    
    yield monitor
    
    monitor.destroy_node()

class TestSystemMonitor:
    """Tests for the SystemMonitor class"""
    
    def test_initialization(self, monitor_node):
        """Test initialization of SystemMonitor"""
        # Verify the monitor has been properly initialized
        assert hasattr(monitor_node, 'log_status_pub')
        assert hasattr(monitor_node, 'log_event_pub')
        assert hasattr(monitor_node, 'log_energy_pub')
        assert hasattr(monitor_node, 'status_sub')
        assert hasattr(monitor_node, 'event_sub')
        assert hasattr(monitor_node, 'energy_sub')
        
        # Check for status publisher
        assert hasattr(monitor_node, 'status_pub')
        
        # Check for monitored_nodes dictionary
        assert hasattr(monitor_node, 'monitored_nodes')
        assert isinstance(monitor_node.monitored_nodes, dict)
        
        # Check for parameter loading
        assert hasattr(monitor_node, 'node_list')
        assert isinstance(monitor_node.node_list, list)
        assert hasattr(monitor_node, 'heartbeat_timeout')
        assert hasattr(monitor_node, 'check_interval')
        assert hasattr(monitor_node, 'debug')
    
    def test_status_forwarding(self, setup_ros):
        """Test forwarding of status messages (collector functionality)"""
        # Create test node and monitor
        test_node = rclpy.create_node("test_status_sender")
        monitor = SystemMonitor()
        
        # Create mock publisher and subscriber
        status_pub = test_node.create_publisher(Status, 'component_status', 10)
        
        # Track forwarded messages
        received_messages = []
        
        def status_callback(msg):
            test_node.get_logger().info(f"Received forwarded status: {msg.content}")
            received_messages.append(msg)
        
        status_sub = test_node.create_subscription(
            Status, 'log_status', status_callback, 10)
        
        # Create executor and spin in separate thread
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(test_node)
        executor.add_node(monitor)
        
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        
        try:
            # Wait for discovery
            time.sleep(2.0)
            
            # Create and publish test message
            msg = Status()
            msg.source = "test_sensor"
            msg.target = "test_target"
            msg.content = "activated"
            msg.task = "measuring"
            
            # Publish message multiple times to increase chance of delivery
            for _ in range(5):
                status_pub.publish(msg)
                time.sleep(0.2)
            
            # Wait for message forwarding with timeout
            result = wait_for(lambda: len(received_messages) > 0, 3.0)
            
            # Verify message was forwarded
            assert len(received_messages) > 0, "No status messages forwarded"
            assert received_messages[0].source == "test_sensor"
            assert received_messages[0].content == "activated"
            assert received_messages[0].task == "measuring"
            
        finally:
            # Clean up
            executor.shutdown()
            if spin_thread.is_alive():
                spin_thread.join(timeout=1.0)
            test_node.destroy_node()
            monitor.destroy_node()
    
    def test_event_forwarding(self, setup_ros):
        """Test forwarding of event messages (collector functionality)"""
        # Create test node and monitor
        test_node = rclpy.create_node("test_event_sender")
        monitor = SystemMonitor()
        
        # Create mock publisher and subscriber
        event_pub = test_node.create_publisher(Event, 'collect_event', 10)
        
        # Track forwarded messages
        received_messages = []
        
        def event_callback(msg):
            test_node.get_logger().info(f"Received forwarded event: {msg.content}")
            received_messages.append(msg)
        
        event_sub = test_node.create_subscription(
            Event, 'log_event', event_callback, 10)
        
        # Create executor and spin in separate thread
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(test_node)
        executor.add_node(monitor)
        
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        
        try:
            # Wait for discovery
            time.sleep(2.0)
            
            # Create and publish test message
            msg = Event()
            msg.source = "test_sensor"
            msg.target = "system"
            msg.content = "activate"
            
            # Publish message multiple times to increase chance of delivery
            for _ in range(5):
                event_pub.publish(msg)
                time.sleep(0.2)
            
            # Wait for message forwarding with timeout
            result = wait_for(lambda: len(received_messages) > 0, 3.0)
            
            # Verify message was forwarded
            assert len(received_messages) > 0, "No event messages forwarded"
            assert received_messages[0].source == "test_sensor"
            assert received_messages[0].content == "activate"
            
        finally:
            # Clean up
            executor.shutdown()
            if spin_thread.is_alive():
                spin_thread.join(timeout=1.0)
            test_node.destroy_node()
            monitor.destroy_node()
    
    def test_energy_forwarding(self, setup_ros):
        """Test forwarding of energy status messages (collector functionality)"""
        # Create test node and monitor
        test_node = rclpy.create_node("test_energy_sender")
        monitor = SystemMonitor()
        
        # Create mock publisher and subscriber
        energy_pub = test_node.create_publisher(
            EnergyStatus, 'collect_energy_status', 10)
        
        # Track forwarded messages
        received_messages = []
        
        def energy_callback(msg):
            test_node.get_logger().info(f"Received forwarded energy status: {msg.content}")
            received_messages.append(msg)
        
        energy_sub = test_node.create_subscription(
            EnergyStatus, 'log_energy_status', energy_callback, 10)
        
        # Create executor and spin in separate thread
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(test_node)
        executor.add_node(monitor)
        
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        
        try:
            # Wait for discovery
            time.sleep(2.0)
            
            # Create and publish test message
            msg = EnergyStatus()
            msg.source = "test_sensor"
            msg.target = "system"
            msg.content = "energy:75.0:cost:0.5"
            
            # Publish message multiple times to increase chance of delivery
            for _ in range(5):
                energy_pub.publish(msg)
                time.sleep(0.2)
            
            # Wait for message forwarding with timeout
            result = wait_for(lambda: len(received_messages) > 0, 3.0)
            
            # Verify message was forwarded
            assert len(received_messages) > 0, "No energy status messages forwarded"
            assert received_messages[0].source == "test_sensor"
            assert received_messages[0].content == "energy:75.0:cost:0.5"
            
        finally:
            # Clean up
            executor.shutdown()
            if spin_thread.is_alive():
                spin_thread.join(timeout=1.0)
            test_node.destroy_node()
            monitor.destroy_node()
    
    def test_node_state_tracking(self, setup_ros):
        """Test tracking of node states (monitor functionality)"""
        # Create test node and monitor
        test_node = rclpy.create_node("test_node_tracker")
        
        # Create a custom monitor with a test node in the monitored list
        monitor = SystemMonitor()
        # Add the test node to monitored nodes
        test_node_name = "test_sensor"
        monitor.monitored_nodes[test_node_name] = {
            'state': 'unknown',
            'last_heartbeat': time.time(),
            'active': False,
            'task': 'idle',
            'content': 'unknown',
            'is_recharging': False
        }
        
        # Create mock publishers
        event_pub = test_node.create_publisher(Event, 'collect_event', 10)
        status_pub = test_node.create_publisher(Status, 'component_status', 10)
        
        # Create executor and spin in separate thread
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(test_node)
        executor.add_node(monitor)
        
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        
        try:
            # Wait for discovery
            time.sleep(2.0)
            
            # 1. Send heartbeat event (activate)
            event_msg = Event()
            event_msg.source = test_node_name
            event_msg.target = "system"
            event_msg.content = "activate"
            
            event_pub.publish(event_msg)
            time.sleep(0.5)
            
            # Check that the node is marked active
            assert monitor.monitored_nodes[test_node_name]['active'] == True, "Node not marked as active"
            
            # 2. Send status update
            status_msg = Status()
            status_msg.source = test_node_name
            status_msg.target = "system"
            status_msg.content = "activated"
            status_msg.task = "measuring"
            
            status_pub.publish(status_msg)
            time.sleep(0.5)
            
            # Check that status is updated
            assert monitor.monitored_nodes[test_node_name]['content'] == "activated"
            assert monitor.monitored_nodes[test_node_name]['task'] == "measuring"
            
            # 3. Send recharge event
            event_msg.content = "recharging"
            event_pub.publish(event_msg)
            time.sleep(0.5)
            
            # Check that recharge state is detected
            assert monitor.monitored_nodes[test_node_name]['is_recharging'] == True
            
            # 4. Send deactivate event
            event_msg.content = "deactivate"
            event_pub.publish(event_msg)
            time.sleep(0.5)
            
            # Check that node is marked inactive
            assert monitor.monitored_nodes[test_node_name]['active'] == False
            
        finally:
            # Clean up
            executor.shutdown()
            if spin_thread.is_alive():
                spin_thread.join(timeout=1.0)
            test_node.destroy_node()
            monitor.destroy_node()
    
    def test_status_publishing(self, setup_ros):
        """Test publishing of monitor status"""
        # Create test node and monitor
        test_node = rclpy.create_node("test_status_receiver")
        monitor = SystemMonitor()
        
        # Clear existing monitored nodes and add only our test node
        monitor.monitored_nodes = {}  # Add this line to clear existing nodes
        
        test_node_name = "test_sensor"
        monitor.monitored_nodes[test_node_name] = {
            'state': 'unknown',
            'last_heartbeat': time.time(),
            'active': True,
            'task': 'measuring',
            'content': 'activated',
            'is_recharging': False
        }
        
        # Track status messages
        received_messages = []
        
        def status_callback(msg):
            test_node.get_logger().info(f"Received status: {msg.data}")
            received_messages.append(msg)
        
        status_sub = test_node.create_subscription(
            String, 'system_monitor/status', status_callback, 10)
        
        # Create executor and spin in separate thread
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(test_node)
        executor.add_node(monitor)
        
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        
        try:
            # Wait for discovery
            time.sleep(2.0)
            
            # Manually trigger status publishing
            monitor.publish_monitor_status()
            
            # Wait for message with timeout
            result = wait_for(lambda: len(received_messages) > 0, 3.0)
            
            # Verify status message content
            assert len(received_messages) > 0, "No status messages received"
            assert test_node_name in received_messages[0].data
            assert "ACTIVE=True" in received_messages[0].data
            assert "TASK=measuring" in received_messages[0].data
            
            
            
        finally:
            # Clean up
            executor.shutdown()
            if spin_thread.is_alive():
                spin_thread.join(timeout=1.0)
            test_node.destroy_node()
            monitor.destroy_node()
    
    def test_heartbeat_checking(self, setup_ros, monkeypatch):
        """Test checking for missing heartbeats"""
        # Create monitor with test nodes
        monitor = SystemMonitor()
        
        # Override node list and debug
        monitor.node_list = ["test_active_node", "test_inactive_node"]
        monitor.debug = True
        
        # Mock the logger
        log_warnings = []
        log_debugs = []
        
        class MockLogger:
            def warn(self, msg):
                log_warnings.append(msg)
                
            def debug(self, msg):
                log_debugs.append(msg)
                
            def info(self, msg):
                pass
                
            def error(self, msg):
                pass
        
        mock_logger = MockLogger()
        monkeypatch.setattr(monitor, 'get_logger', lambda: mock_logger)
        
        # Set up test nodes
        monitor.monitored_nodes = {
            "test_active_node": {
                'state': 'unknown',
                'last_heartbeat': time.time() - 10.0,  # Old heartbeat
                'active': True,  # Active node
                'task': 'measuring',
                'content': 'activated',
                'is_recharging': False
            },
            "test_inactive_node": {
                'state': 'unknown',
                'last_heartbeat': time.time() - 10.0,  # Old heartbeat
                'active': False,  # Inactive node
                'task': 'idle',
                'content': 'deactivated',
                'is_recharging': False
            }
        }
        
        # Set heartbeat timeout
        monitor.heartbeat_timeout = 5.0
        
        try:
            # Check heartbeats
            monitor.check_heartbeats()
            
            # Verify that warning is logged for active node only
            assert len(log_warnings) == 1
            assert "test_active_node" in log_warnings[0]
            assert "hasn't sent a heartbeat" in log_warnings[0]
            
            # Verify debug message for inactive node
            assert len(log_debugs) >= 1
            assert any("test_inactive_node" in msg for msg in log_debugs)
            
        finally:
            # Clean up
            monitor.destroy_node()
    
    def test_main_function(self, setup_ros, monkeypatch):
        """Test the main function of system_monitor.py"""
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
                assert node.get_name() == 'system_monitor'
                
            def spin(self):
                executor_spin_called[0] = True
                
            def shutdown(self):
                pass

        # Fix: Update the import path to use node_monitor instead of system_monitor
        from system_monitor.node_monitor import SystemMonitor, main
        original_monitor = SystemMonitor
        original_executor = rclpy.executors.MultiThreadedExecutor
        
        # Mock the SystemMonitor class
        class MockSystemMonitor:
            def __init__(self):
                node_created[0] = True
                self.destroyed = False
                
            def get_name(self):
                return 'system_monitor'
                
            def destroy_node(self):
                self.destroyed = True
    
        try:
            # Apply mocks - FIX: Use direct list assignment instead of setattr
            def mock_init(*args, **kwargs):
                init_called[0] = True
            
            def mock_shutdown():
                shutdown_called[0] = True
            
            monkeypatch.setattr(rclpy, 'init', mock_init)
            monkeypatch.setattr(rclpy, 'shutdown', mock_shutdown)
            monkeypatch.setattr(rclpy, 'ok', lambda: False)  # Make it exit immediately
            monkeypatch.setattr(rclpy.executors, 'MultiThreadedExecutor', MockExecutor)
            monkeypatch.setattr('system_monitor.node_monitor.SystemMonitor', MockSystemMonitor)
            
            # Import main and call it
            main()
            
            # Verify all parts of main were called
            assert init_called[0], "rclpy.init() should be called"
            assert node_created[0], "SystemMonitor should be instantiated"
            assert executor_add_called[0], "executor.add_node() should be called"
            assert executor_spin_called[0], "executor.spin() should be called"
            assert shutdown_called[0], "rclpy.shutdown() should be called"
            
        finally:
            # Restore original functions
            monkeypatch.setattr(rclpy, 'init', original_init)
            monkeypatch.setattr(rclpy, 'shutdown', original_shutdown)
            monkeypatch.setattr(rclpy, 'ok', original_ok)
            monkeypatch.setattr(rclpy.executors, 'MultiThreadedExecutor', original_executor)
            monkeypatch.setattr('system_monitor.node_monitor.SystemMonitor', original_monitor)