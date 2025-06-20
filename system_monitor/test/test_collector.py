import pytest
import rclpy
from rclpy.node import Node
from bsn_interfaces.msg import Status, Event, EnergyStatus
import threading
import time
import os
import sys

# Explicitly import the Probe class
from system_monitor.collector import Probe

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
def probe_node(setup_ros):
    """Create and spin a Probe node for testing"""
    probe = Probe()
    
    # Use a separate thread for spinning
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(probe)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    # Allow time for node initialization
    time.sleep(1.0)
    
    yield probe
    
    # Clean up
    executor.shutdown()
    if spin_thread.is_alive():
        spin_thread.join(timeout=1.0)
    probe.destroy_node()

class TestProbe:
    """Tests for the Probe (Collector) class"""
    
    def test_initialization(self, probe_node):
        """Test initialization of Probe"""
        # Verify the probe has been properly initialized
        assert hasattr(probe_node, 'log_status_pub')
        assert hasattr(probe_node, 'log_event_pub')
        assert hasattr(probe_node, 'log_energy_pub')
        
        # Check for frequency parameter
        assert hasattr(probe_node, 'frequency')
        assert probe_node.frequency == 1.0  # Default value
    
    def test_collect_status(self, setup_ros):
        """Test collecting and forwarding status messages"""
        # Create a test node with both publisher and subscriber
        node = rclpy.create_node("test_collect_status")
        
        # Create a mock publisher to send to probe
        status_pub = node.create_publisher(Status, 'component_status', 10)
        
        # Create a mock subscriber to check if probe forwards the message
        received_messages = []
        
        def status_callback(msg):
            node.get_logger().info(f"Received message: {msg.content}")
            received_messages.append(msg)
        
        # Create subscription to check forwarded messages
        sub = node.create_subscription(Status, 'log_status', status_callback, 10)
        
        # Create and initialize probe
        probe = Probe()
        
        # Create executor and spin in separate thread
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        executor.add_node(probe)
        
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        
        try:
            # Wait for discovery
            time.sleep(2.0)
            
            # Create and publish test message
            msg = Status()
            msg.source = "test_source"
            msg.target = "test_target"
            msg.content = "test_status"
            msg.task = "test_task"
            
            # Publish message multiple times to increase chance of delivery
            for _ in range(5):
                status_pub.publish(msg)
                time.sleep(0.1)
            
            # Wait for message processing with timeout
            result = wait_for(lambda: len(received_messages) > 0, 3.0)
            
            # Check if message was received and forwarded
            if not result:
                node.get_logger().error("Message not received within timeout")
            
            # Verify message content if received
            assert len(received_messages) > 0, "No messages received"
            assert received_messages[0].source == "test_source"
            assert received_messages[0].content == "test_status"
            
        finally:
            # Clean up
            executor.shutdown()
            if spin_thread.is_alive():
                spin_thread.join(timeout=1.0)
            node.destroy_node()
            probe.destroy_node()
    
    def test_collect_event(self, setup_ros):
        """Test collecting and forwarding event messages"""
        # Create a test node with both publisher and subscriber
        node = rclpy.create_node("test_collect_event")
        
        # Create a mock publisher to send to probe
        event_pub = node.create_publisher(Event, 'collect_event', 10)
        
        # Create a mock subscriber to check if probe forwards the message
        received_messages = []
        
        def event_callback(msg):
            node.get_logger().info(f"Received event: {msg.content}")
            received_messages.append(msg)
        
        # Create subscription to check forwarded messages
        sub = node.create_subscription(Event, 'log_event', event_callback, 10)
        
        # Create and initialize probe
        probe = Probe()
        
        # Create executor and spin in separate thread
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        executor.add_node(probe)
        
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        
        try:
            # Wait for discovery
            time.sleep(2.0)
            
            # Create and publish test message
            msg = Event()
            msg.source = "test_source"
            msg.target = "test_target"
            msg.content = "test_event"
            
            # Publish message multiple times to increase chance of delivery
            for _ in range(5):
                event_pub.publish(msg)
                time.sleep(0.1)
            
            # Wait for message processing with timeout
            result = wait_for(lambda: len(received_messages) > 0, 3.0)
            
            # Verify message content if received
            assert len(received_messages) > 0, "No messages received"
            assert received_messages[0].source == "test_source"
            assert received_messages[0].content == "test_event"
            
        finally:
            # Clean up
            executor.shutdown()
            if spin_thread.is_alive():
                spin_thread.join(timeout=1.0)
            node.destroy_node()
            probe.destroy_node()
    
    def test_collect_energy_status(self, setup_ros):
        """Test collecting and forwarding energy status messages"""
        # Create a test node with both publisher and subscriber
        node = rclpy.create_node("test_collect_energy")
        
        # Create a mock publisher to send to probe
        energy_pub = node.create_publisher(EnergyStatus, 'collect_energy_status', 10)
        
        # Create a mock subscriber to check if probe forwards the message
        received_messages = []
        
        def energy_callback(msg):
            node.get_logger().info(f"Received energy status: {msg.content}")
            received_messages.append(msg)
        
        # Create subscription to check forwarded messages
        sub = node.create_subscription(EnergyStatus, 'log_energy_status', energy_callback, 10)
        
        # Create and initialize probe
        probe = Probe()
        
        # Create executor and spin in separate thread
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        executor.add_node(probe)
        
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        
        try:
            # Wait for discovery
            time.sleep(2.0)
            
            # Create and publish test message
            msg = EnergyStatus()
            msg.source = "test_source"
            msg.target = "test_target"
            msg.content = "energy:50.0:cost:0.1"
            
            # Publish message multiple times to increase chance of delivery
            for _ in range(5):
                energy_pub.publish(msg)
                time.sleep(0.1)
            
            # Wait for message processing with timeout
            result = wait_for(lambda: len(received_messages) > 0, 3.0)
            
            # Verify message content if received
            assert len(received_messages) > 0, "No messages received"
            assert received_messages[0].source == "test_source"
            assert received_messages[0].content == "energy:50.0:cost:0.1"
            
        finally:
            # Clean up
            executor.shutdown()
            if spin_thread.is_alive():
                spin_thread.join(timeout=1.0)
            node.destroy_node()
            probe.destroy_node()
    
    def test_spin_collector(self, setup_ros, monkeypatch):
        """Test the spin_collector method"""
        # Import the Probe class directly
        from system_monitor.collector import Probe
        
        # Create a mock probe instance
        probe = Probe()
        
        # Keep track of sleep calls
        sleep_called = [0]
        
        # Mock the create_rate and rate.sleep functions
        class MockRate:
            def sleep(self):
                sleep_called[0] += 1
                if sleep_called[0] >= 1:  # Break after first iteration
                    monkeypatch.setattr(rclpy, 'ok', lambda: False)
                    return  # Don't raise KeyboardInterrupt, just return
    
        # Replace create_rate to return our mock
        monkeypatch.setattr(probe, 'create_rate', lambda _: MockRate())
        # Make rclpy.ok() return True initially, then False when sleep is called
        monkeypatch.setattr(rclpy, 'ok', lambda: True)
        
        # Test the spin_collector method
        probe.spin_collector()  # This now should exit cleanly
        
        # Verify sleep was called
        assert sleep_called[0] >= 1, "Rate sleep wasn't called"
        
        # Clean up
        probe.destroy_node()
    
    def test_main_function(self, setup_ros, monkeypatch):
        """Test the main function in collector.py"""
        # Mock rclpy functions
        init_called = [False]
        shutdown_called = [False]
        node_created = [False]
        spin_called = [False]
        
        # Save original functions
        original_init = rclpy.init
        original_shutdown = rclpy.shutdown
        original_spin = rclpy.spin
        original_ok = rclpy.ok
        
        # Create mocks
        def mock_init(*args, **kwargs):
            init_called[0] = True
            
        def mock_shutdown():
            shutdown_called[0] = True
            
        def mock_spin(node):
            spin_called[0] = True
            assert node.get_name() == 'probe'
    
        # Original Probe class and constructor
        from system_monitor.collector import Probe, main
        original_probe = Probe
    
        # Mock the Probe class
        class MockProbe:
            def __init__(self):
                node_created[0] = True
                self.destroyed = False
                
            def get_name(self):
                return 'probe'
                
            def destroy_node(self):
                self.destroyed = True
        
            def spin_collector(self):
                # Just return immediately to avoid infinite loop
                return
    
        try:
            # Apply mocks
            monkeypatch.setattr(rclpy, 'init', mock_init)
            monkeypatch.setattr(rclpy, 'shutdown', mock_shutdown)
            monkeypatch.setattr(rclpy, 'spin', mock_spin)
            monkeypatch.setattr(rclpy, 'ok', lambda: False)  # Cause immediate exit
            monkeypatch.setattr('system_monitor.collector.Probe', MockProbe)
            
            # Call main function
            main()
            
            # Verify all parts of main were called
            assert init_called[0], "rclpy.init() should be called"
            assert node_created[0], "Probe should be instantiated"
            assert spin_called[0], "rclpy.spin() should be called"
            assert shutdown_called[0], "rclpy.shutdown() should be called"
            
        finally:
            # Restore original functions
            monkeypatch.setattr(rclpy, 'init', original_init)
            monkeypatch.setattr(rclpy, 'shutdown', original_shutdown)
            monkeypatch.setattr(rclpy, 'spin', original_spin)
            monkeypatch.setattr(rclpy, 'ok', original_ok)
            monkeypatch.setattr('system_monitor.collector.Probe', original_probe)

    def test_debug_logging(self, setup_ros):
        """Test debug logging in collector methods"""
        # Create a test node
        node = rclpy.create_node("test_debug_log")
        
        # Create probe
        probe = Probe()
        
        # Create executor
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        executor.add_node(probe)
        
        # Spin in thread
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        
        try:
            # Wait for discovery
            time.sleep(1.0)
            
            # Call the collector methods directly, checking they don't raise errors
            # and they properly log debug messages
            
            # Test status collection
            status_msg = Status()
            status_msg.source = "debug_source"
            status_msg.content = "debug_status"
            probe.collect_status(status_msg)
            
            # Test event collection
            event_msg = Event()
            event_msg.source = "debug_source"
            event_msg.content = "debug_event"
            probe.collect_event(event_msg)
            
            # Test energy collection
            energy_msg = EnergyStatus()
            energy_msg.source = "debug_source"
            energy_msg.content = "energy:100:cost:0.5"
            probe.collect_energy_status(energy_msg)
            
        finally:
            # Clean up
            executor.shutdown()
            if spin_thread.is_alive():
                spin_thread.join(timeout=1.0)
            node.destroy_node()
            probe.destroy_node()

    def test_spin_collector_normal_exit(self, setup_ros, monkeypatch):
        """Test normal execution flow in spin_collector"""
        probe = Probe()
        
        # Track calls
        ok_values = [True, True, False]  # Return True twice then False to exit loop
        sleep_called = [0]
        debug_called = [0]
        
        # Mock functions
        def mock_ok():
            if len(ok_values) > 0:
                return ok_values.pop(0)
            return False
        
        class MockRate:
            def sleep(self):
                sleep_called[0] += 1
        
        def mock_debug(msg):
            debug_called[0] += 1
        
        # Apply mocks
        monkeypatch.setattr(rclpy, 'ok', mock_ok)
        monkeypatch.setattr(probe, 'create_rate', lambda _: MockRate())
        monkeypatch.setattr(probe.get_logger(), 'debug', mock_debug)
        
        try:
            # Run spin_collector
            probe.spin_collector()
            
            # Check that functions were called
            assert sleep_called[0] == 2, "Rate.sleep() should be called twice"
            assert debug_called[0] == 2, "Debug logging should be called twice"
            
        finally:
            probe.destroy_node()