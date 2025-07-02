import pytest
import rclpy
from rclpy.node import Node
from bsn_interfaces.msg import Status, Event, EnergyStatus, AdaptationCommand, Uncertainty, Persist
import threading
import time

# Explicitly import the Logger class
from system_monitor.logger import Logger

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
def logger_node(setup_ros):
    """Create a Logger node for testing"""
    logger = Logger()
    logger.get_logger().info("Logger started")
    
    yield logger
    
    logger.destroy_node()

class TestLogger:
    """Tests for the Logger class"""
    
    def test_initialization(self, logger_node):
        """Test initialization of Logger"""
        # Verify the logger has been properly initialized
        assert hasattr(logger_node, 'persist_pub')
        assert hasattr(logger_node, 'status_sub')
        assert hasattr(logger_node, 'event_sub')
        assert hasattr(logger_node, 'energy_sub')
        assert hasattr(logger_node, 'adapt_sub')
        assert hasattr(logger_node, 'uncertainty_sub')
        
        # Check for frequency parameter
        assert hasattr(logger_node, 'frequency')
        assert logger_node.frequency == 2.0  # Default value
        
        # Check time reference initialization
        assert hasattr(logger_node, 'time_ref')
    
    def test_now_method(self, logger_node):
        """Test the now() method"""
        # Check that now() returns a timestamp in milliseconds
        now = logger_node.now()
        assert isinstance(now, int)
        
        # Check that calling it again returns a greater or equal value
        time.sleep(0.001)  # Sleep a tiny bit to ensure time passes
        next_now = logger_node.now()
        assert next_now >= now, f"next_now: {next_now} should be >= now: {now}"
    
    def test_receive_status(self, setup_ros):
        """Test receiving and processing status messages"""
        from system_monitor.logger import Logger
        
        # Create test node and logger
        test_node = rclpy.create_node("test_status_node")
        logger = Logger()
        
        # Create mock publisher
        status_pub = test_node.create_publisher(Status, 'log_status', 10)
        
        # Create mock subscriber for persist messages
        received_messages = []
        
        def persist_callback(msg):
            test_node.get_logger().info(f"Received persist message: {msg.content}")
            received_messages.append(msg)
        
        persist_sub = test_node.create_subscription(
            Persist, 'persist', persist_callback, 10)
        
        # Create executor and spin in separate thread
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(test_node)
        executor.add_node(logger)
        
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
                time.sleep(0.2)
            
            # Wait for message processing with timeout
            result = wait_for(lambda: len(received_messages) > 0, 3.0)
            
            # Verify message content if received
            assert len(received_messages) > 0, "No persist messages received"
            assert received_messages[0].source == "test_source"
            assert received_messages[0].target == "test_target"
            assert received_messages[0].type == "Status"
            assert received_messages[0].content == "test_status"
            
        finally:
            # Clean up
            executor.shutdown()
            if spin_thread.is_alive():
                spin_thread.join(timeout=1.0)
            test_node.destroy_node()
            logger.destroy_node()
    
    # Implement the remaining message tests with the same pattern
    def test_receive_event(self, setup_ros):
        """Test receiving and processing event messages"""
        from system_monitor.logger import Logger
        
        # Create test node and logger
        test_node = rclpy.create_node("test_event_node")
        logger = Logger()
        
        # Create mock publisher
        event_pub = test_node.create_publisher(Event, 'log_event', 10)
        
        # Create mock subscriber for persist messages
        received_messages = []
        
        def persist_callback(msg):
            test_node.get_logger().info(f"Received persist message: {msg.content}")
            received_messages.append(msg)
        
        persist_sub = test_node.create_subscription(
            Persist, 'persist', persist_callback, 10)
        
        # Create executor and spin in separate thread
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(test_node)
        executor.add_node(logger)
        
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
                time.sleep(0.2)
            
            # Wait for message processing with timeout
            result = wait_for(lambda: len(received_messages) > 0, 3.0)
            
            # Verify message content if received
            assert len(received_messages) > 0, "No persist messages received"
            assert received_messages[0].source == "test_source"
            assert received_messages[0].target == "test_target"
            assert received_messages[0].type == "Event"
            assert received_messages[0].content == "test_event"
            
        finally:
            # Clean up
            executor.shutdown()
            if spin_thread.is_alive():
                spin_thread.join(timeout=1.0)
            test_node.destroy_node()
            logger.destroy_node()
    
    def test_receive_energy(self, setup_ros):
        """Test receiving and processing energy status messages"""
        from system_monitor.logger import Logger
        
        # Create test node and logger
        test_node = rclpy.create_node("test_energy_node")
        logger = Logger()
        
        # Create mock publisher
        energy_pub = test_node.create_publisher(EnergyStatus, 'log_energy_status', 10)
        
        # Create mock subscriber for persist messages
        received_messages = []
        
        def persist_callback(msg):
            test_node.get_logger().info(f"Received persist message: {msg.content}")
            received_messages.append(msg)
        
        persist_sub = test_node.create_subscription(
            Persist, 'persist', persist_callback, 10)
        
        # Create executor and spin in separate thread
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(test_node)
        executor.add_node(logger)
        
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
                time.sleep(0.2)
            
            # Wait for message processing with timeout
            result = wait_for(lambda: len(received_messages) > 0, 3.0)
            
            # Verify message content if received
            assert len(received_messages) > 0, "No persist messages received"
            assert received_messages[0].source == "test_source"
            assert received_messages[0].target == "test_target"
            assert received_messages[0].type == "EnergyStatus"
            assert received_messages[0].content == "energy:50.0:cost:0.1"
            
        finally:
            # Clean up
            executor.shutdown()
            if spin_thread.is_alive():
                spin_thread.join(timeout=1.0)
            test_node.destroy_node()
            logger.destroy_node()
    
    def test_receive_adapt(self, setup_ros):
        """Test receiving and processing adaptation command messages"""
        from system_monitor.logger import Logger
        
        # Create test node and logger
        test_node = rclpy.create_node("test_adapt_node")
        logger = Logger()
        
        # Create mock publisher
        adapt_pub = test_node.create_publisher(AdaptationCommand, 'log_adapt', 10)
        
        # Create mock subscriber for persist messages
        received_messages = []
        
        def persist_callback(msg):
            test_node.get_logger().info(f"Received persist message: {msg.content}")
            received_messages.append(msg)
        
        persist_sub = test_node.create_subscription(
            Persist, 'persist', persist_callback, 10)
        
        # Create executor and spin in separate thread
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(test_node)
        executor.add_node(logger)
        
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        
        try:
            # Wait for discovery
            time.sleep(2.0)
            
            # Create and publish test message
            msg = AdaptationCommand()
            msg.source = "test_source"
            msg.target = "test_target"
            msg.action = "test_action"
            
            # Publish message multiple times to increase chance of delivery
            for _ in range(5):
                adapt_pub.publish(msg)
                time.sleep(0.2)
            
            # Wait for message processing with timeout
            result = wait_for(lambda: len(received_messages) > 0, 3.0)
            
            # Verify message content if received
            assert len(received_messages) > 0, "No persist messages received"
            assert received_messages[0].source == "test_source"
            assert received_messages[0].target == "test_target"
            assert received_messages[0].type == "Adaptation"
            assert received_messages[0].content == "test_action"
            
        finally:
            # Clean up
            executor.shutdown()
            if spin_thread.is_alive():
                spin_thread.join(timeout=1.0)
            test_node.destroy_node()
            logger.destroy_node()
    
    def test_receive_uncertainty(self, setup_ros):
        """Test receiving and processing uncertainty messages"""
        from system_monitor.logger import Logger
        
        # Create test node and logger
        test_node = rclpy.create_node("test_uncertainty_node")
        logger = Logger()
        
        # Create mock publisher
        uncertainty_pub = test_node.create_publisher(Uncertainty, 'log_uncertainty', 10)
        
        # Create mock subscriber for persist messages
        received_messages = []
        
        def persist_callback(msg):
            test_node.get_logger().info(f"Received persist message: {msg.content}")
            received_messages.append(msg)
        
        persist_sub = test_node.create_subscription(
            Persist, 'persist', persist_callback, 10)
        
        # Create executor and spin in separate thread
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(test_node)
        executor.add_node(logger)
        
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        
        try:
            # Wait for discovery
            time.sleep(2.0)
            
            # Create and publish test message
            msg = Uncertainty()
            msg.source = "test_source"
            msg.target = "test_target"
            msg.content = "test_uncertainty"
            
            # Publish message multiple times to increase chance of delivery
            for _ in range(5):
                uncertainty_pub.publish(msg)
                time.sleep(0.2)
            
            # Wait for message processing with timeout
            result = wait_for(lambda: len(received_messages) > 0, 3.0)
            
            # Verify message content if received
            assert len(received_messages) > 0, "No persist messages received"
            assert received_messages[0].source == "test_source"
            assert received_messages[0].target == "test_target"
            assert received_messages[0].type == "Uncertainty"
            assert received_messages[0].content == "test_uncertainty"
            
        finally:
            # Clean up
            executor.shutdown()
            if spin_thread.is_alive():
                spin_thread.join(timeout=1.0)
            test_node.destroy_node()
            logger.destroy_node()
    
    def test_main_function(self, setup_ros, monkeypatch):
        """Test the main function of logger.py"""
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
            assert node.get_name() == 'logger'
        
        # Original Logger class
        from system_monitor.logger import Logger, main
        original_logger = Logger
        
        # Mock the Logger class
        class MockLogger:
            def __init__(self):
                node_created[0] = True
                self.destroyed = False
                self.frequency = 2.0
                
            def get_name(self):
                return 'logger'
                
            def destroy_node(self):
                self.destroyed = True
                
            def create_rate(self, freq):
                assert freq == 2.0
                
                class MockRate:
                    def sleep(self):
                        # Do nothing
                        pass
                    
                return MockRate()
    
        try:
            # Apply mocks
            monkeypatch.setattr(rclpy, 'init', mock_init)
            monkeypatch.setattr(rclpy, 'shutdown', mock_shutdown)
            monkeypatch.setattr(rclpy, 'spin', mock_spin)
            monkeypatch.setattr(rclpy, 'ok', lambda: False)  # Make it exit immediately
            monkeypatch.setattr('system_monitor.logger.Logger', MockLogger)
            
            # Import main and call it
            main()
            
            # Verify all parts of main were called
            assert init_called[0], "rclpy.init() should be called"
            assert node_created[0], "Logger should be instantiated"
            assert spin_called[0], "rclpy.spin() should be called"
            assert shutdown_called[0], "rclpy.shutdown() should be called"
            
        finally:
            # Restore original functions
            monkeypatch.setattr(rclpy, 'init', original_init)
            monkeypatch.setattr(rclpy, 'shutdown', original_shutdown)
            monkeypatch.setattr(rclpy, 'spin', original_spin)
            monkeypatch.setattr(rclpy, 'ok', original_ok)
            monkeypatch.setattr('system_monitor.logger.Logger', original_logger)

    def test_receive_with_different_timestamps(self, setup_ros):
        """Test receiving messages with different timestamp handling"""
        # Create test node and logger
        test_node = rclpy.create_node("test_timestamp_node")
        logger = Logger()
        
        # Let's directly manipulate time_ref to test timestamp calculation
        original_time_ref = logger.time_ref
        logger.time_ref = 0  # Set to zero to make timestamp equal to now()
        
        try:
            # Call methods directly for coverage
            
            # Test status messages
            status_msg = Status()
            status_msg.source = "test_source"
            status_msg.target = "test_target"
            status_msg.content = "test_status"
            status_msg.task = "test_task"
            
            # Direct call to test coverage
            logger.receive_status(status_msg)
            
            # Test event messages
            event_msg = Event()
            event_msg.source = "test_source"
            event_msg.target = "test_target"
            event_msg.content = "test_event"
            
            # Direct call to test coverage
            logger.receive_event(event_msg)
            
            # Set time_ref to a different value to test different timestamp logic
            logger.time_ref = logger.get_clock().now().nanoseconds
            
            # Test energy status messages
            energy_msg = EnergyStatus()
            energy_msg.source = "test_source"
            energy_msg.target = "test_target"
            energy_msg.content = "energy:50.0:cost:0.1"
            
            # Direct call to test coverage
            logger.receive_energy(energy_msg)
            
        finally:
            # Restore original time_ref
            logger.time_ref = original_time_ref
            
            # Clean up
            test_node.destroy_node()
            logger.destroy_node()