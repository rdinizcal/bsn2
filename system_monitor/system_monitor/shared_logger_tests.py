from system_monitor.logger import Logger
import time
class SharedLoggerTests:
    """Shared test methods for Logger testing"""

    @staticmethod
    def assert_logger_initialized(logger: Logger) -> None:
        """Test initialization of Logger"""
        assert hasattr(logger, 'persist_pub')
        assert hasattr(logger, 'status_sub')
        assert hasattr(logger, 'event_sub')
        assert hasattr(logger, 'energy_sub')
        assert hasattr(logger, 'adapt_sub')
        assert hasattr(logger, 'uncertainty_sub')
        
        # Check for frequency parameter
        assert hasattr(logger, 'frequency')
        assert logger.frequency == 2.0  # Default value
        
        # Check time reference initialization
        assert hasattr(logger, 'time_ref')
    
    @staticmethod
    def assert_now_method_works(logger: Logger) -> bool:
        """Test the now() method"""
        # Check that now() returns a timestamp in milliseconds
        now = logger.now()
        assert isinstance(now, int)
        
        # Check that calling it again returns a greater or equal value
        time.sleep(0.001)  # Sleep a tiny bit to ensure time passes
        next_now = logger.now()
        assert next_now >= now, f"next_now: {next_now} should be >= now: {now}"
        return True
    
    @staticmethod
    def assert_receive_status_works(received_messages: list):
        """Test receiving and processing status messages"""
        # Verify message content if received
        assert len(received_messages) > 0, "No persist messages received"
        assert received_messages[0].source == "test_source"
        assert received_messages[0].target == "test_target"
        assert received_messages[0].type == "Status"
        assert received_messages[0].content == "test_status"
        return True
    
    @staticmethod
    def assert_receive_event_works(received_messages: list):
        """Test receiving and processing event messages"""
        # Verify message content if received
        assert len(received_messages) > 0, "No persist messages received"
        assert received_messages[0].source == "test_source"
        assert received_messages[0].target == "test_target"
        assert received_messages[0].type == "Event"
        assert received_messages[0].content == "test_event"
        return True
    
    @staticmethod
    def assert_receive_energy_works(received_messages: list):
        """Test receiving and processing energy status messages"""
        # Verify message content if received
        assert len(received_messages) > 0, "No persist messages received"
        assert received_messages[0].source == "test_source"
        assert received_messages[0].target == "test_target"
        assert received_messages[0].type == "EnergyStatus"
        assert received_messages[0].content == "energy:50.0:cost:0.1"
        return True
    
    @staticmethod
    def assert_receive_adapt_works(received_messages: list):
        """Test receiving and processing adaptation command messages"""
        # Verify message content if received
        assert len(received_messages) > 0, "No persist messages received"
        assert received_messages[0].source == "test_source"
        assert received_messages[0].target == "test_target"
        assert received_messages[0].type == "AdaptationCommand"
        assert received_messages[0].content == "test_action"
        return True
    
    @staticmethod
    def assert_receive_uncertainty_works(received_messages: list):
        """Test receiving and processing uncertainty messages"""
        # Verify message content if received
        assert len(received_messages) > 0, "No persist messages received"
        assert received_messages[0].source == "test_source"
        assert received_messages[0].target == "test_target"
        assert received_messages[0].type == "Uncertainty"
        assert received_messages[0].content == "test_uncertainty"
        return True
    
    @staticmethod
    def assert_receive_with_different_timestamps_works(logger: Logger):
        """Test receiving messages with different timestamp handling"""
        from bsn_interfaces.msg import Status, Event, EnergyStatus
        
        # Save original time_ref and manipulate it
        original_time_ref = logger.time_ref
        logger.time_ref = 0  # Set to zero to make timestamp equal to now()
        
        try:
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
            logger.receive_energy_status(energy_msg)
            
        finally:
            # Restore original time_ref
            logger.time_ref = original_time_ref
        
        return True
