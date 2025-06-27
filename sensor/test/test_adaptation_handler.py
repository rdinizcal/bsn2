#!/usr/bin/env python3

import pytest
import rclpy
from rclpy.node import Node
from bsn_interfaces.msg import AdaptationCommand
from bsn_interfaces.srv import EffectorRegister, EffectorRegister_Request, EffectorRegister_Response
from sensor.components.adaptation_handler import AdaptationHandler
import time

class MockFuture:
    """Mock future for testing service responses"""
    
    def __init__(self, response=None, exception=None):
        self._response = response
        self._exception = exception
        self._callbacks = []  # Store callbacks
        
    def result(self):
        if self._exception:
            raise self._exception
        return self._response
        
    def add_done_callback(self, callback):
        """Store callback for later manual triggering"""
        self._callbacks.append(callback)
        # Execute callback immediately for test simplicity
        callback(self)

# Updated MockNode - not inheriting from Node anymore
class MockNode:
    """Mock node for testing"""
    
    def __init__(self):
        self.client_created = False
        self.subscription_created = False
        self.subscription_topic = None
        self.subscription_callback = None
        self.callbacks = []  # Store callbacks for testing
        
    def create_client(self, srv_type, srv_name):
        self.client_created = True
        self.client_srv_type = srv_type
        self.client_srv_name = srv_name
        
        class MockClient:
            def __init__(self, parent):
                self.parent = parent
                self.request_sent = False
                self.last_request = None
                self.wait_for_service_called = False
                # Create a default future that will be returned by call_async
                self.future = MockFuture(response=EffectorRegister_Response())
            
            def wait_for_service(self, timeout_sec=1.0):
                self.wait_for_service_called = True
                return True
                
            def call_async(self, request):
                self.request_sent = True
                self.last_request = request
                return self.future
        
        self.mock_client = MockClient(self)
        return self.mock_client
    
    def create_subscription(self, msg_type, topic, callback, qos):
        self.subscription_created = True
        self.subscription_topic = topic
        self.subscription_callback = callback
        self.callbacks.append(callback)
        
        class MockSubscription:
            pass
            
        return MockSubscription()
        
    def get_name(self):
        return "test_adaptation_handler"
        
    def get_logger(self):
        class MockLogger:
            def __init__(self):
                self.info_messages = []
                self.warn_messages = []
                self.error_messages = []
                self.debug_messages = []
                
            def info(self, msg):
                self.info_messages.append(msg)
                
            def warn(self, msg):
                self.warn_messages.append(msg)
                
            def error(self, msg):
                self.error_messages.append(msg)
                
            def debug(self, msg):
                self.debug_messages.append(msg)
                
        if not hasattr(self, 'logger'):
            self.logger = MockLogger()
        return self.logger


@pytest.fixture
def mock_node():
    """Create a mock node for testing"""
    return MockNode()


@pytest.fixture
def adaptation_handler(mock_node):
    """Create an adaptation handler with a mock node"""
    return AdaptationHandler(mock_node)


class TestAdaptationHandler:
    """Tests for the AdaptationHandler class"""
    
    def test_initialization(self, adaptation_handler, mock_node):
        """Test initialization of AdaptationHandler"""
        # Verify handler is initialized with the node
        assert adaptation_handler.node == mock_node
        
        # We don't check for client_created here anymore since the client 
        # is only created when register_with_effector is called, not during initialization

    def test_register_with_effector(self, adaptation_handler, mock_node):
        """Test registration with ParamAdapter"""
        # Call registration - this should create the client
        adaptation_handler.register_with_effector()
        
        # Now verify client was created
        assert mock_node.client_created is True
        assert mock_node.client_srv_type == EffectorRegister
        assert mock_node.client_srv_name == "EffectorRegister"
        
        # Setup successful response
        response = EffectorRegister_Response()
        response.ack = True
        mock_node.mock_client.future = MockFuture(response=response)
        
        # Verify service call
        assert mock_node.mock_client.wait_for_service_called is True
        assert mock_node.mock_client.request_sent is True
        assert mock_node.mock_client.last_request is not None
        
        # Verify request contents
        request = mock_node.mock_client.last_request
        assert request.name == "test_adaptation_handler"
        assert request.connection is True
        
        # Simulate callback being called
        adaptation_handler.registration_callback(mock_node.mock_client.future)
        
        # Verify subscription was created
        assert mock_node.subscription_created is True
        assert mock_node.subscription_topic == "reconfigure_test_adaptation_handler"
        assert mock_node.subscription_callback is not None
        
        # Verify log message
        assert "Successfully registered with ParamAdapter" in mock_node.logger.info_messages
        
    def test_failed_registration(self, adaptation_handler, mock_node):
        """Test failed registration with ParamAdapter"""
        # Call registration - this creates the client
        adaptation_handler.register_with_effector()
        
        # Setup failure response
        response = EffectorRegister_Response()
        response.ack = False
        mock_node.mock_client.future = MockFuture(response=response)
        
        # Simulate callback being called
        adaptation_handler.registration_callback(mock_node.mock_client.future)
        
        # Verify subscription was NOT created
        assert not mock_node.subscription_created
        
        # Verify error log message
        assert "Failed to register with ParamAdapter" in mock_node.logger.error_messages

    def test_registration_exception(self, adaptation_handler, mock_node):
        """Test exception during registration with ParamAdapter"""
        # Call registration - this creates the client
        adaptation_handler.register_with_effector()
        
        # Setup exception
        exception = RuntimeError("Test error")
        mock_node.mock_client.future = MockFuture(exception=exception)
        
        # Simulate callback being called
        adaptation_handler.registration_callback(mock_node.mock_client.future)
        
        # Verify error log message
        assert "Test error" in mock_node.logger.error_messages[-1]
        
    def test_process_adaptation_command(self, adaptation_handler, mock_node):
        """Test processing adaptation command"""
        # Create test command
        cmd = AdaptationCommand()
        cmd.source = "test_source"
        cmd.target = "test_adaptation_handler"
        cmd.action = "freq=1.5,replicate=2"
        
        # Process command
        adaptation_handler.process_adaptation_command(cmd)
        
        # Verify log message
        assert "Received adaptation command: freq=1.5,replicate=2" in mock_node.logger.info_messages
        
        # Verify frequency was updated
        assert hasattr(adaptation_handler, 'frequency')
        assert adaptation_handler.frequency == 1.5
        
        # Verify log about frequency change
        assert "Changing frequency to 1.5Hz" in mock_node.logger.info_messages
        
    def test_process_invalid_adaptation_command(self, adaptation_handler, mock_node):
        """Test processing invalid adaptation command"""
        # Create test command with invalid frequency
        cmd = AdaptationCommand()
        cmd.source = "test_source"
        cmd.target = "test_adaptation_handler"
        cmd.action = "freq=invalid,replicate=2"
        
        # Process command
        adaptation_handler.process_adaptation_command(cmd)
        
        # Verify error log message
        assert "Invalid frequency value: invalid" in mock_node.logger.error_messages
        
    def test_multiple_parameters(self, adaptation_handler, mock_node):
        """Test handling multiple parameters in adaptation command"""
        # Create test command with multiple parameters
        cmd = AdaptationCommand()
        cmd.source = "test_source"
        cmd.target = "test_adaptation_handler"
        cmd.action = "freq=2.0,mode=power_saving,priority=high"
        
        # Process command
        adaptation_handler.process_adaptation_command(cmd)
        
        # Verify frequency was updated
        assert adaptation_handler.frequency == 2.0
        
        # Test another command
        cmd.action = "freq=0.5"
        adaptation_handler.process_adaptation_command(cmd)
        
        # Verify frequency was updated again
        assert adaptation_handler.frequency == 0.5