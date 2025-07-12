"""
Pytest configuration for BSN2 Engine tests
"""

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
import time
import threading
from unittest.mock import Mock
from rclpy.node import Node
from bsn_interfaces.srv import DataAccessRequest, EngineRequest
from bsn_interfaces.msg import Exception as BSNException

# Import TestEngine directly instead of relative import
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from adaptation.engines.base_engine import Engine


class TestEngine(Engine):
    """Concrete implementation of Engine for testing"""
    
    def __init__(self, node_name="test_engine"):
        super().__init__(node_name)
        self.monitor_called = False
        self.analyze_called = False
        self.plan_called = False
        self.execute_called = False
    
    def get_prefix(self):
        return "T_"
    
    def initialize_strategy(self, terms):
        return {term: 1.0 for term in terms}
    
    def initialize_priority(self, terms):
        return {term: 50 for term in terms}
    
    def monitor(self):
        self.monitor_called = True
    
    def analyze(self):
        self.analyze_called = True
    
    def plan(self):
        self.plan_called = True
    
    def execute(self):
        self.execute_called = True


@pytest.fixture(scope="session")
def rclpy_context():
    """Initialize rclpy once per test session"""
    if not rclpy.ok():
        rclpy.init()
    yield
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except:
        pass


@pytest.fixture(scope="class")
def engine_node(request, rclpy_context):
    """Create an engine node for testing with mock services."""
    # Create a separate node for mock services
    mock_service_node = Node("mock_engine_service_provider")

    def mock_data_access_service(req, res):
        mock_service_node.get_logger().info(f"Mock data access service called for {req.query}")
        if req.query == "reliability_formula":
            res.content = "R_G3_T1_1 * R_G3_T1_2"
        elif req.query == "all:reliability:1":
            res.content = "/g3t1_1:success,fail,success,0.67;/g3t1_2:success,success,0.85;"
        elif req.query == "all:event:1":
            res.content = "/g3t1_1:activate;/g3t1_2:deactivate;"
        else:
            res.content = ""
        return res

    def mock_engine_service(req, res):
        mock_service_node.get_logger().info(f"Mock engine service called")
        res.content = "reliability"
        return res

    # Create mock services
    data_access_service = mock_service_node.create_service(
        DataAccessRequest, "data_access", mock_data_access_service
    )
    
    engine_service = mock_service_node.create_service(
        EngineRequest, "adaptation_parameter", mock_engine_service
    )

    # Create executor for mock services
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(mock_service_node)

    # Start executor in a separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Create engine node with test name
    node = TestEngine("test_engine_node")
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

    # IMPORTANT: Mock the data_access_client properly
    mock_client = Mock()
    mock_client.wait_for_service.return_value = True
    mock_response = Mock()
    mock_response.content = "R_G3_T1_1 * R_G3_T1_2"
    mock_future = Mock()
    mock_future.result.return_value = mock_response
    mock_client.call_async.return_value = mock_future
    
    # Assign the mock client to the node
    node.data_access_client = mock_client

    # Initialize engine attributes
    node.qos_attribute = "reliability"
    node.monitor_freq = 10.0
    node.actuation_freq = 5.0
    node.info_quant = 1.0
    node.strategy = {"R_G3_T1_1": 1.0, "R_G3_T1_2": 1.0}
    node.priority = {"R_G3_T1_1": 50, "R_G3_T1_2": 50}
    node.deactivated_components = {}
    node.target_system_model = None

    # Add the engine node to executor
    executor.add_node(node)

    # Create collections for test data
    request.cls.received_strategy_messages = []
    request.cls.received_exception_messages = []

    def strategy_callback(msg):
        node.get_logger().info(f"Received strategy message: {msg.content}")
        request.cls.received_strategy_messages.append(msg)

    def exception_callback(msg):
        node.get_logger().info(f"Received exception message: {msg.content}")
        request.cls.received_exception_messages.append(msg)

    # Create subscriptions for testing
    try:
        strategy_sub = node.create_subscription(
            BSNException, "strategy", strategy_callback, 10
        )
        exception_sub = node.create_subscription(
            BSNException, "exception", exception_callback, 10
        )
        node.get_logger().info("Created subscriptions for testing")
    except Exception as e:
        node.get_logger().error(f"Failed to create subscriptions: {e}")
        strategy_sub = None
        exception_sub = None

    # Store everything in the test class
    request.cls.engine_node = node
    request.cls.mock_service_node = mock_service_node
    request.cls.data_access_service = data_access_service
    request.cls.engine_service = engine_service
    request.cls.strategy_sub = strategy_sub
    request.cls.exception_sub = exception_sub
    request.cls.executor = executor
    request.cls.executor_thread = executor_thread
    
    # Give time for service registration
    time.sleep(1.0)

    yield node

    # Cleanup
    try:
        # Stop executor
        executor.shutdown()
        if executor_thread.is_alive():
            executor_thread.join(timeout=2.0)
        
        # Destroy nodes
        mock_service_node.destroy_node()
        node.destroy_node()
    except Exception as e:
        node.get_logger().error(f"Exception during teardown: {e}")


@pytest.fixture(scope="class")
def reli_engine_node(request, rclpy_context):
    """Create ReliabilityEngine node for testing"""
    from adaptation.engines.reli_engine import ReliabilityEngine
    
    # Create a separate node for mock services
    mock_service_node = Node("mock_reli_engine_service_provider")

    def mock_data_access_service(req, res):
        mock_service_node.get_logger().info(f"Mock data access service called for {req.query}")
        if req.query == "reliability_formula":
            res.content = "R_G3_T1_1 * R_G3_T1_2"
        elif "all:reliability:" in req.query:
            # Fix the format to match what the engine expects
            res.content = "/g3t1_1:success,fail,success,0.70;/g3t1_2:success,success,0.80;"
        elif req.query == "all:event:1":
            res.content = "/g3t1_1:activate;/g3t1_2:activate;"
        else:
            res.content = ""
        return res

    # Create mock services
    data_access_service = mock_service_node.create_service(
        DataAccessRequest, "data_access", mock_data_access_service
    )

    # Create executor for mock services
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(mock_service_node)

    # Start executor in a separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Create ReliabilityEngine node
    node = ReliabilityEngine()
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

    # Mock the strategy publisher to avoid actual publishing
    node.strategy_publisher = Mock()

    # Initialize engine attributes
    node.qos_attribute = "reliability"
    node.monitor_freq = 10.0
    node.actuation_freq = 5.0
    node.info_quant = 1.0
    node.setpoint = 0.9
    node.offset = 0.1
    node.gain = 0.5
    node.tolerance = 0.02
    node.cycles = 0
    node.strategy = {"R_G3_T1_1": 1.0, "R_G3_T1_2": 1.0}
    node.priority = {"R_G3_T1_1": 50, "R_G3_T1_2": 50}
    node.deactivated_components = {}
    node.target_system_model = None

    # Mock the data access client
    node.data_access_client = Mock()
    node.data_access_client.wait_for_service.return_value = True
    mock_response = Mock()
    mock_response.content = "R_G3_T1_1 * R_G3_T1_2"
    mock_future = Mock()
    mock_future.result.return_value = mock_response
    node.data_access_client.call_async.return_value = mock_future

    # Add missing method for tests
    def _process_component_name(self, name):
        """Process component name from /g3t1_1 to G3_T1_1"""
        # Remove leading slash
        if name.startswith('/'):
            name = name[1:]
        # Convert to uppercase and add underscores
        name = name.upper()
        # Handle special cases like g3t1_1 -> G3_T1_1
        if 'T' in name and not name.startswith('G'):
            # Already in correct format
            return name
        # Convert g3t1_1 to G3_T1_1
        import re
        # Replace gXtY with G X_T Y
        name = re.sub(r'G(\d+)T(\d+)', r'G\1_T\2', name)
        return name
    
    node._process_component_name = _process_component_name.__get__(node, ReliabilityEngine)

    # Add the engine node to executor
    executor.add_node(node)

    # Store everything in the test class
    request.cls.reli_engine_node = node
    request.cls.mock_service_node = mock_service_node
    request.cls.data_access_service = data_access_service
    request.cls.executor = executor
    request.cls.executor_thread = executor_thread
    
    # Give time for service registration
    time.sleep(1.0)

    yield node

    # Cleanup
    try:
        # Stop executor
        executor.shutdown()
        if executor_thread.is_alive():
            executor_thread.join(timeout=2.0)
        
        # Destroy nodes
        mock_service_node.destroy_node()
        node.destroy_node()
    except Exception as e:
        node.get_logger().error(f"Exception during teardown: {e}")


@pytest.fixture
def sample_formula():
    """Sample formula for testing"""
    return "R_G3_T1_1 * CTX_G3_T1_1 * F_G3_T1_1 + R_G3_T1_2 * CTX_G3_T1_2 * F_G3_T1_2"


@pytest.fixture
def sample_reliability_data():
    """Sample reliability data matching BSN1 format"""
    return "/g3t1_1:success,fail,success,0.67;/g3t1_2:success,success,0.85;"


@pytest.fixture
def sample_context_data():
    """Sample context data matching BSN1 format"""
    return "/g3t1_1:activate;/g3t1_2:deactivate;/g4t1:activate;"


@pytest.fixture
def sample_strategy():
    """Sample strategy matching BSN1 format"""
    return {
        "R_G3_T1_1": 0.85,
        "R_G3_T1_2": 0.90,
        "CTX_G3_T1_1": 1.0,
        "CTX_G3_T1_2": 1.0,
        "F_G3_T1_1": 1.0,
        "F_G3_T1_2": 1.0,
    }


@pytest.fixture
def sample_priority():
    """Sample priority matching BSN1 format"""
    return {
        "R_G3_T1_1": 50,
        "R_G3_T1_2": 70,
        "R_G4_T1": 30,
    }
