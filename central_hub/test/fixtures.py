import pytest
import time
import rclpy
import math
import yaml
import threading
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.executors import SingleThreadedExecutor
from bsn_interfaces.msg import SensorData, TargetSystemData
from std_msgs.msg import Header
import os
from ament_index_python.packages import get_package_share_directory
from shared_components.test_components.test_utilities import (
    ExecutorThread,
    get_param,
    ensure_ros_init,
    wait_for_service_registration,
    shutdown_ros_init,
)
from shared_components.test_components.node_setups import (
    setup_lifecycle_sensor_node,  # Move this here
    # Add any other functions from node_setups
)
from central_hub.central_hub import CentralHub

@pytest.fixture(scope="module")
def ros_context():
    """Initialize ROS once for all tests in this module."""
    try:
        if not rclpy.ok():
            rclpy.init()
    except:
        pass
    yield


@pytest.fixture(scope="class")
def central_hub_node(request):
    """Create a central hub node for testing."""
    ensure_ros_init()
    # Create node and assign to class
    node = CentralHub()
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
    
    # Create a separate executor for this node
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # Start executor in a thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    # Initialize class attributes for the test class
    request.cls.central_hub = node
    request.cls.publishers = {}
    request.cls.published_messages = []

    # Configure and activate if the node has these methods
    if hasattr(node, 'trigger_configure'):
        node.get_logger().info("Configuring central hub node...")
        node.trigger_configure()
        time.sleep(0.5)
    
    if hasattr(node, 'trigger_activate'):
        node.get_logger().info("Activating central hub...")
        node.trigger_activate()
        time.sleep(0.5)
    
    # Create subscription to capture published TargetSystemData
    def target_system_callback(msg):
        node.get_logger().debug(f"Test captured TargetSystemData: patient_status={msg.patient_status}")
        request.cls.published_messages.append(msg)
    
    sub = node.create_subscription(
        TargetSystemData, 'target_system_data', target_system_callback, 10
    )
    
    # Store important objects for cleanup
    node.test_sub = sub
    node.executor = executor
    node.executor_thread = executor_thread

    # Yield the configured node
    yield node

    # Cleanup after tests
    try:
        node.get_logger().info("Cleaning up central hub node...")
        
        if hasattr(node, 'trigger_deactivate') and hasattr(node, 'active') and node.active:
            try:
                node.trigger_deactivate()
                time.sleep(0.1)
            except Exception as e:
                node.get_logger().warning(f"Deactivation failed: {e}")
        
        if hasattr(node, 'trigger_cleanup'):
            try:
                node.trigger_cleanup()
                time.sleep(0.1)
            except Exception as e:
                node.get_logger().warning(f"Cleanup failed: {e}")
        
        executor.shutdown()
        if executor_thread.is_alive():
            executor_thread.join(timeout=2.0)
        
        node.destroy_node()
        
    except Exception as e:
        print(f"Error during central hub cleanup: {e}")
    finally:
        shutdown_ros_init()
        
@pytest.fixture(scope="function")
def direct_central_hub():
    """Create a fresh central hub node for direct testing"""
    ensure_ros_init()
    
    # Create node
    node = CentralHub()
    node.get_logger().info("Direct test: Creating Central Hub node")
    
    # Create executor
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    
    # Start executor thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    # Store executor and thread for cleanup
    node.executor = executor
    node.executor_thread = executor_thread
    
    yield node
    
    # Cleanup
    node.get_logger().info("Direct test: Cleaning up Central Hub node")
    
    # Clean shutdown
    try:
        # Deactivate if active
        if hasattr(node, 'active') and node.active:
            node.get_logger().info("Deactivating node")
            node.trigger_deactivate()
            time.sleep(0.2)
        
        # Finalize explicitly to test finalization
        if hasattr(node, 'trigger_cleanup'):
            node.get_logger().info("Cleaning up node")
            node.trigger_cleanup()
            time.sleep(0.2)
    except Exception as e:
        node.get_logger().error(f"Error in lifecycle transitions: {e}")
    
    # Stop executor and join thread
    executor.shutdown()
    if executor_thread.is_alive():
        executor_thread.join(timeout=1.0)
    
    # Destroy node
    node.destroy_node()
    shutdown_ros_init()

@pytest.fixture(scope="class")
def unified_central_hub_context(request):
    """Unified fixture for central hub tests"""
    ensure_ros_init()
    
    # Create central hub node
    hub = CentralHub()
    hub.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
    
    # Use ExecutorThread for proper management
    threads = ExecutorThread([hub])  # <- Use 'threads' like other fixtures
    threads.start()
    
    # Configure and activate
    hub.trigger_configure()
    time.sleep(0.2)
    hub.trigger_activate()
    time.sleep(0.2)
    
    # Create publishers for sensor data
    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )
    
    publishers = {}
    for sensor_type in ["thermometer", "ecg", "oximeter", "abps", "abpd", "glucosemeter"]:
        publishers[sensor_type] = hub.create_publisher(
            SensorData, f"sensor_data/{sensor_type}", qos_profile
        )
    
    # Create message collection for published data
    published_messages = []
    
    def target_system_callback(msg):
        hub.get_logger().debug(f"Test captured TargetSystemData: patient_status={msg.patient_status}")
        published_messages.append(msg)
    
    sub = hub.create_subscription(
        TargetSystemData, 'target_system_data', target_system_callback, 10
    )
    
    # Wait for everything to be ready
    time.sleep(0.5)
    
    # THIS IS THE KEY PART - Set class attributes like central_hub_node fixture does
    request.cls.central_hub = hub
    request.cls.publishers = publishers  # <- This was missing!
    request.cls.published_messages = published_messages  # <- This was missing!
    request.cls.threads = threads
    request.cls.test_sub = sub
    
    # Create context object
    context = UnifiedCentralHubContext(
        central_hub=hub,
        publishers=publishers,
        published_messages=published_messages,
        subscription=sub,
        threads=threads  # <- Use 'threads' not 'executor_thread'
    )
    
    yield context
    
    # Cleanup
    try:
        threads.clean_up()  # <- Use threads.clean_up() like other fixtures
    except Exception as e:
        print(f"Error during unified context cleanup: {e}")


class UnifiedCentralHubContext:
    """Unified context for central hub tests"""
    
    def __init__(self, central_hub, publishers, published_messages, subscription, threads):
        self.central_hub = central_hub
        self.publishers = publishers
        self.published_messages = published_messages
        self.subscription = subscription
        self.threads = threads  # <- Use 'threads' not 'executor_thread'

