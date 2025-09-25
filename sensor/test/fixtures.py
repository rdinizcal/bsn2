from sensor.sensor import Sensor
from bsn_interfaces.srv import PatientData, EffectorRegister
from ament_index_python.packages import get_package_share_directory
from bsn_interfaces.msg import SensorData
from bsn_interfaces.srv import PatientData, EffectorRegister
import os
import threading
import yaml
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
import rclpy
from rclpy.node import Node

# from shared_components.test_components.shared_fixtures import ros_context
import pytest
import time
class ExecutorThread(threading.Thread):
    def __init__(self, nodes):
        super().__init__(daemon=True)
        self.executor = SingleThreadedExecutor()
        self.nodes = nodes
        for node in self.nodes:
            self.executor.add_node(node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
    def run(self):
        self.executor_thread.start()

    def clean_up(self):
        for node in self.nodes:
            if hasattr(node, "lifecycle_manager"):
                node.lifecycle_manager.shutdown_node()
            self.executor.remove_node(node)
        self.executor.shutdown()
        for node in self.nodes:
            if hasattr(node, "lifecycle_manager"):
                node.lifecycle_manager.shutdown_node()
        self.executor.remove_node(node)

        self.executor.shutdown()
        for node in self.nodes:
            node.destroy_node()
        if self.executor_thread.is_alive():
            self.executor_thread.join(timeout=2.0)
            
def get_param(package, node_name, yaml_file):
    params_path = os.path.join(
        get_package_share_directory(package), "config", yaml_file
    )
    with open(params_path, "r") as f:
        full_params = yaml.safe_load(f)
    ros_params = full_params[node_name]["ros__parameters"]
    return [Parameter(name=k, value=v) for k, v in ros_params.items()]


@pytest.fixture(scope="class")
def sensor_node(request):
    """Create and manage sensor node for testing"""
    # Initialize ROS
    if not rclpy.ok():
        rclpy.init()
    print(f"passed here in initialization")
    # Create a separate node for the mock service
    mock_service_node = Node("mock_service_provider")

    def mock_patient_service(req, res):
        mock_service_node.get_logger().info(f"Mock service called for {req.vital_sign}")
        res.datapoint = 37.0
        return res

    # Add the missing EffectorRegister mock service
    def mock_effector_register_service(req, res):
        mock_service_node.get_logger().info(
            f"Mock EffectorRegister called for {req.name}"
        )
        res.ack = True
        return res

    test_service = mock_service_node.create_service(
        PatientData, "get_sensor_reading", mock_patient_service
    )

    # Add this line to mock the EffectorRegister service
    effector_service = mock_service_node.create_service(
        EffectorRegister, "EffectorRegister", mock_effector_register_service
    )
    print(f"passed here in effector registration")
    # spin the mock service node to handle requests
    params = get_param("sensor", "thermometer_node", "thermometer.yaml")
    # Create node with a custom name to avoid conflicts
    node = Sensor("thermometer_test_node", parameters=params)

    # Enable auto_recovery for testing to prevent hanging
    node.lifecycle_manager.auto_recovery = True

    node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

    threads = ExecutorThread([node, mock_service_node])
    threads.run()

    # Try configuration and capture the result
    try:

        result = node.trigger_configure()
        node.get_logger().info(f"Configuration result: {result.value}")

        # Check if configuration was successful without asserting yet
        if result.value != 1:
            node.get_logger().error(f"Configuration failed with result {result}")
        else:
            node.get_logger().info("Configuration successful")

            # Activate the node if configuration was successful
            act_result = node.trigger_activate()
            if act_result.value != 1:
                node.get_logger().error(f"Activation failed with result {act_result}")
            else:
                node.get_logger().info("Activation successful")
    except Exception as e:
        node.get_logger().error(f"Exception during configuration: {e}")
        # Continue with setup to see what else might be wrong

    # Create a subscription to capture published data
    request.cls.received_messages = []

    def sensor_data_callback(msg):
        node.get_logger().info(f"Received message: {msg.sensor_datapoint}")
        request.cls.received_messages.append(msg)

    # Create subscription - try to handle both component and non-component versions
    try:
        topic = None
        # Try component-based architecture first
        if hasattr(node, "config") and hasattr(node.config, "sensor"):
            topic = f"sensor_data/{node.config.sensor}"
        # Fall back to direct attribute
        elif hasattr(node, "sensor"):
            topic = f"sensor_data/{node.sensor}"
        else:
            # Default if neither is found
            topic = "sensor_data/thermometer"

        sub = node.create_subscription(SensorData, topic, sensor_data_callback, 10)
        node.get_logger().info(f"Created subscription to topic: {topic}")
    except Exception as e:
        node.get_logger().error(f"Failed to create subscription: {e}")
        sub = None

    # Store node, service, and subscription in class
    request.cls.sensor_node = node
    request.cls.mock_service_node = mock_service_node
    request.cls.test_service = test_service
    request.cls.test_sub = sub

    # Make sure the service can be discovered before proceeding
    time.sleep(1.0)  # Give time for service registration

    yield node

    try:
        threads.clean_up()

    except Exception as e:
        print(f"Error during sensor cleanup: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


class SensorTestContext:
    def __init__(self, sensor_node, mock_service_node):
        self.sensor_node = sensor_node
        self.mock_service_node = mock_service_node
        self.processed_value = 0
        self.transferred_msg = 0


@pytest.fixture(scope="function")
def context():
    """Create and manage sensor node for BDD testing (no request.cls)."""
    if not rclpy.ok():
        rclpy.init()
    print(f"passed here in initialization")
    
    mock_service_node = Node("mock_service_provider")

    def mock_patient_service(req, res):
        mock_service_node.get_logger().info(f"Mock service called for {req.vital_sign}")
        res.datapoint = 37.0
        return res

    def mock_effector_register_service(req, res):
        mock_service_node.get_logger().info(
            f"Mock EffectorRegister called for {req.name}"
        )
        res.ack = True
        return res

    test_service = mock_service_node.create_service(
        PatientData, "get_sensor_reading", mock_patient_service
    )
    effector_service = mock_service_node.create_service(
        EffectorRegister, "EffectorRegister", mock_effector_register_service
    )
    print(f"passed here in effector registration")
    
    params = get_param("sensor", "thermometer_node", "thermometer.yaml")
    node = Sensor("thermometer_test_node", parameters=params)
    node.lifecycle_manager.auto_recovery = True
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
    node.get_logger().info("Node created, attempting configuration...")

    # Use ExecutorThread instead of manual threading
    threads = ExecutorThread([mock_service_node, node])
    threads.run()

    if hasattr(node, "trigger_configure"):
        node.trigger_configure()
    time.sleep(0.5)
    node.trigger_activate()

    # Attach received_messages directly to the node for BDD
    node.received_messages = []

    def sensor_data_callback(msg):
        node.get_logger().info(f"Received message: {msg.sensor_datapoint}")
        node.received_messages.append(msg)

    try:
        topic = None
        if hasattr(node, "config") and hasattr(node.config, "sensor"):
            topic = f"sensor_data/{node.config.sensor}"
        elif hasattr(node, "sensor"):
            topic = f"sensor_data/{node.sensor}"
        else:
            topic = "sensor_data/thermometer"
        sub = node.create_subscription(SensorData, topic, sensor_data_callback, 10)
        node.get_logger().info(f"Created subscription to topic: {topic}")
    except Exception as e:
        node.get_logger().error(f"Failed to create subscription: {e}")
        sub = None

    # Wait for service registration
    time.sleep(1.0)

    yield SensorTestContext(node, mock_service_node)

    # Use ExecutorThread cleanup
    try:
        threads.clean_up()
    except Exception as e:
        print(f"Error during sensor cleanup: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


@pytest.fixture(scope="class")
def lifecycle_sensor():
    """Create a fresh sensor node for each test."""
    if not rclpy.ok():
        rclpy.init()

    import random
    random_suffix = str(random.randint(1000, 9999))

    # Create a separate node for the mock service
    mock_service_node = Node(f"mock_service_provider_{random_suffix}")

    def mock_patient_service(req, res):
        mock_service_node.get_logger().info(f"Mock service called for {req.vital_sign}")
        res.datapoint = 37.0
        return res

    def mock_effector_register_service(req, res):
        mock_service_node.get_logger().info(
            f"Mock EffectorRegister called for {req.name}"
        )
        res.ack = True
        return res

    test_service = mock_service_node.create_service(
        PatientData, "get_sensor_reading", mock_patient_service
    )

    effector_service = mock_service_node.create_service(
        EffectorRegister, "EffectorRegister", mock_effector_register_service
    )

    params = get_param("sensor", "thermometer_node", "thermometer.yaml")

    # Create node with a unique name to avoid conflicts
    sensor_name = f"lifecycle_test_node_{random_suffix}"
    node = Sensor(sensor_name, parameters=params)

    # Use ExecutorThread instead of manual threading
    threads = ExecutorThread([mock_service_node, node])
    threads.run()

    # Configure the node and wait a bit
    if hasattr(node, "trigger_configure"):
        node.trigger_configure()
    time.sleep(0.5)

    yield node

    # Use ExecutorThread cleanup
    try:
        threads.clean_up()
    except Exception as e:
        print(f"Error during lifecycle node cleanup: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()
