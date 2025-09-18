from sensor.sensor import Sensor
from bsn_interfaces.srv import PatientData, EffectorRegister
from ament_index_python.packages import get_package_share_directory
import os
import yaml
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
import rclpy
from rclpy.node import Node
from shared_components.test_components.shared_fixtures import ros_context

@pytest.fixture(scope="class")
def sensor_node(request):
    """Create and manage sensor node for testing"""
    # Initialize ROS
    rclpy.init()
    print(f'passed here in initialization')
    # Create a separate node for the mock service
    mock_service_node = Node("mock_service_provider")

    def mock_patient_service(req, res):
        mock_service_node.get_logger().info(f"Mock service called for {req.vital_sign}")
        res.datapoint = 37.0
        return res

    # Add the missing EffectorRegister mock service
    def mock_effector_register_service(req, res):
        mock_service_node.get_logger().info(f"Mock EffectorRegister called for {req.name}")
        res.ack = True
        return res

    test_service = mock_service_node.create_service(
        PatientData, 'get_sensor_reading', mock_patient_service
    )
    
    # Add this line to mock the EffectorRegister service
    effector_service = mock_service_node.create_service(
        EffectorRegister, 'EffectorRegister', mock_effector_register_service
    )
    print(f'passed here in effector registration')
    # spin the mock service node to handle requests
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(mock_service_node)

    # Start executor in a separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Load params from YAML file - we'll use thermometer for testing
    params_path = os.path.join(
        get_package_share_directory("sensor"), "config", "thermometer.yaml"
    )
    with open(params_path, "r") as f:
        full_params = yaml.safe_load(f)

    # Prepare parameters
    ros_params = full_params["thermometer_node"]["ros__parameters"]
    params = [Parameter(name=k, value=v) for k, v in ros_params.items()]
    
    # Log the parameters we're using
    print("\nUsing parameters:")
    for p in params:
        print(f"  {p.name}: {p.value}")

    # Create node with a custom name to avoid conflicts
    node = Sensor("thermometer_test_node", parameters=params)
    
    # Enable auto_recovery for testing to prevent hanging
    node.lifecycle_manager.auto_recovery = True
    
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

    # Log basic information without using get_parameter_names()
    node.get_logger().info("Node created, attempting configuration...")

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

        # Add the node to executor regardless
        executor.add_node(node)
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
        if hasattr(node, 'config') and hasattr(node.config, 'sensor'):
            topic = f"sensor_data/{node.config.sensor}"
        # Fall back to direct attribute
        elif hasattr(node, 'sensor'):
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
    request.cls.executor = executor
    request.cls.executor_thread = executor_thread
    
    # Make sure the service can be discovered before proceeding
    time.sleep(1.0)  # Give time for service registration

    yield node

   
    try:
        # Shutdown the node first
        if hasattr(node, 'lifecycle_manager'):
            node.lifecycle_manager.shutdown_node()
        
        # Remove node from executor
        executor.remove_node(node)
        
        # Shutdown executor
        executor.shutdown()
        
        # Destroy the node
        node.destroy_node()
        
        # Wait for executor thread to finish
        if executor_thread.is_alive():
            executor_thread.join(timeout=2.0)
            
    except Exception as e:
        print(f"Error during sensor cleanup: {e}")
    finally:
        rclpy.shutdown()

@pytest.fixture
def lifecycle_sensor(ros_context):
    """Create a fresh sensor node for each test."""
    # Create a unique node name for each test to avoid conflicts
    import random
    random_suffix = str(random.randint(1000, 9999))
    
    # Create a separate node for the mock service
    mock_service_node = Node(f"mock_service_provider_{random_suffix}")

    def mock_patient_service(req, res):
        mock_service_node.get_logger().info(f"Mock service called for {req.vital_sign}")
        res.datapoint = 37.0
        return res
    def mock_effector_register_service(req, res):
        mock_service_node.get_logger().info(f"Mock EffectorRegister called for {req.name}")
        res.ack = True
        return res

    test_service = mock_service_node.create_service(
        PatientData, "get_sensor_reading", mock_patient_service
    )
    
    effector_service = mock_service_node.create_service(
        EffectorRegister, 'EffectorRegister', mock_effector_register_service
    )
    
    params_path = os.path.join(
        get_package_share_directory("sensor"), "config", "thermometer.yaml"
    )
    with open(params_path, "r") as f:
        full_params = yaml.safe_load(f)

    # Prepare parameters
    ros_params = full_params["thermometer_node"]["ros__parameters"]
    
    # Fix any topic names that could have trailing slashes
    for key, value in ros_params.items():
        if isinstance(value, str) and value.endswith('/'):
            ros_params[key] = value[:-1]
    
    params = [Parameter(name=k, value=v) for k, v in ros_params.items()]
    
    # Create node with a unique name to avoid conflicts
    sensor_name = f"lifecycle_test_node_{random_suffix}"
    node = Sensor(sensor_name, parameters=params)
    
    # Set up executor
    executor = SingleThreadedExecutor()
    executor.add_node(mock_service_node)
    executor.add_node(node)

    # Start executor in a separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    # Configure the node and wait a bit
    if hasattr(node, 'trigger_configure'):
        node.trigger_configure()
    time.sleep(0.5)  # Give time for configuration
    
    # Store services and nodes as attributes for cleanup
    node.mock_service_node = mock_service_node
    node.test_service = test_service
    node.executor_thread = executor_thread
    node.executor = executor
    
    # Yield the sensor node for testing
    yield node
    
    # Always clean up properly
    try:
        # Shutdown executor first
        executor.shutdown()
        if executor_thread.is_alive():
            executor_thread.join(timeout=1.0)
            
        # Always destroy nodes
        mock_service_node.destroy_node()
        node.destroy_node()
    except Exception as e:
        print(f"Error during lifecycle node cleanup: {e}")
