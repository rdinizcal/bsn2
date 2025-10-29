import pytest
import time
from sensor.sensor import Sensor
from central_hub.central_hub import CentralHub
from bsn_interfaces.srv import PatientData, EffectorRegister
from bsn_interfaces.msg import SensorData
from rclpy.node import Node
import rclpy
from shared_components.test_components.test_utilities import (
    ExecutorThread,
    get_param,
    ensure_ros_init,
    wait_for_service_registration,
    shutdown_ros_init,
)
from shared_components.test_components.node_setups import (
    setup_lifecycle_sensor_node,
    setup_effector_register_service,
    setup_lifecycle_central_hub_node,
)


class SensorTestContext:
    def __init__(self, central_hub_node, sensor_node, mock_service_node):
        self.central_hub_node: CentralHub = central_hub_node
        self.sensor_node: Sensor = sensor_node
        self.mock_service_node = mock_service_node
        self.processed_value = 0
        self.transferred_msg = 0
        self.test_data = {}


@pytest.fixture(scope="class")
def sensor_node(request):
    """Create and manage sensor node for testing"""
    ensure_ros_init()
    mock_effector_register = setup_effector_register_service()
    main_node, mock_service_node = setup_lifecycle_sensor_node()
    threads: ExecutorThread = ExecutorThread(
        [mock_effector_register, mock_service_node, main_node]
    )
    
    threads.run()
    
    try:
        result = main_node.trigger_configure()
        
        # Start executor thread AFTER configuration
        if result.value == 1:
            act_result = main_node.trigger_activate()
            if act_result.value != 1:
                main_node.get_logger().error(
                    f"Activation failed with result {act_result}"
                )
        else:
            main_node.get_logger().error(f"Configuration failed with result {result}")
    except Exception as e:
        main_node.get_logger().error(f"Exception during setup: {e}")
    wait_for_service_registration(0.5)
    request.cls.sensor_node = main_node
    request.cls.mock_service_node = mock_service_node
    yield main_node

    # Cleanup using shared ExecutorThread
    try:
        threads.clean_up()
    except Exception as e:
        print(f"Error during sensor cleanup: {e}")
    finally:
        shutdown_ros_init()


@pytest.fixture(scope="module")
def context():
    """Create and manage sensor node for BDD testing"""
    ensure_ros_init()
    mock_effector_register_provider = setup_effector_register_service()
    main_node, mock_service_node = setup_lifecycle_sensor_node()
    print("Effector register service setup complete.")
    central_hub_node = setup_lifecycle_central_hub_node()
    threads: ExecutorThread = ExecutorThread(
        [
            mock_effector_register_provider,
            mock_service_node,
            main_node,
            central_hub_node,
        ]
    )
    
    # Start executor thread first (original pattern)
    threads.run()
    time.sleep(0.1)  # Allow executor to start
    
    # Configure and activate nodes
    if hasattr(main_node, "trigger_configure"):
        main_node.trigger_configure()
        central_hub_node.trigger_configure()
    time.sleep(0.2)  # Allow configuration to complete
    
    # Activate nodes
    main_node.trigger_activate()
    central_hub_node.trigger_activate()
    wait_for_service_registration(0.5)

    yield SensorTestContext(central_hub_node, main_node, mock_service_node)

    try:
        threads.clean_up()
    except Exception as e:
        print(f"Error during sensor cleanup: {e}")
    finally:
        shutdown_ros_init()


@pytest.fixture(scope="class")
def lifecycle_sensor():
    """Create a fresh sensor node for each test."""
    ensure_ros_init()
    mock_effector_service = setup_effector_register_service()
    main_node, mock_service_node = setup_lifecycle_sensor_node()
    threads: ExecutorThread = ExecutorThread(
        [mock_effector_service, mock_service_node, main_node]
    )
    
    # Configure the node BEFORE starting executor thread
    if hasattr(main_node, "trigger_configure"):
        main_node.trigger_configure()
    time.sleep(0.2)  # Allow configuration to complete
    
    # Start executor thread AFTER configuration
    threads.run()
    time.sleep(0.3)  # Allow executor to start

    yield main_node

    # Use ExecutorThread cleanup
    try:
        threads.clean_up()
    except Exception as e:
        print(f"Error during lifecycle node cleanup: {e}")
    finally:
        shutdown_ros_init()
