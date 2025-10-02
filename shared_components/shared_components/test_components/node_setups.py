import pytest
from .shared_fixtures import mock_service_node
from sensor.sensor import Sensor
from central_hub.central_hub import CentralHub
from bsn_interfaces.srv import PatientData, EffectorRegister
from bsn_interfaces.msg import SensorData
from rclpy.node import Node
import rclpy
from .test_utilities import ensure_ros_init, get_param


def setup_lifecycle_sensor_node():
    """Setup function to initialize sensor node for tests"""
    ensure_ros_init()

    # Create a separate node for the mock service
    mock_service_node = Node("mock_service_provider")

    def mock_patient_service(req, res):
        mock_service_node.get_logger().info(f"Mock service called for {req.vital_sign}")
        res.datapoint = 37.0
        return res

    # Create services
    test_service = mock_service_node.create_service(
        PatientData, "get_sensor_reading", mock_patient_service
    )

    # Load parameters and create sensor node
    params = get_param("sensor", "thermometer_node", "thermometer.yaml")
    node = Sensor("thermometer_test_node", parameters=params)
    node.lifecycle_manager.auto_recovery = True

    return node, mock_service_node
def setup_lifecycle_central_hub_node():
    """Setup function to initialize central hub node for tests"""
    ensure_ros_init()
    # Load parameters and create central hub node
    node = CentralHub()
    node.lifecycle_manager.auto_recovery = True
    
    return node

def setup_effector_register_service():
    """Setup function to initialize effector register service for tests"""
    ensure_ros_init()

    # Create a separate node for the mock service
    mock_effector_register_provider = Node("mock_effector_register_provider")

    def mock_effector_register_service(req, res):
        mock_effector_register_provider.get_logger().info(
            f"Mock EffectorRegister called for {req.name}"
        )
        res.ack = True
        return res

    # Create services
    effector_service = mock_effector_register_provider.create_service(
        EffectorRegister, "EffectorRegister", mock_effector_register_service
    )

    return mock_effector_register_provider