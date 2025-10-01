import pytest
from sensor.sensor import Sensor
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

    def mock_effector_register_service(req, res):
        mock_service_node.get_logger().info(
            f"Mock EffectorRegister called for {req.name}"
        )
        res.ack = True
        return res

    # Create services
    test_service = mock_service_node.create_service(
        PatientData, "get_sensor_reading", mock_patient_service
    )
    effector_service = mock_service_node.create_service(
        EffectorRegister, "EffectorRegister", mock_effector_register_service
    )

    # Load parameters and create sensor node
    params = get_param("sensor", "thermometer_node", "thermometer.yaml")
    node = Sensor("thermometer_test_node", parameters=params)
    node.lifecycle_manager.auto_recovery = True

    return node, mock_service_node
