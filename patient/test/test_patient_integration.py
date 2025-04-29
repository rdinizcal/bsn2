import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from patient.patient import Patient
from bsn_interfaces.srv import PatientData


@pytest.fixture
def patient_node_spin():
    rclpy.init()
    executor = SingleThreadedExecutor()
    node = Patient()
    executor.add_node(node)

    import threading

    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    yield node

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


def test_service_response(patient_node_spin):
    client_node = Node("test_client")
    client = client_node.create_client(PatientData, "get_sensor_reading")

    assert client.wait_for_service(timeout_sec=3.0), "Service not available."

    req = PatientData.Request()
    req.vital_sign = patient_node_spin.vital_signs[0]

    future = client.call_async(req)
    rclpy.spin_until_future_complete(client_node, future, timeout_sec=5.0)

    assert future.done()
    response = future.result()
    assert response is not None
    assert response.datapoint >= 0.0

    client_node.destroy_node()
