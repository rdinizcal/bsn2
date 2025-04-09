from rclpy.node import Node
from abc import ABC, abstractmethod

from bsn_interfaces.srv import PatientData
from bsn_interfaces.msg import SensorData

class Sensor(Node, ABC):

    def __init__(self, node_name: str, sensor: str):
        super().__init__(node_name)
        self.cli = self.create_client(PatientData, 'get_sensor_reading')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.sensor = sensor
        self.req = PatientData.Request()
        self.publisher_ = self.create_publisher(SensorData, 'sensor_data/' + sensor, 10)

    @abstractmethod
    def get_vital_sign(self) -> str:
        pass

    def collect(self):
        self.req.vital_sign = self.get_vital_sign()
        response = self.cli.call(self.req)
        self.get_logger().info(f'++Collect++\n new data from {self.req.vital_sign} collected: [{response.datapoint}]')
        return response.datapoint

    def process(self, datapoint: float):
        return datapoint

    def transfer(self, datapoint: float):
        msg = SensorData()
        msg.sensor_type = self.sensor
        msg.sensor_datapoint = datapoint
        self.publisher_.publish(msg)
        self.get_logger().info(f'++Transfer++\n Publishing: "{msg.sensor_datapoint}"')
