from rclpy.node import Node
import rclpy
import threading
from bsn_interfaces.srv import PatientData
from bsn_interfaces.msg import SensorData
import rclpy.parameter

class Sensor(Node):

    def __init__(self, node_name: str):
        super().__init__(node_name)  # Use the node's name as provided in the launch file
        self.declare_parameter('sensor', '')
        self.declare_parameter('vital_sign', '')
        self.declare_parameter('frequency', '1.0')

        self.sensor = self.get_parameter('sensor').get_parameter_value().string_value
        self.vital_sign = self.get_parameter('vital_sign').get_parameter_value().string_value
        self.get_logger().debug(f'vital sign in Sensor {self.get_name()}: {self.vital_sign}')
        self.frequency = float(self.get_parameter('frequency').get_parameter_value().string_value)

        self.cli = self.create_client(PatientData, 'get_sensor_reading')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.req = PatientData.Request()
        self.publisher_ = self.create_publisher(SensorData, f'sensor_data/{self.sensor}', 10)

    def collect(self):
        self.req.vital_sign = self.vital_sign
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

    def spin_sensor(self):
        rate = self.create_rate(self.frequency)
        while rclpy.ok():
            datapoint = self.collect()
            datapoint = self.process(datapoint)
            self.transfer(datapoint)
            rate.sleep()

def main(args=None):
    rclpy.init(args=args)

    # Get node name from CLI remapping (set in the launch file)
    
    
    sensor_node = Sensor(node_name='sensor_node')  # pass the resolved name
    
    thread = threading.Thread(target=rclpy.spin, args=(sensor_node, ), daemon=True)
    thread.start()

    try:
        sensor_node.spin_sensor()
    finally:
        sensor_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()