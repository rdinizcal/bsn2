import rclpy
from rclpy.node import Node

from bsn_interfaces.srv import PatientData
from bsn_interfaces.msg import SensorData

import threading

class Sensor(Node):

    def __init__(self):
        super().__init__('sensor')
        self.cli = self.create_client(PatientData, 'get_sensor_reading')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = PatientData.Request()

        self.publisher_ = self.create_publisher(SensorData, 'sensor_data/thermometer', 10)

    def collect(self):
        self.req.vital_sign = "temperature" 
        
        response = self.cli.call(self.req)
        self.get_logger().info('++Collect++\n new data collected: [%s]' % response.datapoint)
        
        return response.datapoint
    
    def process(self, datapoint:float):
        self.get_logger().info('++Process++')
        return datapoint
    
    def transfer(self, datapoint:float):
        msg = SensorData()
        msg.sensor_datapoint = datapoint
        self.publisher_.publish(msg)
        self.get_logger().info('++Transfer++\n Publishing: "%s"' % msg.sensor_datapoint)

def main(args=None):
    rclpy.init(args=args)

    sensor = Sensor()

    # Run spin in a thread, make thread daemon so we don't have to join it to exit
    thread = threading.Thread(target=rclpy.spin, args=(sensor, ), daemon=True)
    thread.start()
    rate = sensor.create_rate(1) # 1 Hz

    while rclpy.ok():
        datapoint = sensor.collect()
        datapoint = sensor.process(datapoint=datapoint)
        sensor.transfer(datapoint=datapoint)
        rate.sleep()

    sensor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()