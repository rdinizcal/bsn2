import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from bsn_interfaces.msg import SensorData
import threading
from collections import deque

class CentralHub(Node):

    def __init__(self):
        super().__init__('central_hub')
        self.subscription = self.create_subscription(SensorData,'sensor_data/thermometer',self.receive_datapoint,10)
        self.subscription  # prevent unused variable warning  
        
        self.window_size = 5
        self.latest_temp = None    
        self.data_window = deque(maxlen=self.window_size)  

    def receive_datapoint(self, msg):
        self.get_logger().info('++Received Datapoint++')
        self.get_logger().info(f'data from: {msg.sensor_type} {msg.sensor_datapoint}')
        self.latest_temp = msg
        
        self.data_window.append(msg.sensor_datapoint)
    
    def fuse_data(self, data:int):
        if len(self.data_window) < 5 :
            self.get_logger().info('Await for more results')
            return None
        avg_risk = sum(self.data_window) / len(self.data_window)
        # Implement MVA
        if 35 <= avg_risk < 36:
            risk_level = 'moderate'
        elif 36 <= avg_risk <= 38:
            risk_level = 'normal'
        elif 38 < avg_risk <= 40:
            risk_level = 'moderate'
        else:
            risk_level = 'high'
        
        self.get_logger().info(f'Fused Data: {avg_risk}°C - Risk Level: {risk_level}')
        return risk_level
    
    def emit_alert(self, data:int):
        self.get_logger().info('++Emit Alert++')
        risk = self.fuse_data()
        if risk == 'high':
            self.get_logger().warning('ALERT: High Temperature Detected!')
        elif risk == 'moderate':
            self.get_logger().info('Warning: Abnormal temperature detected.')
        else:
            self.get_logger().info('no Data')

def main(args=None):
    rclpy.init(args=args)

    emergency_detector = CentralHub()

    # Run spin in a thread, make thread daemon so we don't have to join it to exit
    thread = threading.Thread(target=rclpy.spin, args=(emergency_detector, ), daemon=True)
    thread.start()
    rate = emergency_detector.create_rate(10) # 1 Hz

    while rclpy.ok():
        emergency_detector.fuse_data()
        emergency_detector.emit_alert()
        rate.sleep()

    sensor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()