import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import threading


class CentralHub(Node):

    def __init__(self):
        super().__init__('central_hub')
        self.subscription = self.create_subscription(String,'sensor_data/thermometer',self.receive_datapoint,10)
        self.subscription  # prevent unused variable warning        

    def receive_datapoint(self, msg):
        self.get_logger().info('++Receive Datapoint++')
        self.get_logger().info('I heard: "%s"' % msg.data)
        pass
    
    def fuse_data(self, data:int):
        self.get_logger().info('++Fuse Data++')
        pass
    
    def emit_alert(self, data:int):
        self.get_logger().info('++Emit Alert++')
        pass

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