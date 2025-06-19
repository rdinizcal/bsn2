import rclpy
from rclpy.node import Node
from bsn_interfaces.msg import Status, Event, EnergyStatus
import threading

class Probe(Node):
    """
    Probe (collector) component for BSN system in ROS2
    
    Collects messages from components and forwards them to Logger
    Acts as a centralized collection point for system monitoring
    """

    def __init__(self):
        super().__init__('probe')
        self.get_logger().info("Starting BSN Probe")
        
        self.declare_parameter("frequency", 1.0).value
        self.frequency = self.get_parameter("frequency").value
        # Create publishers to log_* topics for the Logger
        self.log_status_pub = self.create_publisher(
            Status, 'log_status', 10)
            
        self.log_event_pub = self.create_publisher(
            Event, 'log_event', 10)
            
        self.log_energy_pub = self.create_publisher(
            EnergyStatus, 'log_energy_status', 10)
        
        # Create subscribers for component messages
        self.status_sub = self.create_subscription(
            Status, 'component_status', self.collect_status, 10)
            
        self.event_sub = self.create_subscription(
            Event, 'collect_event', self.collect_event, 10)
            
        self.energy_sub = self.create_subscription(
            EnergyStatus, 'collect_energy_status', self.collect_energy_status, 10)
            
        self.get_logger().info('Probe initialized and ready to collect messages')

    def collect_status(self, msg):
        """Forward status message to Logger"""
        self.get_logger().debug(f"Collected status from {msg.source}")
        self.log_status_pub.publish(msg)
    
    def collect_event(self, msg):
        """Forward event message to Logger"""
        self.get_logger().debug(f"Collected event from {msg.source}")
        self.log_event_pub.publish(msg)
    
    def collect_energy_status(self, msg):
        """Forward energy status message to Logger"""
        self.get_logger().debug(f"Collected energy status from {msg.source}")
        self.log_energy_pub.publish(msg)

    def spin_collector(self):
        """
        Simple rate-limited spin loop for the collector
        Uses the configured frequency parameter
        """
        
        rate = self.create_rate(self.frequency)
        while rclpy.ok():
            # Instead of calling gen_data, just log a heartbeat message
            self.get_logger().debug("Collector heartbeat")
            
            # Sleep at the configured frequency
            rate.sleep()


def main(args=None):
    rclpy.init(args=args)

    collector = Probe()

    
    thread = threading.Thread(target=rclpy.spin, args=(collector,), daemon=True)
    thread.start()

    try:
        collector.spin_collector()
    finally:
        collector.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()