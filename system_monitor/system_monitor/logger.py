import rclpy
from rclpy.node import Node
from bsn_interfaces.msg import Status, Event, EnergyStatus, AdaptationCommand, Uncertainty, Persist
import threading
class Logger(Node):
    def __init__(self):
        super().__init__('logger')
        self.declare_parameter("frequency", 2.0)  # Default frequency of 2 Hz
        self.frequency = self.get_parameter("frequency").value
        # Store time reference
        self.time_ref = self.get_clock().now().nanoseconds
        
        # Create persist publisher
        self.persist_pub = self.create_publisher(Persist, 'persist', 10)
        
        # Create subscribers for all message types
        self.status_sub = self.create_subscription(
            Status, 'log_status', self.receive_status, 1000)
            
        self.event_sub = self.create_subscription(
            Event, 'log_event', self.receive_event, 1000)
            
        self.energy_sub = self.create_subscription(
            EnergyStatus, 'log_energy_status', self.receive_energy, 1000)
            
        self.adapt_sub = self.create_subscription(
            AdaptationCommand, 'log_adapt', self.receive_adapt, 1000)
            
        self.uncertainty_sub = self.create_subscription(
            Uncertainty, 'log_uncertainty', self.receive_uncertainty, 1000)
            
        self.get_logger().info('Logger started')
    
    def now(self):
        """Get current time in milliseconds"""
        return self.get_clock().now().nanoseconds // 1000000
    
    def receive_status(self, msg):
        """Process a status message"""
        persist_msg = Persist()
        persist_msg.source = msg.source
        persist_msg.target = msg.target
        persist_msg.type = "Status"
        persist_msg.timestamp = self.now() - (self.time_ref // 1000000)
        persist_msg.content = msg.content
        
        self.persist_pub.publish(persist_msg)
    
    def receive_event(self, msg):
        """Process an event message"""
        persist_msg = Persist()
        persist_msg.source = msg.source
        persist_msg.target = msg.target
        persist_msg.type = "Event"
        persist_msg.timestamp = self.now() - (self.time_ref // 1000000)
        persist_msg.content = msg.content
        
        self.persist_pub.publish(persist_msg)
    
    def receive_energy(self, msg):
        """Process an energy status message"""
        persist_msg = Persist()
        persist_msg.source = msg.source
        persist_msg.target = msg.target
        persist_msg.type = "EnergyStatus"
        persist_msg.timestamp = self.now() - (self.time_ref // 1000000)
        persist_msg.content = msg.content
        
        self.persist_pub.publish(persist_msg)
    
    def receive_adapt(self, msg):
        """Process an adaptation command message"""
        persist_msg = Persist()
        persist_msg.source = msg.source
        persist_msg.target = msg.target
        persist_msg.type = "Adaptation"
        persist_msg.timestamp = self.now() - (self.time_ref // 1000000)
        persist_msg.content = msg.action
        
        self.persist_pub.publish(persist_msg)
    
    def receive_uncertainty(self, msg):
        """Process an uncertainty message"""
        persist_msg = Persist()
        persist_msg.source = msg.source
        persist_msg.target = msg.target
        persist_msg.type = "Uncertainty"
        persist_msg.timestamp = self.now() - (self.time_ref // 1000000)
        persist_msg.content = msg.content
        
        self.persist_pub.publish(persist_msg)

def main(args=None):
    rclpy.init(args=args)

    logger = Logger()

    # Run spin in a thread, make thread daemon so we don't have to join it to exit
    thread = threading.Thread(
        target=rclpy.spin, args=(logger,), daemon=True
    )
    thread.start()
    rate = logger.create_rate(logger.frequency)  # 2 Hz

    try:
        while rclpy.ok():
            rate.sleep()
    finally:
        logger.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()