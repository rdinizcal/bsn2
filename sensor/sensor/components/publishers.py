from bsn_interfaces.msg import SensorData, EnergyStatus, Event, Status
from std_msgs.msg import Header

class PublisherManager:
    """Manages publishers for the sensor node"""
    
    def __init__(self, node):
        self.node = node
        self.status_pub = None
        
        # Create event publisher early
        self.event_pub = node.create_publisher(Event, 'collect_event', 10)
        
        # Data publisher will be created on configure
        self.data_pub = None
    
    def setup_publishers(self):
        """Set up publishers during configure transition"""
        # Create status publisher
        self.status_pub = self.node.create_publisher(
            Status, 'component_status', 10
        )
        
        # Create sensor data publisher
        self.data_pub = self.node.create_publisher(
            SensorData, f'sensor_data/{self.node.config.sensor}', 10
        )
    
    def publish_status(self, content, task):
        """Publish component status"""
        if self.status_pub is None:
            self.node.get_logger().debug("Status publisher not available, skipping publish")
            return
            
        msg = Status()
        msg.source = self.node.get_name()
        msg.target = "system"
        msg.content = content
        msg.task = task
        self.status_pub.publish(msg)
        self.node.get_logger().debug(f"Status published: {content}, task: {task}")
    
    def publish_event(self, event_type):
        """Publish an event"""
        msg = Event()
        msg.source = self.node.get_name()
        msg.target = "system"
        msg.content = event_type
        msg.freq = float(self.node.config.frequency)
        self.event_pub.publish(msg)
        self.node.get_logger().info(f"Event published: {event_type}")
    
    def publish_heartbeat(self):
        """Publish periodic heartbeat"""
        msg = Event()
        msg.source = self.node.get_name()
        msg.target = "system"
        msg.freq = float(self.node.config.frequency)
        
        # Check for recharge mode
        if hasattr(self.node, 'battery_manager') and hasattr(self.node.battery_manager, 'is_recharging'):
            if self.node.battery_manager.is_recharging:
                msg.content = "recharging"
            else:
                msg.content = "activate" if self.node.active else "deactivate"
        else:
            msg.content = "activate" if self.node.active else "deactivate"
            
        self.event_pub.publish(msg)
        self.node.get_logger().debug(f"Heartbeat published: {msg.content}")
    
    def publish_sensor_data(self, datapoint, risk_value, risk_level):
        """Publish sensor data"""
        if self.data_pub is None:
            return
            
        msg = SensorData()
        header = Header()
        header.stamp = self.node.get_clock().now().to_msg()
        header.frame_id = self.node.config.sensor
        
        msg.header = header
        msg.sensor_type = self.node.config.sensor
        msg.sensor_datapoint = datapoint
        msg.battery_level = self.node.battery_manager.battery.current_level
        msg.risk = float(risk_value)
        msg.risk_level = risk_level
        
        self.data_pub.publish(msg)
        self.node.get_logger().info(
            f"++Transfer++\n Value: {msg.sensor_datapoint}, Risk: {msg.risk:.2f}%, "
            f"Level: {msg.risk_level}, Battery: {msg.battery_level:.1f}%"
        )