from bsn_interfaces.msg import Event, Status, TargetSystemData
from std_msgs.msg import Header

class PublisherManager:
    """Manages all publishers for the central hub"""
    
    def __init__(self, node):
        self.node = node
        self.status_pub = None
        
        # Create event publisher early
        self.event_pub = node.create_publisher(
            Event, 'collect_event', 10
        )
        
        # Target system data publisher will be created on configure
        self.target_system_publisher = None
    
    def setup_publishers(self):
        """Set up publishers during configure transition"""
        self.status_pub = self.node.create_publisher(
            Status, 'component_status', 10
        )
        
        self.target_system_publisher = self.node.create_publisher(
            TargetSystemData, "target_system_data", 10
        )
    
    def publish_status(self, content, task):
        """Publish component status"""
        if self.status_pub is None:
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
        msg.freq = self.node.config.frequency
        self.event_pub.publish(msg)
        self.node.get_logger().info(f"Event published: {event_type}")
    
    def publish_heartbeat(self):
        """Publish periodic heartbeat"""
        msg = Event()
        msg.source = self.node.get_name()
        msg.target = "system"
        msg.freq = float(self.node.config.frequency)

        # Check for recharge mode
        if self.node.battery_manager.is_recharging:
            msg.content = "recharging"
        else:
            msg.content = "activate" if self.node.active else "deactivate"

        self.event_pub.publish(msg)
        self.node.get_logger().debug(f"Heartbeat published: {msg.content}")
    
    def publish_system_data(self, patient_status, latest_data, latest_risk, sensor_battery_levels):
        """Publish data to target system"""
        if self.target_system_publisher is None:
            return
            
        # Use small battery for transmission
        self.node.battery_manager.consume(0.001)
        
        msg = TargetSystemData()
        msg.header = Header()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "central_hub"
        
        # Add sensor data and risk values
        msg.trm_data = latest_data.get("thermometer", -1.0)
        msg.ecg_data = latest_data.get("ecg", -1.0)
        msg.oxi_data = latest_data.get("oximeter", -1.0)
        msg.abps_data = latest_data.get("abps", -1.0)
        msg.abpd_data = latest_data.get("abpd", -1.0)
        msg.glc_data = latest_data.get("glucosemeter", -1.0)
        
        msg.trm_risk = latest_risk.get("thermometer", 0.0)
        msg.ecg_risk = latest_risk.get("ecg", 0.0)
        msg.oxi_risk = latest_risk.get("oximeter", 0.0)
        msg.abps_risk = latest_risk.get("abps", 0.0)
        msg.abpd_risk = latest_risk.get("abpd", 0.0)
        msg.glc_risk = latest_risk.get("glucosemeter", 0.0)
        
        msg.trm_batt = sensor_battery_levels.get("thermometer", 100.0)
        msg.ecg_batt = sensor_battery_levels.get("ecg", 100.0)
        msg.oxi_batt = sensor_battery_levels.get("oximeter", 100.0)
        msg.abps_batt = sensor_battery_levels.get("abps", 100.0)
        msg.abpd_batt = sensor_battery_levels.get("abpd", 100.0)
        msg.glc_batt = sensor_battery_levels.get("glucosemeter", 100.0)
        
        msg.patient_status = patient_status
        
        self.target_system_publisher.publish(msg)
        self.node.get_logger().info("Published TargetSystemData")