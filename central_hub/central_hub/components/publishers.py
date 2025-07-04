"""
Publisher management for central hub node.

This module manages all ROS publishers for the central hub including
system data publishing, status updates, events, and heartbeat messages
for the Body Sensor Network emergency detection system.
"""

from bsn_interfaces.msg import Event, Status, TargetSystemData
from std_msgs.msg import Header


class PublisherManager:
    """
    Manages all publishers for the central hub.
    
    This class centralizes all publishing operations for the central hub,
    including target system data, component status, system events, and
    heartbeat messages. It handles message formatting and ensures proper
    header information for system communication.
    
    Attributes:
        node: Reference to the parent central hub node.
        status_pub: Publisher for component status messages.
        event_pub: Publisher for system events.
        target_system_publisher: Publisher for target system data.
        
    Examples:
        ```python
        pub_mgr = PublisherManager(central_hub_node)
        pub_mgr.setup_publishers()
        
        # Publish system data
        pub_mgr.publish_system_data(75.2, sensor_data, risk_data, battery_levels)
        
        # Publish status update
        pub_mgr.publish_status("activated", "processing")
        ```
    """
    
    def __init__(self, node):
        """
        Initialize publisher manager for central hub.
        
        Creates early publishers for events and prepares for additional
        publishers to be set up during configuration phase.
        
        Args:
            node: The parent central hub node instance.
        """
        self.node = node
        self.status_pub = None
        
        # Create event publisher early
        self.event_pub = node.create_publisher(
            Event, 'collect_event', 10
        )
        
        # Target system data publisher will be created on configure
        self.target_system_publisher = None
    
    def setup_publishers(self):
        """
        Set up publishers during configure transition.
        
        Creates the remaining publishers that require hub-specific
        configuration information including status and target system
        data publishers.
        """
        self.status_pub = self.node.create_publisher(
            Status, 'component_status', 10
        )
        
        self.target_system_publisher = self.node.create_publisher(
            TargetSystemData, "target_system_data", 10
        )
    
    def publish_status(self, content, task):
        """
        Publish component status for system monitoring.
        
        Sends status updates about the central hub's current state
        and active task to the system for monitoring and coordination.
        
        Args:
            content (str): Status content (e.g., "activated", "configured").
            task (str): Current task being performed (e.g., "idle", "calculate").
        """
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
        """
        Publish a system event for coordination.
        
        Sends system events to notify other components of important
        state changes or operations in the central hub.
        
        Args:
            event_type (str): Type of event (e.g., "activate", "recharge_start").
        """
        msg = Event()
        msg.source = self.node.get_name()
        msg.target = "system"
        msg.content = event_type
        msg.freq = self.node.config.frequency
        self.event_pub.publish(msg)
        self.node.get_logger().info(f"Event published: {event_type}")
    
    def publish_heartbeat(self):
        """
        Publish periodic heartbeat for system monitoring.
        
        Sends regular heartbeat messages to indicate the central hub is alive
        and communicating. Content varies based on current operational state
        including recharge mode detection.
        """
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
        """
        Publish comprehensive system data for emergency detection.
        
        Publishes integrated sensor data, risk assessments, and system status
        to external monitoring systems. This is the primary output of the
        emergency detection system containing all relevant patient monitoring
        information.
        
        Args:
            patient_status (float): Overall patient risk percentage (0-100).
            latest_data (dict): Latest sensor readings from all sensors.
            latest_risk (dict): Latest risk percentages from all sensors.
            sensor_battery_levels (dict): Current battery levels of all sensors.
        """
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