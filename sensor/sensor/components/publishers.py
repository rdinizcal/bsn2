"""
Publisher management for sensor nodes.

This module manages all ROS publishers for sensor data, status updates,
events, and heartbeat messages. It provides a centralized interface
for publishing various types of messages.
"""

from bsn_interfaces.msg import SensorData, EnergyStatus, Event, Status
from std_msgs.msg import Header


class PublisherManager:
    """
    Manages publishers for the sensor node.
    
    This class centralizes all publishing operations for the sensor node,
    including sensor data, component status, system events, and heartbeat
    messages. It handles message formatting and ensures proper header
    information is included.
    
    Attributes:
        node: Reference to the parent sensor node.
        status_pub: Publisher for component status messages.
        event_pub: Publisher for system events.
        data_pub: Publisher for sensor data.
        
    Examples:
        ```python
        pub_mgr = PublisherManager(sensor_node)
        
        # Publish sensor data
        pub_mgr.publish_sensor_data(37.5, 15.2, "low")
        
        # Publish status update
        pub_mgr.publish_status("activated", "processing")
        
        # Publish system event
        pub_mgr.publish_event("data_collected")
        ```
    """
    
    def __init__(self, node):
        """
        Initialize publisher manager.
        
        Creates early publishers for events and prepares for additional
        publishers to be set up during configuration.
        
        Args:
            node: The parent sensor node instance.
        """
        self.node = node
        self.status_pub = None
        
        # Create event publisher early
        self.event_pub = node.create_publisher(Event, 'collect_event', 10)
        
        # Data publisher will be created on configure
        self.data_pub = None
    
    def setup_publishers(self):
        """
        Set up publishers during configure transition.
        
        Creates the remaining publishers that require sensor-specific
        configuration information.
        """
        # Create status publisher
        self.status_pub = self.node.create_publisher(
            Status, 'component_status', 10
        )
        
        # Create sensor data publisher
        self.data_pub = self.node.create_publisher(
            SensorData, f'sensor_data/{self.node.config.sensor}', 10
        )
    
    def publish_status(self, content, task):
        """
        Publish component status.
        
        Sends status updates about the sensor component's current state
        and active task to the system for monitoring.
        
        Args:
            content (str): Status content (e.g., "activated", "configured").
            task (str): Current task being performed (e.g., "idle", "collect").
        """
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
        """
        Publish an event.
        
        Sends system events to notify other components of important
        state changes or operations.
        
        Args:
            event_type (str): Type of event (e.g., "activate", "deactivate").
        """
        msg = Event()
        msg.source = self.node.get_name()
        msg.target = "system"
        msg.content = event_type
        msg.freq = float(self.node.config.frequency)
        self.event_pub.publish(msg)
        self.node.get_logger().info(f"Event published: {event_type}")
    
    def publish_heartbeat(self):
        """
        Publish periodic heartbeat.
        
        Sends regular heartbeat messages to indicate sensor node is alive
        and communicating. Content varies based on current operational state.
        """
        msg = Event()
        msg.source = self.node.get_name()
        msg.target = "system"
        msg.freq = float(self.node.config.frequency)
        
        # Check for recharge mode
        if (hasattr(self.node, 'battery_manager') and 
            hasattr(self.node.battery_manager, 'is_recharging')):
            if self.node.battery_manager.is_recharging:
                msg.content = "recharging"
            else:
                msg.content = "activate" if self.node.active else "deactivate"
        else:
            msg.content = "activate" if self.node.active else "deactivate"
            
        self.event_pub.publish(msg)
        self.node.get_logger().debug(f"Heartbeat published: {msg.content}")
    
    def publish_sensor_data(self, datapoint, risk_value, risk_level):
        """
        Publish sensor data.
        
        Publishes processed sensor readings with risk assessment and
        battery status information.
        
        Args:
            datapoint (float): Processed sensor measurement value.
            risk_value (float): Numerical risk percentage (0-100).
            risk_level (str): Risk level label ("low", "moderate", "high").
        """
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