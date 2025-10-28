"""
Publisher management for sensor nodes.

This module manages all ROS publishers for sensor data, status updates,
events, and heartbeat messages. It provides a centralized interface
for publishing various types of messages.
"""

from bsn_interfaces.msg import SensorData
from std_msgs.msg import Header
from shared_components.core.publisher_manager_base import PublisherManagerBase

class PublisherManager(PublisherManagerBase):
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
        super().__init__(node)
        # Data publisher will be created on configure
        self.data_pub = None
    
    def setup_exclusive_publishers(self) -> bool:
        """
        Set up any exclusive publishers for the sensor node.
        
        This method can be overridden to create additional publishers
        that are specific to certain sensor types or configurations.
        """
        try:
            self.data_pub = self.node.create_publisher(
                SensorData, f'sensor_data/{self.node.config.component}', 10
            )
            return True
        except Exception as e:
            self.node.get_logger().error(f"Failed to create data publisher: {e}")
            return False
    
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
        header.frame_id = self.node.config.component

        msg.header = header
        msg.sensor_type = self.node.config.component
        msg.sensor_datapoint = datapoint
        msg.battery_level = self.node.battery_manager.battery.current_level
        msg.risk = float(risk_value)
        msg.risk_level = risk_level
        
        self.data_pub.publish(msg)
        self.node.get_logger().info(
            f"++Transfer++\n Value: {msg.sensor_datapoint}, Risk: {msg.risk:.2f}%, "
            f"Level: {msg.risk_level}, Battery: {msg.battery_level:.1f}%"
        )