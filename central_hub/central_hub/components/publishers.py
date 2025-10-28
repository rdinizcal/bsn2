"""
Publisher management for central hub node.

This module manages all ROS publishers for the central hub including
system data publishing, status updates, events, and heartbeat messages
for the Body Sensor Network emergency detection system.
"""

from bsn_interfaces.msg import TargetSystemData
from std_msgs.msg import Header
from shared_components.enums import StatusContent, Task
from shared_components.core.publisher_manager_base import PublisherManagerBase


class PublisherManager(PublisherManagerBase):
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
        super().__init__(node)
        # Create event publisher early
        self.target_system_publisher = None
    
    def setup_exclusive_publishers(self) -> bool:
        """
        Set up publishers during configure transition.
        
        Creates the remaining publishers that require hub-specific
        configuration information including status and target system
        data publishers.
        """
        try:
            # Create status publisher
            self.target_system_publisher = self.node.create_publisher(
                TargetSystemData, "target_system_data", 10
            )
            return True
        except Exception as e:
            self.node.get_logger().error(f"Failed to create exclusive publishers: {e}")
            return False
    
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
        if self.target_system_publisher is None or not self.node.active:
            self.node.get_logger().debug("Target system publisher not available or node inactive, skipping publish")
            self.publish_status(StatusContent.FAIL, Task.TRANSFER)
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