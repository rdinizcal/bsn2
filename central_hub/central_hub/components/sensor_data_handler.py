"""
Sensor data reception and management for central hub.

This module handles receiving sensor data from multiple sensors, storing
the latest readings, and managing sensor battery level information for
the emergency detection system.
"""

from bsn_interfaces.msg import SensorData


class SensorDataHandler:
    """
    Handles sensor data reception and storage for the central hub.
    
    This class manages subscriptions to multiple sensor data topics, stores
    the latest sensor readings and risk assessments, and tracks battery levels
    of individual sensors for system monitoring.
    
    Attributes:
        node: Reference to the parent central hub node.
        NOT_USED (str): Constant for unused sensor data.
        latest_risk (dict): Latest risk percentages from each sensor.
        sensor_battery_levels (dict): Current battery levels of each sensor.
        latest_data (dict): Latest sensor readings from each sensor.
        latest_risks_labels (dict): Latest risk level labels from each sensor.
        
    Examples:
        ```python
        handler = SensorDataHandler(central_hub_node)
        handler.setup_subscriptions()
        
        # Access latest data
        temp = handler.latest_data["thermometer"]
        risk = handler.latest_risk["thermometer"]
        ```
    """
    
    def __init__(self, node):
        """
        Initialize sensor data handler.
        
        Sets up data structures for tracking sensor readings, risk levels,
        and battery status for all supported sensor types.
        
        Args:
            node: The parent central hub node instance.
        """
        self.node = node
        
        # Data structures for tracking sensors
        self.NOT_USED = ""
        self.latest_risk = {
            "abpd": 0.0, "abps": 0.0, "ecg": 0.0,
            "glucosemeter": 0.0, "oximeter": 0.0, "thermometer": 0.0,
        }
        
        self.sensor_battery_levels = {
            "abpd": 100.0, "abps": 100.0, "ecg": 100.0,
            "glucosemeter": 100.0, "oximeter": 100.0, "thermometer": 100.0,
        }
        
        self.latest_data = {
            "abpd": -1.0, "abps": -1.0, "ecg": -1.0,
            "glucosemeter": -1.0, "oximeter": -1.0, "thermometer": -1.0,
        }

        self.latest_risks_labels = {
            "abpd": "unknown", "abps": "unknown", "ecg": "unknown",
            "glucosemeter": "unknown", "oximeter": "unknown", "thermometer": "unknown",
        }
    
    def setup_subscriptions(self):
        """
        Set up subscriptions to sensor data topics.
        
        Creates ROS subscriptions for all supported sensor types including
        blood pressure (systolic/diastolic), ECG, glucose, oximeter, and
        thermometer sensors.
        """
        self.sub_abpd = self.node.create_subscription(
            SensorData, "sensor_data/abpd", self.receive_datapoint, 10
        )
        self.sub_abps = self.node.create_subscription(
            SensorData, "sensor_data/abps", self.receive_datapoint, 10
        )
        self.sub_ecg = self.node.create_subscription(
            SensorData, "sensor_data/ecg", self.receive_datapoint, 10
        )
        self.sub_glucosemeter = self.node.create_subscription(
            SensorData, "sensor_data/glucosemeter", self.receive_datapoint, 10
        )
        self.sub_oximeter = self.node.create_subscription(
            SensorData, "sensor_data/oximeter", self.receive_datapoint, 10
        )
        self.sub_thermometer = self.node.create_subscription(
            SensorData, "sensor_data/thermometer", self.receive_datapoint, 10
        )
        
    def cleanup_subscriptions(self):
        """
        Clean up sensor data subscriptions.
        
        Sets all subscription references to None, allowing them to be
        garbage collected during lifecycle cleanup operations.
        """
        self.sub_abpd = None
        self.sub_abps = None
        self.sub_ecg = None
        self.sub_glucosemeter = None
        self.sub_oximeter = None
        self.sub_thermometer = None
    
    def receive_datapoint(self, msg):
        """
        Receive and process sensor data messages.
        
        Processes incoming sensor data including sensor readings, risk
        assessments, and battery levels. Updates internal data structures
        and publishes status updates.
        
        Args:
            msg (SensorData): Incoming sensor data message containing
                sensor type, reading value, risk assessment, and battery level.
        """
        # Skip if not active
        if not self.node.active:
            return
            
        self.node.publisher_manager.publish_status(
            self.node.active and "activated" or "deactivated", "receive"
        )
            
        if msg.sensor_type == "":
            self.node.get_logger().debug(
                f"null value received: {msg.sensor_type} with {msg.sensor_datapoint}"
            )
        else:
            # Update the latest data and risk for the corresponding sensor type
            self.latest_data[msg.sensor_type] = msg.sensor_datapoint
            self.latest_risks_labels[msg.sensor_type] = msg.risk_level
            self.latest_risk[msg.sensor_type] = msg.risk
            
            # Capture battery level from sensor
            self.sensor_battery_levels[msg.sensor_type] = getattr(msg, 'battery_level', 100.0)

            # Use a small amount of battery to receive data 
            self.node.battery_manager.consume(0.001)

            self.node.get_logger().info(
                f"Received data from {msg.sensor_type}: {msg.sensor_datapoint} with risk: {msg.risk_level}"
            )
            
        self.node.publisher_manager.publish_status(
            self.node.active and "activated" or "deactivated", "idle"
        )