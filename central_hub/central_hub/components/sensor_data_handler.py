from bsn_interfaces.msg import SensorData

class SensorDataHandler:
    """Handles sensor data reception and storage"""
    
    def __init__(self, node):
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
        """Set up subscriptions to sensor data"""
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
        """Clean up subscriptions"""
        self.sub_abpd = None
        self.sub_abps = None
        self.sub_ecg = None
        self.sub_glucosemeter = None
        self.sub_oximeter = None
        self.sub_thermometer = None
    
    def receive_datapoint(self, msg):
        """Receive data from sensors"""
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