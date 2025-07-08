from collections import deque

class DataProcessor:
    """Handles data collection, processing, and transmission"""
    
    def __init__(self, node):
        self.node = node
        
        # Initialize data window for moving average
        self.data_window = deque(maxlen=node.config.window_size)
        
        # Initialize client for patient data
        self.setup_client()
    
    def setup_client(self):
        """Set up client for patient data service"""
        from bsn_interfaces.srv import PatientData
        import time
        
        self.cli = self.node.create_client(PatientData, "/get_sensor_reading")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Service not available, waiting again...")
            
        self.req = PatientData.Request()
    def reset(self):
        """Reset data window and processing state"""
        self.data_window.clear()
        
    
    def collect(self):
        """Collect data from patient service"""
        self.node.publisher_manager.publish_status(
            self.node.active and "activated" or "deactivated", "collect"
        )
        
        # Check if active before collecting
        if not self.node.active:
            self.node.battery_manager.recharge()
            return -1.0
            
        # Consume battery for collection
        self.node.battery_manager.consume(1.0)
        
        # Make service call
        self.req.vital_sign = self.node.config.vital_sign
        response = self.cli.call(self.req)
        
        if response is None:
            self.node.get_logger().error("Service call failed. No response received.")
            self.node.publisher_manager.publish_status(
                self.node.active and "activated" or "deactivated", "idle"
            )
            return -1.0
            
        self.node.get_logger().debug(
            f"++Collect++\n new data from {self.req.vital_sign} collected: [{response.datapoint}]"
        )
        
        self.node.publisher_manager.publish_status(
            self.node.active and "activated" or "deactivated", "idle"
        )
        return response.datapoint
    
    def process(self, datapoint):
        """Process data with moving average"""
        if not self.node.active or datapoint < 0:
            return -1.0
            
        self.node.publisher_manager.publish_status(
            self.node.active and "activated" or "deactivated", "process"
        )
        
        # Consume battery for processing
        multiplier = min(1.0, self.node.config.window_size / 10.0)
        self.node.battery_manager.consume(multiplier)
        
        # Add to data window
        self.data_window.append(datapoint)
        
        # Calculate moving average
        if len(self.data_window) == self.node.config.window_size:
            moving_avg = sum(self.data_window) / self.node.config.window_size
            self.node.get_logger().debug(f"Moving average for {self.node.config.sensor}: {moving_avg}")
            result = moving_avg
        else:
            self.node.get_logger().info(
                f"Insufficient data for moving average. Current window size: {len(self.data_window)}"
            )
            result = -1.0
            
        self.node.publisher_manager.publish_status(
            self.node.active and "activated" or "deactivated", "idle"
        )
        return result
    
    def transfer(self, datapoint):
        """Transfer processed data"""
        if not self.node.active or datapoint < 0:
            return
            
        self.node.publisher_manager.publish_status(
            self.node.active and "activated" or "deactivated", "transfer"
        )
        
        # Consume battery for transfer
        self.node.battery_manager.consume(1.0)
        
        # Calculate risk
        risk_value = self.node.risk_manager.evaluate_risk(datapoint)
        risk_level = self.node.risk_manager.get_risk_label(risk_value)
        
        # Publish the data
        self.node.publisher_manager.publish_sensor_data(datapoint, risk_value, risk_level)
        
        # Send energy status
        self.node.battery_manager.send_energy_status()
        
        self.node.publisher_manager.publish_status(
            self.node.active and "activated" or "deactivated", "idle"
        )