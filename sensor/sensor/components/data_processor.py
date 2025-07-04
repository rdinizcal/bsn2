"""
Data collection, processing, and transmission for sensor nodes.

This module handles the complete data pipeline from collecting raw sensor
readings through service calls to processing with moving averages and
transferring the final results.
"""

from collections import deque


class DataProcessor:
    """
    Handles data collection, processing, and transmission.
    
    This class manages the complete data processing pipeline including:
    - Collecting raw data from patient data service
    - Processing data using moving average filtering
    - Transferring processed data with risk evaluation
    
    Attributes:
        node: Reference to the parent sensor node.
        data_window (deque): Moving window for calculating averages.
        cli: ROS service client for patient data.
        req: Service request object.
        
    Examples:
        ```python
        processor = DataProcessor(sensor_node)
        
        # Collect raw data
        raw_data = processor.collect()
        
        # Process with moving average
        processed_data = processor.process(raw_data)
        
        # Transfer with risk evaluation
        processor.transfer(processed_data)
        ```
    """
    
    def __init__(self, node):
        """
        Initialize data processor.
        
        Sets up the data window for moving average calculation and
        initializes the service client for patient data collection.
        
        Args:
            node: The parent sensor node instance.
        """
        self.node = node
        
        # Initialize data window for moving average
        self.data_window = deque(maxlen=node.config.window_size)
        
        # Initialize client for patient data
        self.setup_client()
    
    def setup_client(self):
        """
        Set up client for patient data service.
        
        Creates a ROS service client and waits for the patient data service
        to become available before proceeding.
        """
        from bsn_interfaces.srv import PatientData
        import time
        
        self.cli = self.node.create_client(PatientData, "/get_sensor_reading")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Service not available, waiting again...")
            
        self.req = PatientData.Request()
        
    def reset(self):
        """
        Reset data window and processing state.
        
        Clears the moving average window, typically called during
        lifecycle cleanup operations.
        """
        self.data_window.clear()
    
    def collect(self):
        """
        Collect data from patient service.
        
        Makes a service call to get the latest sensor reading for the
        configured vital sign. Handles battery consumption and status
        reporting during the collection process.
        
        Returns:
            float: Collected sensor reading, or -1.0 if collection failed
                   or node is inactive.
        """
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
        """
        Process data with moving average.
        
        Adds the new datapoint to the moving window and calculates
        the average if the window is full. Handles battery consumption
        proportional to the window size.
        
        Args:
            datapoint (float): Raw sensor reading to process.
            
        Returns:
            float: Moving average of the data window, or -1.0 if window
                   is not full or node is inactive.
        """
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
            self.node.get_logger().debug(
                f"Moving average for {self.node.config.sensor}: {moving_avg}"
            )
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
        """
        Transfer processed data.
        
        Publishes the processed sensor data with risk evaluation and
        sends energy status updates. Handles battery consumption for
        the transfer operation.
        
        Args:
            datapoint (float): Processed sensor reading to transfer.
        """
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