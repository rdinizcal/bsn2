"""
Data fusion algorithms for central hub emergency detection.

This module implements sophisticated data fusion algorithms that combine
sensor readings from multiple sources to calculate overall patient risk
status using weighted averaging and deviation-based calculations.
"""


class DataFusionEngine:
    """
    Implements data fusion algorithm for sensor data integration.
    
    This class combines data from multiple sensors to calculate a unified
    patient risk status. It uses weighted averaging based on data deviations
    and special handling for blood pressure readings to provide accurate
    emergency detection capabilities.
    
    Attributes:
        node: Reference to the parent central hub node.
        
    Examples:
        ```python
        fusion_engine = DataFusionEngine(central_hub_node)
        
        # Calculate patient risk status
        risk_status = fusion_engine.fuse_data()
        print(f"Patient risk: {risk_status:.1f}%")
        ```
    """
    
    def __init__(self, node):
        """
        Initialize data fusion engine.
        
        Sets up the fusion engine with reference to the parent node
        for accessing sensor data and system status.
        
        Args:
            node: The parent central hub node instance.
        """
        self.node = node
    
    def fuse_data(self):
        """
        Calculate patient risk status using advanced fusion algorithm.
        
        Implements a sophisticated data fusion algorithm that:
        1. Processes risk values from all available sensors
        2. Applies special handling for blood pressure data
        3. Calculates weighted averages based on data deviations
        4. Returns a unified patient risk percentage
        
        The algorithm handles missing data gracefully and provides
        meaningful results even with partial sensor availability.
        
        Returns:
            float: Patient risk status as percentage (0-100), or 0.0
                   if no valid sensor data is available or hub is inactive.
        """
        # Skip if hub not active
        if not self.node.active:
            return 0.0
            
        self.node.publisher_manager.publish_status("activated", "calculate")
        
        # Use battery for computation
        data_count = sum(1 for risk in self.node.sensor_handler.latest_risk.values() if risk >= 0)
        self.node.battery_manager.consume(data_count * 0.001)
        
        # Original data fusion algorithm implementation
        sensor_types = [
            "thermometer", "ecg", "oximeter", "abps", "abpd", "glucosemeter",
        ]

        # Get risk values in order
        packets_received = []
        for sensor_type in sensor_types:
            packets_received.append(self.node.sensor_handler.latest_risk.get(sensor_type, -1.0))

        # Calculation variables
        average = 0.0
        count = 0
        index = 0
        bpr_avg = 0.0
        values = []

        # Process each sensor's risk value
        for risk in packets_received:
            if risk >= 0:
                # Special handling for blood pressure
                if index == 3 or index == 4:  # abps or abpd
                    bpr_avg += risk
                else:
                    average += risk
                    values.append(risk)
                count += 1

            # Process blood pressure after both readings
            if index == 4 and bpr_avg >= 0.0:
                bpr_avg /= 2  # Average systolic and diastolic
                average += bpr_avg
                values.append(bpr_avg)

            index += 1

        # Calculate final risk
        if count == 0:
            self.node.publisher_manager.publish_status("activated", "idle")
            return 0.0
            
        avg = average / count
        
        # Calculate weighted risk using deviations
        risk_status = self._calculate_weighted_risk(values, avg)
        
        self.node.publisher_manager.publish_status("activated", "idle")
        return risk_status
        
    def _calculate_weighted_risk(self, values, avg):
        """
        Calculate weighted risk based on data deviations.
        
        Implements a sophisticated weighting algorithm that considers
        how much each sensor reading deviates from the average. Sensors
        with higher deviations receive more weight in the final calculation.
        
        Args:
            values (list): List of valid sensor risk values.
            avg (float): Simple average of all sensor values.
            
        Returns:
            float: Weighted average risk value, or simple average
                   if all values are identical.
        """
        if not values:
            return 0.0
            
        # Calculate deviations
        deviations = []
        min_dev = float("inf")
        max_dev = -float("inf")

        for value in values:
            dev = value - avg
            deviations.append(dev)
            
            if dev > max_dev:
                max_dev = dev
            if dev < min_dev:
                min_dev = dev

        # If all values are the same
        if max_dev - min_dev <= 0.0:
            return avg

        # Calculate weighted average based on deviations
        weighted_average = 0.0
        weight_sum = 0.0
        
        for i in range(len(values)):
            norm_dev = (deviations[i] - min_dev) / (max_dev - min_dev)
            weight_sum += norm_dev
            weighted_average += values[i] * norm_dev

        # Return final weighted risk
        if weight_sum > 0:
            return weighted_average / weight_sum
        else:
            return avg