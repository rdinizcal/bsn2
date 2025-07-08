class DataFusionEngine:
    """Implements data fusion algorithm for sensor data"""
    
    def __init__(self, node):
        self.node = node
    
    def fuse_data(self):
        """Calculate patient risk status using fusion algorithm"""
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
        """Calculate weighted risk based on deviations"""
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