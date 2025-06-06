from sensor.utils.risk_evaluator import RiskEvaluator

class RiskManager:
    """Manages risk evaluation for sensor data"""
    
    def __init__(self, node):
        """Initialize with reference to parent node"""
        self.node = node
        self.evaluator = RiskEvaluator()
        self.ranges = {}
        self.risk_percentages = None
    
    def configure_risk_ranges(self):
        """Set up risk ranges based on sensor type"""
        # Get sensor type
        sensor_type = self.node.config.sensor
        
        # Parse risk percentages
        low_risk = self._parse_range(self.node.config.lowrisk_percent)
        mid_risk = self._parse_range(self.node.config.midrisk_percent)
        high_risk = self._parse_range(self.node.config.highrisk_percent)
        
        self.risk_percentages = (
            (low_risk[0], low_risk[1]),
            (mid_risk[0], mid_risk[1]),
            (high_risk[0], high_risk[1])
        )
        
        # Parse sensor-specific ranges
        high_risk0 = self._parse_range(self.node.config.HighRisk0)
        mid_risk0 = self._parse_range(self.node.config.MidRisk0)
        low_risk = self._parse_range(self.node.config.LowRisk)
        mid_risk1 = self._parse_range(self.node.config.MidRisk1)
        high_risk1 = self._parse_range(self.node.config.HighRisk1)
        
        # Create range dictionary
        self.ranges = {
            "high_risk0": (high_risk0[0], high_risk0[1]),
            "mid_risk0": (mid_risk0[0], mid_risk0[1]),
            "low_risk": (low_risk[0], low_risk[1]),
            "mid_risk1": (mid_risk1[0], mid_risk1[1]),
            "high_risk1": (high_risk1[0], high_risk1[1])
        }
        
        # Configure evaluator with sensor ranges
        self.evaluator.configure(sensor_type, self.ranges, self.risk_percentages)
        
        self.node.get_logger().info(
            f"Risk ranges configured for {sensor_type}: "
            f"HR0={high_risk0}, MR0={mid_risk0}, LR={low_risk}, "
            f"MR1={mid_risk1}, HR1={high_risk1}"
        )
    
    def _parse_range(self, range_str):
        """Parse a range string like '0,20' into a tuple (0.0, 20.0)"""
        try:
            if isinstance(range_str, str):
                parts = range_str.split(',')
                return (float(parts[0]), float(parts[1]))
            else:
                # In case it's already processed or a parameter
                if hasattr(range_str, 'value'):
                    parts = range_str.value.split(',')
                    return (float(parts[0]), float(parts[1]))
                return (-1.0, -1.0)
        except (ValueError, IndexError):
            self.node.get_logger().warn(f"Invalid range format: {range_str}")
            return (-1.0, -1.0)
    
    def evaluate_risk(self, datapoint):
        """Calculate risk percentage for the given datapoint"""
        if datapoint < 0:
            return -1.0
            
        sensor_type = self.node.config.sensor
        return self.evaluator.evaluate_risk(sensor_type, datapoint)
    
    def get_risk_label(self, risk_value):
        """Convert numerical risk to a label"""
        if risk_value < 0:
            return "unknown"
            
        return self.evaluator.risk_label(risk_value)
    
    def is_high_risk(self, risk_value):
        """Check if risk value indicates high risk"""
        return self.evaluator.is_high_risk(risk_value)
    
    def is_medium_risk(self, risk_value):
        """Check if risk value indicates medium risk"""
        return self.evaluator.is_medium_risk(risk_value)
    
    def is_low_risk(self, risk_value):
        """Check if risk value indicates low risk"""
        return self.evaluator.is_low_risk(risk_value)