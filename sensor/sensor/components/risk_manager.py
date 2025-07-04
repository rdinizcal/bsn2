"""
Risk evaluation and management for sensor nodes.

This module provides risk assessment capabilities for sensor readings,
including configurable risk ranges and percentage-based risk scoring
tailored to different sensor types and vital signs.
"""

from sensor.utils.risk_evaluator import RiskEvaluator


class RiskManager:
    """
    Manages risk evaluation for sensor data.
    
    This class handles the configuration and execution of risk assessment
    for sensor readings. It uses sensor-specific risk ranges to calculate
    numerical risk percentages and corresponding risk level labels.
    
    Attributes:
        node: Reference to the parent sensor node.
        evaluator (RiskEvaluator): Risk evaluation engine.
        ranges (dict): Sensor-specific risk value ranges.
        risk_percentages (tuple): Risk percentage ranges for scoring.
        
    Examples:
        ```python
        risk_mgr = RiskManager(sensor_node)
        risk_mgr.configure_risk_ranges()
        
        # Evaluate risk for a sensor reading
        risk_value = risk_mgr.evaluate_risk(37.5)  # Returns percentage
        risk_label = risk_mgr.get_risk_label(risk_value)  # Returns "low"
        ```
    """
    
    def __init__(self, node):
        """
        Initialize risk manager.
        
        Sets up the risk evaluator and prepares for configuration
        with sensor-specific parameters.
        
        Args:
            node: The parent sensor node instance.
        """
        self.node = node
        self.evaluator = RiskEvaluator()
        self.ranges = {}
        self.risk_percentages = None
    
    def configure_risk_ranges(self):
        """
        Set up risk ranges based on sensor type.
        
        Parses sensor-specific configuration parameters to establish
        risk value ranges and percentage scoring ranges. Configures
        the risk evaluator with these parameters.
        """
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
        """
        Parse a range string like '0,20' into a tuple (0.0, 20.0).
        
        Handles both string and parameter object inputs, providing
        robust parsing for configuration values.
        
        Args:
            range_str: Range specification as string or parameter object.
            
        Returns:
            tuple: Parsed range as (min_value, max_value), or (-1.0, -1.0)
                   if parsing fails.
        """
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
        """
        Calculate risk percentage for the given datapoint.
        
        Uses the configured risk evaluator to assess the risk level
        of a sensor reading based on sensor-specific ranges.
        
        Args:
            datapoint (float): Sensor reading to evaluate.
            
        Returns:
            float: Risk percentage (0-100), or -1.0 if evaluation fails.
        """
        if datapoint < 0:
            return -1.0
            
        sensor_type = self.node.config.sensor
        return self.evaluator.evaluate_risk(sensor_type, datapoint)
    
    def get_risk_label(self, risk_value):
        """
        Convert numerical risk to a label.
        
        Transforms a numerical risk percentage into a human-readable
        risk level label for communication and display purposes.
        
        Args:
            risk_value (float): Risk percentage value.
            
        Returns:
            str: Risk level label ("low", "moderate", "high", "unknown").
        """
        if risk_value < 0:
            return "unknown"
            
        return self.evaluator.risk_label(risk_value)
    
    def is_high_risk(self, risk_value):
        """
        Check if risk value indicates high risk.
        
        Args:
            risk_value (float): Risk percentage to check.
            
        Returns:
            bool: True if risk is in high range, False otherwise.
        """
        return self.evaluator.is_high_risk(risk_value)
    
    def is_medium_risk(self, risk_value):
        """
        Check if risk value indicates medium risk.
        
        Args:
            risk_value (float): Risk percentage to check.
            
        Returns:
            bool: True if risk is in medium range, False otherwise.
        """
        return self.evaluator.is_medium_risk(risk_value)
    
    def is_low_risk(self, risk_value):
        """
        Check if risk value indicates low risk.
        
        Args:
            risk_value (float): Risk percentage to check.
            
        Returns:
            bool: True if risk is in low range, False otherwise.
        """
        return self.evaluator.is_low_risk(risk_value)