"""
Risk evaluation utility for sensor data assessment.

This module provides the core risk evaluation algorithms for assessing
sensor readings against configurable risk ranges and calculating
percentage-based risk scores.
"""


class RiskEvaluator:
    """
    Risk evaluation engine for sensor data.
    
    This class implements the mathematical algorithms for evaluating
    sensor readings against configured risk ranges and converting them
    to percentage-based risk scores with corresponding labels.
    
    Attributes:
        low_percentage (tuple): Low risk percentage range (min, max).
        mid_percentage (tuple): Medium risk percentage range (min, max).
        high_percentage (tuple): High risk percentage range (min, max).
        sensor_ranges (dict): Sensor-specific risk value ranges.
        
    Examples:
        ```python
        evaluator = RiskEvaluator()
        
        # Configure for thermometer
        ranges = {
            "low_risk": (36.0, 37.5),
            "mid_risk1": (37.5, 39.0),
            "high_risk1": (39.0, 42.0)
        }
        evaluator.configure("thermometer", ranges)
        
        # Evaluate risk
        risk = evaluator.evaluate_risk("thermometer", 38.5)
        label = evaluator.risk_label(risk)
        ```
    """
    
    def __init__(self):
        """
        Initialize risk evaluator.
        
        Sets up default risk percentage ranges and initializes
        the sensor ranges dictionary.
        """
        # Risk ranges percentages
        self.low_percentage = (0.0, 20.0)
        self.mid_percentage = (21.0, 65.0)
        self.high_percentage = (66.0, 100.0)

        self.sensor_ranges = {}

    def configure(self, sensor_type, sensor_ranges, risk_percentages=None):
        """
        Configure the risk evaluator with sensor-specific ranges.
        
        Sets up risk evaluation parameters for a specific sensor type
        including value ranges and optional custom percentage ranges.
        
        Args:
            sensor_type (str): Type of sensor (e.g., "thermometer").
            sensor_ranges (dict): Dictionary of risk value ranges.
            risk_percentages (list, optional): Custom percentage ranges
                as [(low_min, low_max), (mid_min, mid_max), (high_min, high_max)].
        """
        self.sensor_ranges[sensor_type] = sensor_ranges

        # Update percentage ranges if provided
        if risk_percentages:
            self.low_percentage = risk_percentages[0]
            self.mid_percentage = risk_percentages[1]
            self.high_percentage = risk_percentages[2]

    def get_displacement(self, range_min, range_max, value, logic="crescent"):
        """
        Calculate how far a value is from the boundaries of its range.
        
        Computes a normalized displacement (0-1) based on the position
        of the value within the range and the specified logic.
        
        Args:
            range_min (float): Minimum value of the range.
            range_max (float): Maximum value of the range.
            value (float): Value to evaluate within the range.
            logic (str): Logic type - "crescent", "decrescent", or "medium".
            
        Returns:
            float: Normalized displacement value (0.0-1.0).
            
        Raises:
            ValueError: If logic parameter is invalid.
        """
        if logic == "decrescent":
            result = abs(value - range_max)
            result /= range_max - range_min
        elif logic == "crescent":
            result = value - range_min
            result /= range_max - range_min
        elif logic == "medium":
            medium_value = (range_max + range_min) / 2.0
            result = abs(value - medium_value)
            result /= range_max - medium_value
        else:
            raise ValueError("Invalid logic parameter")

        return result

    def convert_percentage(self, perc_min, perc_max, displacement):
        """
        Convert a 0-1 displacement to a percentage in the given range.
        
        Maps a normalized displacement value to a percentage within
        the specified percentage range.
        
        Args:
            perc_min (float): Minimum percentage value.
            perc_max (float): Maximum percentage value.
            displacement (float): Normalized displacement (0-1).
            
        Returns:
            float: Percentage value within the specified range.
        """
        return perc_min + displacement * (perc_max - perc_min)

    def evaluate_risk(self, sensor_type, value):
        """
        Calculate risk percentage (0-100) for a given sensor value.
        
        Evaluates the risk level of a sensor reading by determining
        which risk range it falls into and calculating the appropriate
        percentage score based on displacement within that range.
        
        Args:
            sensor_type (str): Type of sensor being evaluated.
            value (float): Sensor reading value.
            
        Returns:
            float: Risk percentage (0-100), or -1.0 if sensor type
                   is unknown or value is out of range.
        """
        if sensor_type not in self.sensor_ranges:
            return -1.0

        ranges = self.sensor_ranges[sensor_type]

        # Check which range the value falls into
        if self.in_range(ranges["low_risk"], value):
            # Low risk range - medium displacement from center is lowest risk
            disp = self.get_displacement(
                ranges["low_risk"][0], ranges["low_risk"][1], value, "medium"
            )
            return self.convert_percentage(
                self.low_percentage[0], self.low_percentage[1], disp
            )

        elif self.in_range(ranges["mid_risk0"], value):
            # Medium risk (low side) - decrescent from boundary to low
            disp = self.get_displacement(
                ranges["mid_risk0"][0], ranges["mid_risk0"][1], value, "decrescent"
            )
            return self.convert_percentage(
                self.mid_percentage[0], self.mid_percentage[1], disp
            )

        elif self.in_range(ranges["mid_risk1"], value):
            # Medium risk (high side) - crescent from boundary to high
            disp = self.get_displacement(
                ranges["mid_risk1"][0], ranges["mid_risk1"][1], value, "crescent"
            )
            return self.convert_percentage(
                self.mid_percentage[0], self.mid_percentage[1], disp
            )

        elif self.in_range(ranges["high_risk0"], value):
            # High risk (low side) - decrescent from boundary to low
            disp = self.get_displacement(
                ranges["high_risk0"][0], ranges["high_risk0"][1], value, "decrescent"
            )
            return self.convert_percentage(
                self.high_percentage[0], self.high_percentage[1], disp
            )

        elif self.in_range(ranges["high_risk1"], value):
            # High risk (high side) - crescent from boundary to high
            disp = self.get_displacement(
                ranges["high_risk1"][0], ranges["high_risk1"][1], value, "crescent"
            )
            return self.convert_percentage(
                self.high_percentage[0], self.high_percentage[1], disp
            )

        return -1.0  # Value is not in any range

    def in_range(self, range_tuple, value):
        """
        Check if a value is within a range and the range is valid.
        
        Validates that the range is properly defined and contains
        the specified value.
        
        Args:
            range_tuple (tuple): Range as (min_value, max_value).
            value (float): Value to check.
            
        Returns:
            bool: True if value is within the valid range, False otherwise.
        """
        min_val, max_val = range_tuple
        return (min_val != -1.0 or max_val != -1.0) and (min_val <= value <= max_val)

    def is_low_risk(self, risk_value):
        """
        Check if a risk value is in the low risk percentage range.
        
        Args:
            risk_value (float): Risk percentage to check.
            
        Returns:
            bool: True if risk is in low range, False otherwise.
        """
        return self.low_percentage[0] <= risk_value <= self.low_percentage[1]

    def is_medium_risk(self, risk_value):
        """
        Check if a risk value is in the medium risk percentage range.
        
        Args:
            risk_value (float): Risk percentage to check.
            
        Returns:
            bool: True if risk is in medium range, False otherwise.
        """
        return self.mid_percentage[0] <= risk_value <= self.mid_percentage[1]

    def is_high_risk(self, risk_value):
        """
        Check if a risk value is in the high risk percentage range.
        
        Args:
            risk_value (float): Risk percentage to check.
            
        Returns:
            bool: True if risk is in high range, False otherwise.
        """
        return self.high_percentage[0] <= risk_value <= self.high_percentage[1]

    def risk_label(self, risk_value):
        """
        Convert a numeric risk value to a label.
        
        Provides human-readable risk level labels based on the
        percentage ranges configured for the evaluator.
        
        Args:
            risk_value (float): Risk percentage value.
            
        Returns:
            str: Risk level label ("low", "moderate", "high", "unknown").
        """
        if self.is_low_risk(risk_value):
            return "low"
        elif self.is_medium_risk(risk_value):
            return "moderate"
        elif self.is_high_risk(risk_value):
            return "high"
        else:
            return "unknown"
