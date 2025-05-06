class RiskEvaluator:
    def __init__(self):
        # Risk ranges percentages
        self.low_percentage = (0.0, 20.0)
        self.mid_percentage = (21.0, 65.0)
        self.high_percentage = (66.0, 100.0)
        
        self.sensor_ranges = {}
        
    def configure(self, sensor_type, sensor_ranges, risk_percentages=None):
        """Configure the risk evaluator with sensor-specific ranges"""
        self.sensor_ranges[sensor_type] = sensor_ranges
        
        # Update percentage ranges if provided
        if risk_percentages:
            self.low_percentage = risk_percentages[0]
            self.mid_percentage = risk_percentages[1]
            self.high_percentage = risk_percentages[2]

    def get_displacement(self, range_min, range_max, value, logic="crescent"):
        """Calculate how far a value is from the boundaries of its range"""
        if logic == "decrescent":
            result = abs(value - range_max)
            result /= (range_max - range_min)
        elif logic == "crescent":
            result = value - range_min
            result /= (range_max - range_min)
        elif logic == "medium":
            medium_value = (range_max + range_min) / 2.0
            result = abs(value - medium_value)
            result /= (range_max - medium_value)
        else:
            raise ValueError("Invalid logic parameter")
        
        return result

    def convert_percentage(self, perc_min, perc_max, displacement):
        """Convert a 0-1 displacement to a percentage in the given range"""
        return perc_min + displacement * (perc_max - perc_min)

    def evaluate_risk(self, sensor_type, value):
        """Calculate risk percentage (0-100) for a given sensor value"""
        if sensor_type not in self.sensor_ranges:
            return -1.0
            
        ranges = self.sensor_ranges[sensor_type]
        
        # Check which range the value falls into
        if self.in_range(ranges["low_risk"], value):
            # Low risk range - medium displacement from center is lowest risk
            disp = self.get_displacement(ranges["low_risk"][0], ranges["low_risk"][1], value, "medium")
            return self.convert_percentage(self.low_percentage[0], self.low_percentage[1], disp)
            
        elif self.in_range(ranges["mid_risk0"], value):
            # Medium risk (low side) - decrescent from boundary to low
            disp = self.get_displacement(ranges["mid_risk0"][0], ranges["mid_risk0"][1], value, "decrescent")
            return self.convert_percentage(self.mid_percentage[0], self.mid_percentage[1], disp)
            
        elif self.in_range(ranges["mid_risk1"], value):
            # Medium risk (high side) - crescent from boundary to high
            disp = self.get_displacement(ranges["mid_risk1"][0], ranges["mid_risk1"][1], value, "crescent")
            return self.convert_percentage(self.mid_percentage[0], self.mid_percentage[1], disp)
            
        elif self.in_range(ranges["high_risk0"], value):
            # High risk (low side) - decrescent from boundary to low
            disp = self.get_displacement(ranges["high_risk0"][0], ranges["high_risk0"][1], value, "decrescent")
            return self.convert_percentage(self.high_percentage[0], self.high_percentage[1], disp)
            
        elif self.in_range(ranges["high_risk1"], value):
            # High risk (high side) - crescent from boundary to high
            disp = self.get_displacement(ranges["high_risk1"][0], ranges["high_risk1"][1], value, "crescent")
            return self.convert_percentage(self.high_percentage[0], self.high_percentage[1], disp)
            
        return -1.0  # Value is not in any range

    def in_range(self, range_tuple, value):
        """Check if a value is within a range and the range is valid"""
        min_val, max_val = range_tuple
        return (min_val != -1.0 or max_val != -1.0) and (min_val <= value <= max_val)
        
    def is_low_risk(self, risk_value):
        """Check if a risk value is in the low risk percentage range"""
        return self.low_percentage[0] <= risk_value <= self.low_percentage[1]
        
    def is_medium_risk(self, risk_value):
        """Check if a risk value is in the medium risk percentage range"""
        return self.mid_percentage[0] <= risk_value <= self.mid_percentage[1]
        
    def is_high_risk(self, risk_value):
        """Check if a risk value is in the high risk percentage range"""
        return self.high_percentage[0] <= risk_value <= self.high_percentage[1]
        
    def risk_label(self, risk_value):
        """Convert a numeric risk value to a label"""
        if self.is_low_risk(risk_value):
            return "low"
        elif self.is_medium_risk(risk_value):
            return "moderate"
        elif self.is_high_risk(risk_value):
            return "high"
        else:
            return "unknown"