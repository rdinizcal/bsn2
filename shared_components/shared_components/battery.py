# battery.py
class Battery:
    """ROS2 port of the BSN Battery class for energy simulation"""
    
    def __init__(self, id_name="sensor_battery", capacity=100.0, current_level=100.0, unit=1.0):
        """
        Initialize a battery with specific parameters
        
        Args:
            id_name (str): Battery identifier
            capacity (float): Maximum capacity (typically 100%)
            current_level (float): Current battery level (should be <= capacity)
            unit (float): Consumption/generation unit per operation
        """
        if capacity <= 0:
            raise ValueError("Capacity should not be negative or null")
        
        if current_level < 0 or current_level > capacity:
            raise ValueError("Current level should not be negative or null nor bigger than the capacity")
            
        if unit < 0 or unit > capacity:
            raise ValueError("The resolution should not be negative or null nor bigger than the capacity")
            
        self.id = id_name
        self.capacity = capacity
        self.current_level = current_level
        self.unit = unit
    
    def consume(self, multiplier=1.0):
        """
        Consume battery based on unit and multiplier
        
        Args:
            multiplier (float): Multiplier for consumption unit
        """
        self.current_level -= self.unit * multiplier
        if self.current_level < 0:
            self.current_level = 0
    
    def generate(self, multiplier=1.0):
        """
        Generate/recharge battery based on unit and multiplier
        
        Args:
            multiplier (float): Multiplier for generation unit
        """
        self.current_level += self.unit * multiplier
        if self.current_level > self.capacity:
            self.current_level = self.capacity
        
    def __str__(self):
        """String representation of battery"""
        return f"Battery: {self.id} {self.capacity} {self.current_level} {self.unit}"