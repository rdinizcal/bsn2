"""
Configuration management for central hub node.

This module handles parameter declaration, validation, and access for central
hub configuration including battery settings, frequency parameters, and
adaptation settings.
"""


class ConfigManager:
    """
    Handles parameter declaration and access for Central Hub.
    
    This class manages all configuration parameters for the central hub including
    battery configuration, operating frequency, and system adaptation settings.
    It declares ROS parameters and provides easy access to configuration values.
    
    Attributes:
        node: Reference to the parent ROS node.
        battery_id (str): Unique identifier for the battery.
        battery_capacity (float): Maximum battery capacity.
        battery_level (float): Initial battery level.
        battery_unit (float): Battery consumption unit.
        instant_recharge (bool): Whether battery recharges instantly.
        frequency (float): Operating frequency for detection cycle.
        activate_adaptation (bool): Whether system adaptation is enabled.
        
    Examples:
        ```python
        config = ConfigManager(central_hub_node)
        print(f"Operating frequency: {config.frequency} Hz")
        print(f"Battery capacity: {config.battery_capacity}")
        ```
    """
    
    def __init__(self, node):
        """
        Initialize configuration manager for Central Hub.
        
        Declares all required ROS parameters and extracts their values
        for easy access throughout the central hub node.
        
        Args:
            node: The parent Central Hub ROS node instance.
        """
        self.node = node
        
        # Declare battery parameters
        node.declare_parameter("battery_id", "hub_battery")
        node.declare_parameter("frequency", 2.0)
        node.declare_parameter("battery_capacity", 100.0)  
        node.declare_parameter("battery_level", 100.0)
        node.declare_parameter("battery_unit", 0.001)
        node.declare_parameter("instant_recharge", False)
        
        # Get battery parameters
        self.battery_id = node.get_parameter("battery_id").value
        self.battery_capacity = float(node.get_parameter("battery_capacity").value)
        self.battery_level = float(node.get_parameter("battery_level").value)
        self.battery_unit = float(node.get_parameter("battery_unit").value)
        self.instant_recharge = node.get_parameter("instant_recharge").value
        
        # Get frequency
        self.frequency = float(node.get_parameter("frequency").value)
        
        # Declare and get adaptation parameter
        self.node.declare_parameter("enable_adaptation", False)
        self.activate_adaptation = self.node.get_parameter("enable_adaptation").value
        
        # Log configuration
        node.get_logger().info(f"Initialized Central Hub with frequency: {self.frequency}Hz")