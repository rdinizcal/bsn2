class ConfigManager:
    """Handles parameter declaration and access for Central Hub"""
    
    def __init__(self, node):
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
        
        # Log configuration
        node.get_logger().info(f"Initialized Central Hub with frequency: {self.frequency}Hz")