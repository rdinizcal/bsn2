"""
Configuration management for sensor nodes.

This module handles parameter declaration, validation, and access for sensor
node configuration including battery settings, risk evaluation parameters,
and sensor-specific settings.
"""


from shared_components.core.config_manager_base import ConfigManagerBase


class ConfigManager(ConfigManagerBase):
    """
    Handles parameter declaration and access for sensor nodes.
    
    This class manages all configuration parameters for a sensor node including
    basic sensor settings, battery configuration, and risk evaluation parameters.
    It declares ROS parameters and provides easy access to configuration values.
    
    Attributes:
        node: Reference to the parent ROS node.
        sensor (str): Type of sensor (e.g., 'thermometer', 'ecg').
        vital_sign (str): Type of vital sign measured.
        frequency (float): Data collection frequency in Hz.
        window_size (int): Size of moving average window.
        battery_id (str): Unique identifier for the battery.
        battery_capacity (float): Maximum battery capacity.
        battery_level (float): Initial battery level.
        battery_unit (float): Battery consumption unit.
        instant_recharge (bool): Whether battery recharges instantly.
        activate_adaptation (bool): Whether adaptation is enabled.
        
    Examples:
        ```python
        config = ConfigManager(sensor_node)
        print(f"Sensor type: {config.sensor}")
        print(f"Frequency: {config.frequency} Hz")
        ```
    """
    
    def __init__(self, node):
        """
        Initialize configuration manager.
        
        Declares all required ROS parameters and extracts their values
        for easy access throughout the sensor node.
        
        Args:
            node: The parent ROS node instance.
        """
        # initialize base
        super().__init__(node)
        # sensor-specific loading implemented in `load`
        self.load(node)
        self.validate()

    def load(self, node) -> None:
        """Declare and read sensor-specific parameters.

        This implements the abstract `load` API from the base class and
        centralizes sensor-only parameter handling (vital_sign, risk
        ranges, window size, battery defaults that depend on component
        name, etc.).
        """
        node = self.node

        # Declare basic parameters (component and frequency declared by base)
        node.declare_parameter("vital_sign", "")

        self.vital_sign = node.get_parameter("vital_sign").get_parameter_value().string_value

        # Declare and read risk evaluation parameters
        self._declare_risk_parameters()

        # Set window size for moving average
        self.window_size = 5

        # Log configuration
        node.get_logger().info(
            f"Initialized sensor: {self.component}, vital sign: {self.vital_sign}, "
            f"frequency: {self.frequency}Hz"
        )
        
    def _declare_risk_parameters(self):
        """
        Declare risk evaluation parameters.
        
        Sets up parameters for risk percentage ranges and sensor-specific
        risk value ranges used in risk assessment calculations.
        """
        # Risk percentage ranges
        self.node.declare_parameter("lowrisk_percent", "0,20")
        self.node.declare_parameter("midrisk_percent", "21,65")
        self.node.declare_parameter("highrisk_percent", "66,100")
        
        # Save the risk percentage attributes 
        self.lowrisk_percent = self.node.get_parameter("lowrisk_percent").value
        self.midrisk_percent = self.node.get_parameter("midrisk_percent").value
        self.highrisk_percent = self.node.get_parameter("highrisk_percent").value
        
        # Risk value ranges
        self.node.declare_parameter("HighRisk0", "-1,-1")
        self.node.declare_parameter("MidRisk0", "-1,-1")
        self.node.declare_parameter("LowRisk", "-1,-1")
        self.node.declare_parameter("MidRisk1", "-1,-1")
        self.node.declare_parameter("HighRisk1", "-1,-1")
        
        # Save the risk range attributes
        self.HighRisk0 = self.node.get_parameter("HighRisk0").value
        self.MidRisk0 = self.node.get_parameter("MidRisk0").value
        self.LowRisk = self.node.get_parameter("LowRisk").value
        self.MidRisk1 = self.node.get_parameter("MidRisk1").value
        self.HighRisk1 = self.node.get_parameter("HighRisk1").value

        #self.activate_adaptation = True