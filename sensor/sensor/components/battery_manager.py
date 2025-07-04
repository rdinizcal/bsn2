"""
Battery management for sensor nodes.

This module handles battery state monitoring, charging behavior, energy
consumption tracking, and automatic recharge mode management based on
battery levels.
"""

from shared_components.battery import Battery
from bsn_interfaces.msg import EnergyStatus
from std_msgs.msg import Header


class BatteryManager:
    """
    Manages battery state, charging, and energy reporting.
    
    This class handles all battery-related operations including consumption
    tracking, automatic recharging when levels are low, and publishing
    energy status updates to the system.
    
    Attributes:
        node: Reference to the parent sensor node.
        battery (Battery): Battery component instance.
        is_recharging (bool): Flag indicating if in recharge mode.
        cost (float): Accumulated energy cost since last report.
        energy_status_pub: Publisher for energy status messages.
        battery_id (str): Unique battery identifier.
        battery_capacity (float): Maximum battery capacity.
        battery_level (float): Current battery level.
        battery_unit (float): Energy consumption unit.
        instant_recharge (bool): Whether battery recharges instantly.
        
    Examples:
        ```python
        battery_mgr = BatteryManager(sensor_node)
        
        # Consume energy for an operation
        battery_mgr.consume(2.5)
        
        # Check if recharging is needed
        if battery_mgr.is_recharging:
            battery_mgr.recharge()
        ```
    """
    
    def __init__(self, node):
        """
        Initialize battery manager.
        
        Sets up battery component, publishers, and timers for battery
        monitoring and management.
        
        Args:
            node: The parent sensor node instance.
        """
        self.node = node
        
        # Get battery parameters
        self.battery_id = node.config.battery_id
        self.battery_capacity = node.config.battery_capacity
        self.battery_level = node.config.battery_level
        self.battery_unit = node.config.battery_unit
        self.instant_recharge = node.config.instant_recharge
        
        # Initialize battery
        self.battery = Battery(
            id_name=self.battery_id,
            capacity=self.battery_capacity,
            current_level=self.battery_level,
            unit=self.battery_unit
        )
        
        # Add recharge mode flag
        self.is_recharging = False
        
        self.cost = 0.0
        
        # Create energy status publisher
        self.energy_status_pub = node.create_publisher(
            EnergyStatus, f'collect_energy_status/{node.config.sensor}', 10
        )
        
        # Create timer for battery management
        node.timer = node.create_timer(1.0, self.check_battery_status)
    
    def consume(self, amount):
        """
        Consume battery energy.
        
        Reduces battery level and tracks accumulated cost for operations.
        
        Args:
            amount (float): Amount of energy to consume.
        """
        self.battery.consume(amount)
        self.cost += amount
        
    def recharge(self):
        """
        Recharge the battery.
        
        Increases battery level based on recharge mode. In instant recharge
        mode, battery is fully charged immediately. Otherwise, gradual
        charging is performed.
        """
        if not self.instant_recharge:
            if self.battery.current_level <= 100:
                self.battery.generate(1)
        else:
            self.battery.generate(100)
            
    def check_battery_status(self):
        """
        Periodic battery status check.
        
        Monitors battery level and automatically manages recharge mode
        transitions. Publishes energy status updates and handles charging
        when necessary.
        """
        self.node.get_logger().debug(f"Battery level: {self.battery.current_level:.1f}%")
        
        # Check if we need to enter recharge mode
        if not self.is_recharging and self.battery.current_level < 5.0:
            self.node.get_logger().warn(
                f"Battery low ({self.battery.current_level:.1f}%), entering recharge mode"
            )
            self.enter_recharge_mode()
        
        # Always handle recharging when in recharge mode or instant recharge is enabled
        if self.is_recharging or self.instant_recharge:
            old_level = self.battery.current_level
            self.recharge()
            if self.battery.current_level > old_level + 0.5:
                self.node.get_logger().info(f"Recharging battery: {self.battery.current_level:.1f}%")
        
        # Check if we can exit recharge mode
        if self.is_recharging and self.battery.current_level > 15.0:
            self.node.get_logger().info(
                f"Battery charged enough ({self.battery.current_level:.1f}%), exiting recharge mode"
            )
            self.exit_recharge_mode()
        
        # Always send energy status
        self.send_energy_status()
    
    def enter_recharge_mode(self):
        """
        Enter recharge mode - only essential operations continue.
        
        Suspends normal sensor operations and focuses on battery charging.
        Publishes appropriate status messages to indicate recharging state.
        """
        self.is_recharging = True
        
        # Keep node active but publish recharging status
        self.node.publisher_manager.publish_status("deactivated", "recharging")
        self.node.publisher_manager.publish_event("recharge_start")
        
        self.node.get_logger().info(
            "Entered recharge mode - processing stopped until battery > 15%"
        )
    
    def exit_recharge_mode(self):
        """
        Exit recharge mode - resume normal operations.
        
        Returns to normal sensor operation mode after battery has been
        sufficiently recharged. Updates status to reflect active state.
        """
        self.is_recharging = False
        
        # Update status
        self.node.publisher_manager.publish_status("activated", "idle")
        self.node.publisher_manager.publish_event("recharge_complete")
        
        self.node.get_logger().info(
            "Exited recharge mode - resuming normal processing"
        )
    
    def send_energy_status(self):
        """
        Send energy status report.
        
        Publishes current battery level and accumulated energy cost
        to the system for monitoring purposes. Resets cost counter
        after reporting.
        """
        msg = EnergyStatus()
        msg.header = Header()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.source = self.node.get_name()
        msg.target = "system"
        msg.content = f"energy:{self.battery.current_level:.2f}:cost:{self.cost:.2f}"
        
        self.energy_status_pub.publish(msg)
        self.node.get_logger().debug(f"Energy status: {msg.content}")
        
        # Reset cost counter after reporting
        self.cost = 0.0