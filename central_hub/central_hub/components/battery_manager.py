"""
Battery management for central hub node.

This module handles battery state monitoring, charging behavior, energy
consumption tracking, and automatic recharge mode management for the
central hub in the Body Sensor Network.
"""

from shared_components.battery import Battery
from bsn_interfaces.msg import EnergyStatus
from std_msgs.msg import Header


class BatteryManager:
    """
    Manages battery state, charging, and energy reporting for central hub.
    
    This class handles all battery-related operations for the central hub
    including consumption tracking, automatic recharging when levels are low,
    and publishing energy status updates to the system for monitoring.
    
    Attributes:
        node: Reference to the parent central hub node.
        battery (Battery): Battery component instance.
        is_recharging (bool): Flag indicating if in recharge mode.
        low_threshold (float): Battery level that triggers recharge mode.
        recovery_threshold (float): Battery level to exit recharge mode.
        instant_recharge (bool): Whether battery recharges instantly.
        cost (float): Accumulated energy cost since last report.
        energy_status_pub: Publisher for energy status messages.
        
    Examples:
        ```python
        battery_mgr = BatteryManager(central_hub_node)
        
        # Consume energy for processing
        battery_mgr.consume(1.5)
        
        # Check if recharging is needed
        if battery_mgr.is_recharging:
            battery_mgr.recharge()
        ```
    """
    
    def __init__(self, node):
        """
        Initialize battery manager for central hub.
        
        Sets up battery component, energy status publisher, and battery
        monitoring timer with hub-specific thresholds and settings.
        
        Args:
            node: The parent central hub node instance.
        """
        self.node = node
        
        # Initialize battery
        self.battery = Battery(
            id_name=node.config.battery_id,
            capacity=node.config.battery_capacity,
            current_level=node.config.battery_level,
            unit=node.config.battery_unit
        )
        
        # Add recharge mode flag and thresholds
        self.is_recharging = False
        self.low_threshold = 2.0  # When to enter recharge mode
        self.recovery_threshold = 15.0  # When to exit recharge mode
        
        self.instant_recharge = node.config.instant_recharge
        self.cost = 0.0
        
        # Energy status publisher
        self.energy_status_pub = node.create_publisher(
            EnergyStatus, 'collect_energy_status/central_hub_node', 10
        )
        
        # Create timer for battery management
        node.timer = node.create_timer(1.0, self.check_battery_status)
    
    def consume(self, amount):
        """
        Consume battery energy for hub operations.
        
        Reduces battery level and tracks accumulated cost for various
        hub operations including data processing and communication.
        
        Args:
            amount (float): Amount of energy to consume.
        """
        self.battery.consume(amount)
        self.cost += amount
        
    def recharge(self):
        """
        Recharge the central hub battery.
        
        Increases battery level based on recharge mode. In instant recharge
        mode, battery is fully charged immediately. Otherwise, gradual
        charging provides 5% per second (full charge in 20 seconds).
        """
        if not self.instant_recharge:
            # Recover 5% per second (100% in 20 seconds)
            self.battery.generate(5.0)
        else:
            self.battery.generate(100)  # Instantly recharge to full
    
    def check_battery_status(self):
        """
        Periodic battery status monitoring and management.
        
        Monitors battery level and automatically manages recharge mode
        transitions. Handles charging when necessary and publishes energy
        status updates for system monitoring.
        """
        # Check if we need to enter recharge mode
        if not self.is_recharging and self.battery.current_level < self.low_threshold:
            self.node.get_logger().warn(f"Battery low ({self.battery.current_level:.1f}%), entering recharge mode")
            self.enter_recharge_mode()
        
        # Always handle recharging when in recharge mode or instant recharge is enabled
        if self.is_recharging or self.instant_recharge:
            old_level = self.battery.current_level
            self.recharge()
            if self.battery.current_level > old_level + 0.5:
                self.node.get_logger().info(f"Recharging battery: {self.battery.current_level:.1f}%")
        
        # Check if we can exit recharge mode
        if self.is_recharging and self.battery.current_level > self.recovery_threshold:
            self.node.get_logger().info(f"Battery charged enough ({self.battery.current_level:.1f}%), exiting recharge mode")
            self.exit_recharge_mode()
        
        # Always send energy status
        self.send_energy_status()
    
    def enter_recharge_mode(self):
        """
        Enter recharge mode - suspend non-essential operations.
        
        Puts the hub into power conservation mode where only essential
        operations continue. Publishes appropriate status messages to
        indicate the hub is in recharge state.
        """
        self.is_recharging = True
        
        # Keep node active but publish recharging status
        self.node.publisher_manager.publish_status("deactivated", "recharging")
        self.node.publisher_manager.publish_event("recharge_start")
        
        self.node.get_logger().info("Entered recharge mode - processing stopped until battery > 15%")
    
    def exit_recharge_mode(self):
        """
        Exit recharge mode - resume normal operations.
        
        Returns the hub to normal operation mode after battery has been
        sufficiently recharged. Updates status to reflect active processing
        state and resumes emergency detection operations.
        """
        self.is_recharging = False
        
        # Update status
        self.node.publisher_manager.publish_status("activated", "idle")
        self.node.publisher_manager.publish_event("recharge_complete")
        
        self.node.get_logger().info("Exited recharge mode - resuming normal processing")
    
    def send_energy_status(self):
        """
        Send energy status report to system monitoring.
        
        Publishes current battery level and accumulated energy cost
        to the system for monitoring and analysis. Resets cost counter
        after reporting to track incremental consumption.
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