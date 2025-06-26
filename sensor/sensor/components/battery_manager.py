from system_monitor.battery import Battery
from bsn_interfaces.msg import EnergyStatus
from std_msgs.msg import Header

class BatteryManager:
    """Manages battery state, charging, and energy reporting"""
    
    def __init__(self, node):
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
        """Consume battery energy"""
        self.battery.consume(amount)
        self.cost += amount
        
    def recharge(self):
        """Recharge the battery"""
        if not self.instant_recharge:
            if self.battery.current_level <= 100:
                self.battery.generate(1)
        else:
            self.battery.generate(100)
            
    def check_battery_status(self):
        """Periodic battery status check"""
        self.node.get_logger().debug(f"Battery level: {self.battery.current_level:.1f}%")
        
        # Check if we need to enter recharge mode
        if not self.is_recharging and self.battery.current_level < 5.0:
            self.node.get_logger().warn(f"Battery low ({self.battery.current_level:.1f}%), entering recharge mode")
            self.enter_recharge_mode()
        
        # Always handle recharging when in recharge mode or instant recharge is enabled
        if self.is_recharging or self.instant_recharge:
            old_level = self.battery.current_level
            self.recharge()
            if self.battery.current_level > old_level + 0.5:
                self.node.get_logger().info(f"Recharging battery: {self.battery.current_level:.1f}%")
        
        # Check if we can exit recharge mode
        if self.is_recharging and self.battery.current_level > 15.0:
            self.node.get_logger().info(f"Battery charged enough ({self.battery.current_level:.1f}%), exiting recharge mode")
            self.exit_recharge_mode()
        
        # Always send energy status
        self.send_energy_status()
    
    def enter_recharge_mode(self):
        """Enter recharge mode - only essential operations continue"""
        self.is_recharging = True
        
        # Keep node active but publish recharging status
        self.node.publisher_manager.publish_status("deactivated", "recharging")
        self.node.publisher_manager.publish_event("recharge_start")
        
        self.node.get_logger().info("Entered recharge mode - processing stopped until battery > 15%")
    
    def exit_recharge_mode(self):
        """Exit recharge mode - resume normal operations"""
        self.is_recharging = False
        
        # Update status
        self.node.publisher_manager.publish_status("activated", "idle")
        self.node.publisher_manager.publish_event("recharge_complete")
        
        self.node.get_logger().info("Exited recharge mode - resuming normal processing")
    
    def send_energy_status(self):
        """Send energy status report"""
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