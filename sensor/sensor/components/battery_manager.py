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
        
        # Battery-based lifecycle management
        if self.node.active and self.battery.current_level < 5.0:
            self.node.get_logger().warn(f"Battery low ({self.battery.current_level:.1f}%), deactivating {self.node.sensor}")
            self.node.lifecycle_manager.deactivate_node()
        
        # Handle recharging
        if self.node.active or self.instant_recharge:
            old_level = self.battery.current_level
            self.recharge()
            if self.battery.current_level > old_level + 0.5:
                self.node.get_logger().info(f"Recharging battery: {self.battery.current_level:.1f}%")
        
        # Battery-based reactivation - must check if node is in correct state (inactive)    
        if self.node.active and self.battery.current_level > 15.0:
            self.node.get_logger().info(f"Battery charged enough ({self.battery.current_level:.1f}%), activating {self.node.get_name()}")
            self.node.lifecycle_manager.activate_node()
        elif self.node.active and self.battery.current_level > 15.0:
            if self.node.lifecycle_manager.configure_node():
                # Wait a moment for configuration to complete
                self.node.create_timer(0.5, lambda: self.node.lifecycle_manager.activate_node())
    
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