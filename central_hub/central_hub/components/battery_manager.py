from system_monitor.battery import Battery
from bsn_interfaces.msg import EnergyStatus
from std_msgs.msg import Header

class BatteryManager:
    """Manages battery state, charging, and energy reporting"""
    
    def __init__(self, node):
        self.node = node
        
        # Initialize battery
        self.battery = Battery(
            id_name=node.config.battery_id,
            capacity=node.config.battery_capacity,
            current_level=node.config.battery_level,
            unit=node.config.battery_unit
        )
        
        self.instant_recharge = node.config.instant_recharge
        self.cost = 0.0
        
        # Energy status publisher
        self.energy_status_pub = node.create_publisher(
            EnergyStatus, 'collect_energy_status/central_hub_node', 10
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
            # Recover 5% per second (100% in 20 seconds)
            self.battery.generate((100.0/20.0)/self.node.config.frequency)
        else:
            self.battery.generate(100)  # Instantly recharge to full
            
    def check_battery_status(self):
        """Check and manage battery status"""
        # Turn on if charged enough, turn off if too low
        if not self.node.active and self.battery.current_level > 90:
            self.node.active = True
            self.node.get_logger().info("Central Hub activated due to sufficient battery")
            self.node.publisher_manager.publish_event("activate")
        elif self.node.active and self.battery.current_level < 2:
            self.node.active = False
            self.node.get_logger().info("Central Hub deactivated due to low battery")
            self.node.publisher_manager.publish_event("deactivate")
            
        # Recharge if inactive or when instant_recharge is enabled
        if not self.node.active or self.instant_recharge:
            self.recharge()
        
        # Log battery status
        self.node.get_logger().info(f"Hub battery level: {self.battery.current_level:.1f}%")
    
    def send_energy_status(self):
        """Send energy status to monitoring system"""
        msg = EnergyStatus()
        msg.header = Header()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.source = self.node.get_name()
        msg.target = "system"
        msg.content = f"energy:{self.battery.current_level:.2f}:cost:{self.cost:.4f}"
        
        self.energy_status_pub.publish(msg)
        self.node.get_logger().debug(f"Energy status: {msg.content}")
        
        # Reset cost counter after reporting
        self.cost = 0.0