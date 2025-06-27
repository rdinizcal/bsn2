from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
import rclpy
import threading
import time
from sensor.components.data_processor import DataProcessor
from sensor.components.battery_manager import BatteryManager
from sensor.components.risk_manager import RiskManager
from sensor.components.publishers import PublisherManager
from sensor.components.config_manager import ConfigManager
from shared_components.lifecycle_manager import LifecycleManager  # Add this import
from shared_components.adaptation_handler import AdaptationHandler
class Sensor(LifecycleNode):
    """Main sensor node class with individual lifecycle management."""
    
    def __init__(self, node_name: str, parameters=None):
        super().__init__(node_name, parameter_overrides=parameters or [])
        
        # Configuration manager (loads and manages parameters)
        self.config = ConfigManager(self)
        
        # Component managers
        self.battery_manager = BatteryManager(self)
        self.publisher_manager = PublisherManager(self)
        self.risk_manager = RiskManager(self)
        self.processor = DataProcessor(self)
        
        # Lifecycle manager - handles its own state transitions
        self.lifecycle_manager = LifecycleManager(self)
        
        self.adaptation_handler = AdaptationHandler(self)
        if self.config.activate_adaptation: 
            self.adaptation_handler.register_with_effector()
        # Node state
        self.active = False
        self._finalized = False
        self._heartbeat_timer = None
        
        # Configure lifecycle management
        self.lifecycle_manager.set_auto_management_flags(
            auto_configure=True,
            auto_activate=True,
            battery_aware=True,
            auto_recovery=True
        )
        
        # Set battery thresholds for lifecycle decisions
        self.lifecycle_manager.configure_thresholds(
            battery_low=5.0,
            battery_recovery=15.0
        )
        
    # Lifecycle callbacks remain the same but are now managed by LifecycleManager
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Handle transition to Configured state."""
        self.get_logger().info(f"Configuring {self.config.sensor} sensor...")
        
        try:
            # Set up publishers
            self.publisher_manager.setup_publishers()
            # Publish configured status
            self.publisher_manager.publish_status("configured", "idle")
            
            # Set up risk manager with sensor-specific configuration
            self.risk_manager.configure_risk_ranges()
            
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Error during configuration: {e}")
            return TransitionCallbackReturn.ERROR
        
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Handle transition to Active state."""
        self.get_logger().info(f"Activating {self.config.sensor} sensor...")
        
        try:
            # Update node state
            self.active = True
            
            # Set up heartbeat timer
            if self._heartbeat_timer is None:
                self._heartbeat_timer = self.create_timer(
                    2.0, self.publisher_manager.publish_heartbeat
                )
            else:
                self._heartbeat_timer.reset()
            
            # Publish immediate activation events
            self.publisher_manager.publish_event("activate")
            self.publisher_manager.publish_status("activated", "idle")
            
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Error during activation: {e}")
            return TransitionCallbackReturn.ERROR
        
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Handle transition to Inactive state."""
        self.get_logger().info(f"Deactivating {self.config.sensor} sensor...")
        
        try:
            # Update node state
            self.active = False
        
            
            # Publish deactivation events
            self.publisher_manager.publish_event("deactivate")
            self.publisher_manager.publish_status("deactivated", "idle")
            
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Error during deactivation: {e}")
            return TransitionCallbackReturn.ERROR
        
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Handle transition to Unconfigured state."""
        self.get_logger().info(f"Cleaning up {self.config.sensor} sensor...")
        
        try:
            # Cancel heartbeat timer but don't set to None to allow reactivation
            if self._heartbeat_timer:
                self._heartbeat_timer.cancel()
            
            # Reset data processor if it has a reset method
            if hasattr(self.processor, 'reset'):
                self.processor.reset()
            
            # Publish status update
            self.publisher_manager.publish_status("unconfigured", "idle")
            
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Error during cleanup: {e}")
            return TransitionCallbackReturn.ERROR
        
    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Handle shutdown transition."""
        self.get_logger().info(f"Shutting down {self.config.sensor} sensor...")
        
        try:
            # Stop lifecycle management
            self.lifecycle_manager.stop_auto_management()
            
            # Update node state flags
            self.active = False
            self._finalized = True
            
            # Cancel all timers
            if self._heartbeat_timer:
                self._heartbeat_timer.cancel()
                self._heartbeat_timer = None
            
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Error during shutdown: {e}")
            return TransitionCallbackReturn.FAILURE
    
    # Main functionality
    def is_active(self):
        return self.active
        
    def spin_sensor(self):
        """Main sensor loop with individual lifecycle management"""
        rate = self.create_rate(self.config.frequency)
        
        # Start automatic lifecycle management
        self.get_logger().info("Starting individual lifecycle management...")
        self.lifecycle_manager.start_auto_management()
        
        while rclpy.ok():
            if self._finalized:
                self.get_logger().info("Node has been shut down, exiting spin loop")
                break
                
            # Only process if active AND not in recharge mode
            if self.active and not self.battery_manager.is_recharging:
                try:
                    # Perform sensor operations through processor
                    datapoint = self.processor.collect()
                    if datapoint >= 0:
                        datapoint = self.processor.process(datapoint)
                        self.processor.transfer(datapoint)
                except Exception as e:
                    self.get_logger().error(f"Sensor operation failed: {str(e)}")
            else:
                # Handle recharging while inactive or in recharge mode
                self.battery_manager.recharge()
                
            rate.sleep()
            
def main(args=None):
    rclpy.init(args=args)

    # Get node name from CLI remapping (set in the launch file)
    sensor_node = Sensor(node_name="sensor_node")

    thread = threading.Thread(target=rclpy.spin, args=(sensor_node,), daemon=True)
    thread.start()

    try:
        sensor_node.spin_sensor()
    finally:
        sensor_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
