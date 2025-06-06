from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
import rclpy
import threading
import time

# Import component managers
from central_hub.components.battery_manager import BatteryManager
from central_hub.components.config_manager import ConfigManager
from central_hub.components.data_fusion import DataFusionEngine
from central_hub.components.publishers import PublisherManager
from central_hub.components.sensor_data_handler import SensorDataHandler
from central_hub.utils.visualization import Visualizer
from central_hub.utils.risk_analyzer import RiskAnalyzer

class CentralHub(LifecycleNode):
    """
    Central Hub for the Body Sensor Network.
    
    This class coordinates different components to monitor patient health.
    """
    
    def __init__(self):
        super().__init__("central_hub")
        
        # Configuration manager (loads and manages parameters)
        self.config = ConfigManager(self)
        
        # Component managers
        self.battery_manager = BatteryManager(self)
        self.publisher_manager = PublisherManager(self)
        self.sensor_handler = SensorDataHandler(self)
        self.fusion_engine = DataFusionEngine(self)
        self.risk_analyzer = RiskAnalyzer(self)
        self.visualizer = Visualizer(self)
        
        # Node state
        self.active = False
        self._finalized = False
        self._heartbeat_timer = None
    
    # Lifecycle Node callback methods
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Callback when transitioning to Configured state."""
        self.get_logger().info("Configuring Central Hub...")
        
        try:
            # Set up publishers
            self.publisher_manager.setup_publishers()
            
            # Setup subscriptions
            self.sensor_handler.setup_subscriptions()
            
            # Publish configured status
            self.publisher_manager.publish_status("configured", "idle")
            
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Error during configuration: {e}")
            return TransitionCallbackReturn.ERROR

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating Central Hub...")
        
        try:
            self.active = True
            # Start heartbeat timer
            if self._heartbeat_timer is None:
                self._heartbeat_timer = self.create_timer(
                    2.0, self.publisher_manager.publish_heartbeat
                )
            else:
                self._heartbeat_timer.reset()
                
            # Publish activation event immediately
            self.publisher_manager.publish_event("activate")
            self.publisher_manager.publish_status("activated", "idle")
            
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Error during activation: {e}")
            return TransitionCallbackReturn.ERROR

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating Central Hub...")
        
        try:
            self.active = False
            
            # Keep heartbeat timer running, but change content to "deactivate"
            # DO NOT cancel the timer
            
            # Publish deactivation events
            self.publisher_manager.publish_event("deactivate")
            self.publisher_manager.publish_status("deactivated", "idle")
            
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Error during deactivation: {e}")
            return TransitionCallbackReturn.ERROR

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up Central Hub...")
        
        try:
            # Clean up subscriptions
            self.sensor_handler.cleanup_subscriptions()
            
            self.publisher_manager.publish_status("unconfigured", "idle")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Error during cleanup: {e}")
            return TransitionCallbackReturn.ERROR

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down Central Hub...")
        
        try:
            self.active = False
            self._finalized = True
            
            # Cancel heartbeat timer
            if self._heartbeat_timer:
                self._heartbeat_timer.cancel()
                self._heartbeat_timer = None
                
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Error during shutdown: {e}")
            return TransitionCallbackReturn.FAILURE
    
    # Main functionality methods
    def is_active(self):
        """Check if hub is active"""
        return self.active
        
    def detect(self):
        """Main detection function that processes and publishes health data"""
        # Skip processing if hub is not active
        if not self.is_active():
            self.battery_manager.recharge()
            return
            
        try:
            # Run data fusion to get patient status
            patient_status = self.fusion_engine.fuse_data()
            
            # Format and display log message
            log_message = self.visualizer.format_log_message()
            self.get_logger().info(log_message)
            
            # Emit alerts for high/moderate risks
            self.risk_analyzer.emit_alert(patient_status)
            
            # Publish system data
            self.publisher_manager.publish_system_data(
                patient_status, 
                self.sensor_handler.latest_data,
                self.sensor_handler.latest_risk,
                self.sensor_handler.sensor_battery_levels
            )
            
            # Report energy status
            self.battery_manager.send_energy_status()
            
        except Exception as e:
            self.get_logger().error(f"Error in detect(): {str(e)}")


def main(args=None):
    rclpy.init(args=args)

    hub = CentralHub()
    
    # Run spin in a thread
    thread = threading.Thread(
        target=rclpy.spin, args=(hub,), daemon=True
    )
    thread.start()
    rate = hub.create_rate(hub.config.frequency)

    try:
        # Automatically configure and activate
        hub.trigger_configure()
        time.sleep(0.5)  # Wait for configuration
        hub.trigger_activate()
        time.sleep(0.5)  # Wait for activation
        
        while rclpy.ok():
            if hub._finalized:
                break
                
            # Only run detection if hub is active
            if hub.is_active():
                hub.detect()
            else:
                # If hub is inactive, try to recharge
                hub.battery_manager.recharge()
                hub.get_logger().warning(
                    f"Hub in power saving mode, recharging: {hub.battery_manager.battery.current_level:.1f}%"
                )
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        hub.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()