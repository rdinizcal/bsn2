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
#from central_hub.components.lifecycle_manager import LifecycleManager
from central_hub.utils.visualization import Visualizer
from central_hub.utils.risk_analyzer import RiskAnalyzer
from shared_components.lifecycle_manager import LifecycleManager  # Add this import
from shared_components.adaptation_handler import AdaptationHandler

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
        
        # Add lifecycle manager
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
            battery_low=2.0,  # Lower for hub
            battery_recovery=15.0
        )

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

    def is_active(self):
        """Check if hub is active"""
        return self.active

    def detect(self):
        """Main detection function that processes and publishes health data"""
        # Skip processing if hub is not active or in recharge mode
        if not self.is_active() or (hasattr(self.battery_manager, 'is_recharging') and self.battery_manager.is_recharging):
            # Always handle recharging
            if hasattr(self.battery_manager, 'recharge'):
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
    
    # Start automatic lifecycle management
    hub.lifecycle_manager.start_auto_management()
    
    # Run spin in a thread
    thread = threading.Thread(
        target=rclpy.spin, args=(hub,), daemon=True
    )
    thread.start()
    rate = hub.create_rate(hub.config.frequency)

    try:
        while rclpy.ok():
            if hub._finalized:
                break
                
            # Run detection if active and not in recharge mode
            if hub.is_active() and not hub.battery_manager.is_recharging:
                hub.detect()
            else:
                # Let battery manager handle recharging
                hub.get_logger().debug(
                    f"Hub in {'recharging' if hub.battery_manager.is_recharging else 'inactive'} state, "
                    f"battery: {hub.battery_manager.battery.current_level:.1f}%"
                )
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        hub.destroy_node()
        rclpy.shutdown()
        thread.join()


if __name__ == "__main__":
    main()