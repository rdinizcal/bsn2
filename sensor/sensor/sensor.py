"""
Main sensor node implementation with lifecycle management.

This module provides the core sensor functionality including data collection,
processing, risk evaluation, and battery management within a ROS 2 lifecycle
node framework.
"""

from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
import rclpy
import threading
import time
from sensor.components.data_processor import DataProcessor
from sensor.components.battery_manager import BatteryManager
from sensor.components.risk_manager import RiskManager
from sensor.components.publishers import PublisherManager
from sensor.components.config_manager import ConfigManager
from shared_components.lifecycle_manager import LifecycleManager
from shared_components.adaptation_handler import AdaptationHandler


class Sensor(LifecycleNode):
    """
    Main sensor node class with individual lifecycle management.
    
    This class implements a ROS 2 lifecycle node for sensor data collection,
    processing, and publishing. It manages battery consumption, risk evaluation,
    and automatic lifecycle transitions based on battery levels.
    
    Attributes:
        config (ConfigManager): Manages node parameters and configuration.
        battery_manager (BatteryManager): Handles battery state and charging.
        publisher_manager (PublisherManager): Manages all ROS publishers.
        risk_manager (RiskManager): Evaluates risk levels for sensor data.
        processor (DataProcessor): Handles data collection and processing.
        lifecycle_manager (LifecycleManager): Manages automatic state transitions.
        adaptation_handler (AdaptationHandler): Handles system adaptation.
        active (bool): Current activation state of the node.
        
    Examples:
        Basic usage:
        ```python
        import rclpy
        from sensor.sensor import Sensor
        
        rclpy.init()
        sensor_node = Sensor("thermometer_node")
        rclpy.spin(sensor_node)
        ```
    """
    
    def __init__(self, node_name: str, parameters=None):
        """
        Initialize the sensor node.
        
        Args:
            node_name: Name of the ROS node.
            parameters: Optional list of parameter overrides.
        """
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
        
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """
        Handle transition to Configured state.
        
        Sets up publishers and configures risk evaluation ranges based on
        sensor type and parameters.
        
        Args:
            state: Current lifecycle state.
            
        Returns:
            SUCCESS if configuration successful, ERROR otherwise.
        """
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
        """
        Handle transition to Active state.
        
        Activates the sensor for data collection and starts heartbeat timer.
        
        Args:
            state: Current lifecycle state.
            
        Returns:
            SUCCESS if activation successful, ERROR otherwise.
        """
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
        """
        Handle transition to Inactive state.
        
        Deactivates the sensor and stops data collection.
        
        Args:
            state: Current lifecycle state.
            
        Returns:
            SUCCESS if deactivation successful, ERROR otherwise.
        """
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
        """
        Handle transition to Unconfigured state.
        
        Cleans up resources and resets data processor state.
        
        Args:
            state: Current lifecycle state.
            
        Returns:
            SUCCESS if cleanup successful, ERROR otherwise.
        """
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
        """
        Handle shutdown transition.
        
        Performs final cleanup and stops all operations.
        
        Args:
            state: Current lifecycle state.
            
        Returns:
            SUCCESS if shutdown successful, FAILURE otherwise.
        """
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
    
    def is_active(self):
        """
        Check if the sensor is currently active.
        
        Returns:
            bool: True if sensor is active, False otherwise.
        """
        return self.active
        
    def spin_sensor(self):
        """
        Main sensor loop with individual lifecycle management.
        
        Continuously collects, processes, and publishes sensor data while
        managing battery consumption and automatic lifecycle transitions.
        The loop runs until ROS is shut down or the node is finalized.
        """
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
    """
    Main entry point for the sensor node.
    
    Initializes ROS, creates the sensor node, and starts the main sensor loop
    in a separate thread while spinning the node for ROS communication.
    
    Args:
        args: Command line arguments passed to ROS.
    """
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
