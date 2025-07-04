"""
Automatic lifecycle management for ROS 2 lifecycle nodes.

This module provides intelligent lifecycle management capabilities for ROS 2
lifecycle nodes, including battery-aware state transitions, automatic recovery,
and configurable management policies for BSN system components.
"""

from rclpy.lifecycle import State, TransitionCallbackReturn
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition
import threading
import time


class LifecycleManager:
    """
    Intelligent lifecycle manager for ROS 2 lifecycle nodes in BSN systems.
    
    This class provides comprehensive lifecycle management including automatic
    state transitions, battery-aware operations, self-monitoring, recovery
    capabilities, and configurable management policies. It enables BSN components
    to operate autonomously while respecting power constraints and system health.
    
    The manager handles the complete lifecycle from initial configuration through
    active operation to graceful shutdown, with intelligent decision-making based
    on battery levels, system conditions, and configured policies.
    
    Attributes:
        node: Reference to the managed ROS 2 lifecycle node.
        auto_manage (bool): Whether automatic management is enabled.
        monitoring_active (bool): Whether lifecycle monitoring is running.
        battery_low_threshold (float): Battery level triggering low-power mode.
        battery_recovery_threshold (float): Battery level for returning to active.
        auto_configure_delay (float): Delay before automatic configuration.
        auto_activate_delay (float): Delay before automatic activation.
        last_state_check (float): Timestamp of last state monitoring check.
        state_check_interval (float): Interval between state checks.
        auto_configure_on_start (bool): Whether to auto-configure on startup.
        auto_activate_after_configure (bool): Whether to auto-activate after config.
        battery_aware_deactivation (bool): Whether to deactivate on low battery.
        auto_recovery (bool): Whether to automatically recover from failures.
        
    Examples:
        Basic lifecycle management:
        ```python
        from shared_components.lifecycle_manager import LifecycleManager
        
        # Create lifecycle node
        node = MyLifecycleNode()
        
        # Add lifecycle management
        lifecycle_mgr = LifecycleManager(node)
        lifecycle_mgr.start_auto_management()
        
        # Node will automatically configure and activate
        ```
        
        Custom configuration:
        ```python
        mgr = LifecycleManager(node)
        
        # Configure battery thresholds
        mgr.configure_thresholds(battery_low=10.0, battery_recovery=20.0)
        
        # Set management policies
        mgr.set_auto_management_flags(
            auto_configure=True,
            auto_activate=True,
            battery_aware=True,
            auto_recovery=True
        )
        
        mgr.start_auto_management()
        ```
    """
    
    def __init__(self, node):
        """
        Initialize lifecycle manager for a ROS 2 lifecycle node.
        
        Sets up lifecycle management with default configuration including
        battery thresholds, timing parameters, and management policies.
        The manager is initialized but not started until explicitly activated.
        
        Args:
            node: ROS 2 lifecycle node instance to manage. Must implement
                  lifecycle transitions and provide get_logger() method.
                  
        Examples:
            ```python
            # Create manager for sensor node
            sensor_mgr = LifecycleManager(sensor_node)
            
            # Create manager for central hub
            hub_mgr = LifecycleManager(central_hub_node)
            ```
        """
        self.node = node
        self.auto_manage = True
        self.monitoring_active = False
        
        # Configuration parameters
        self.battery_low_threshold = 5.0
        self.battery_recovery_threshold = 15.0
        self.auto_configure_delay = 2.0
        self.auto_activate_delay = 1.0
        
        # State tracking
        self.last_state_check = time.time()
        self.state_check_interval = 5.0
        
        # Auto-management flags
        self.auto_configure_on_start = True
        self.auto_activate_after_configure = True
        self.battery_aware_deactivation = True
        self.auto_recovery = True
        
        self.node.get_logger().info("Lifecycle Manager initialized")
    
    def start_auto_management(self):
        """
        Start automatic lifecycle management with monitoring and policies.
        
        Initiates the automatic lifecycle management system including periodic
        state monitoring, initial configuration sequence, and battery-aware
        operations. Creates timers for ongoing management tasks.
        
        This method is idempotent - calling it multiple times has no effect
        if management is already active.
        
        Examples:
            ```python
            mgr = LifecycleManager(node)
            
            # Start management - node will auto-configure and activate
            mgr.start_auto_management()
            
            # Management is now running in background
            ```
        """
        if self.monitoring_active:
            return
            
        self.monitoring_active = True
        self.auto_manage = True
        
        # Create timer for periodic state monitoring
        self.monitor_timer = self.node.create_timer(
            self.state_check_interval, 
            self.monitor_lifecycle_state
        )
        
        # Start initial configuration sequence
        if self.auto_configure_on_start:
            self.node.create_timer(
                self.auto_configure_delay, 
                self.initial_setup
            )
        
        self.node.get_logger().info("Lifecycle auto-management started")
    
    def stop_auto_management(self):
        """
        Stop automatic lifecycle management and monitoring.
        
        Disables all automatic lifecycle management including state monitoring,
        recovery attempts, and policy enforcement. The node remains in its
        current state but will no longer be automatically managed.
        
        Examples:
            ```python
            # Stop automatic management for manual control
            mgr.stop_auto_management()
            
            # Now manually control lifecycle
            mgr.configure_node()
            mgr.activate_node()
            ```
        """
        self.auto_manage = False
        self.monitoring_active = False
        
        if hasattr(self, 'monitor_timer'):
            self.monitor_timer.cancel()
            
        self.node.get_logger().info("Lifecycle auto-management stopped")
    
    def initial_setup(self):
        """
        Perform initial configuration and activation sequence.
        
        Executes the startup sequence for a lifecycle node including automatic
        configuration and optional activation. Respects the configured delays
        and management policies to ensure proper initialization timing.
        
        This method is typically called automatically during startup but can
        be invoked manually for re-initialization scenarios.
        """
        # Always assume unconfigured on initial setup
        if self.node.active:
            self.node.get_logger().debug("Node is already active, skipping initial setup")
            return
        
        # Always configure first
        self.node.get_logger().info("Auto-configuring node...")
        success = self.configure_node()
        
        if success and self.auto_activate_after_configure:
            # Use a one-shot timer for activation
            def activate_once():
                # Assume node is now in inactive state after configuration
                self.activate_node()
            
            # Schedule activation after configuration
            self.node.create_timer(self.auto_activate_delay, activate_once)
    
    def monitor_lifecycle_state(self):
        """
        Monitor and manage lifecycle state based on system conditions.
        
        Performs periodic monitoring of node state and battery conditions,
        implementing automatic recovery policies and battery-aware management.
        This method runs continuously when auto-management is enabled.
        
        The monitor checks for:
        - Inactive nodes that should be recovered
        - Unconfigured nodes that need configuration
        - Battery levels for power-aware decisions
        - System health for recovery decisions
        """
        if not self.auto_manage:
            return
            
        battery_level = self.get_battery_level()
        
        # Auto-recovery logic
        if self.auto_recovery and not self.node.active:
            # If node has been configured before but isn't active
            if hasattr(self.node, '_was_configured'):
                # Try to activate if battery level is sufficient
                if battery_level > self.battery_recovery_threshold:
                    self.node.get_logger().info("Auto-recovery: activating node")
                    self.activate_node()
            else:
                # Node hasn't been configured yet
                self.node.get_logger().warn("Node needs configuration, attempting...")
                if self.configure_node():
                    setattr(self.node, '_was_configured', True)
                
        self.last_state_check = time.time()
    
    def get_battery_level(self):
        """
        Get current battery level from the managed node.
        
        Retrieves the current battery level from the node's battery manager
        if available, providing a fallback default for nodes without
        battery management.
        
        Returns:
            float: Current battery level as percentage (0-100). Returns 100.0
                   if no battery manager is available or on error.
                   
        Examples:
            ```python
            level = mgr.get_battery_level()
            if level < 10.0:
                print("Battery critically low!")
            ```
        """
        try:
            if hasattr(self.node, 'battery_manager') and hasattr(self.node.battery_manager, 'battery'):
                return self.node.battery_manager.battery.current_level
            return 100.0  # Default if no battery manager
        except Exception:
            return 100.0
    
    def configure_node(self):
        """
        Configure the managed lifecycle node.
        
        Triggers the configuration transition on the managed node and handles
        the response. Marks the node as configured for future management
        decisions and provides appropriate logging.
        
        Returns:
            bool: True if configuration was successful, False otherwise.
            
        Examples:
            ```python
            if mgr.configure_node():
                print("Node configured successfully")
            else:
                print("Configuration failed")
            ```
        """
        try:
            self.node.get_logger().info("Triggering node configuration...")
            result = self.node.trigger_configure()
            
            if result == TransitionCallbackReturn.SUCCESS or int(result) == int(TransitionCallbackReturn.SUCCESS):
                self.node.get_logger().info("Node configuration successful")
                # Mark as configured for future reference
                setattr(self.node, '_was_configured', True)
                return True
            else:
                self.node.get_logger().error("Node configuration failed")
                return False
                
        except Exception as e:
            self.node.get_logger().error(f"Error during configuration: {e}")
            return False
    
    def activate_node(self):
        """
        Activate the managed lifecycle node with battery awareness.
        
        Triggers the activation transition if conditions are met including
        sufficient battery level and proper configuration state. Implements
        battery-aware activation to prevent activation under low power conditions.
        
        Returns:
            bool: True if activation was successful, False otherwise.
            
        Examples:
            ```python
            if mgr.activate_node():
                print("Node is now active")
            else:
                print("Activation failed or conditions not met")
            ```
        """
        try:
            # Check if node is already active
            if self.node.active:
                self.node.get_logger().debug("Node is already active")
                return True
                
            # Check battery level
            battery_level = self.get_battery_level()
            if battery_level < self.battery_recovery_threshold:
                self.node.get_logger().warn(f"Battery too low ({battery_level:.1f}%) for activation")
                return False
                
            self.node.get_logger().info("Triggering node activation...")
            result = self.node.trigger_activate()
            
            if result == TransitionCallbackReturn.SUCCESS:
                self.node.get_logger().info("Node activation successful")
                return True
            else:
                self.node.get_logger().error("Node activation failed")
                return False
                
        except Exception as e:
            self.node.get_logger().error(f"Error during activation: {e}")
            return False
    
    def deactivate_node(self):
        """
        Deactivate the managed lifecycle node.
        
        Triggers the deactivation transition to put the node into inactive
        state while preserving configuration. Provides appropriate logging
        and error handling for the transition.
        
        Returns:
            bool: True if deactivation was successful, False otherwise.
            
        Examples:
            ```python
            # Gracefully deactivate for maintenance
            if mgr.deactivate_node():
                print("Node deactivated for maintenance")
            ```
        """
        try:
            # Check if node is already inactive
            if not self.node.active:
                self.node.get_logger().info("Node is already inactive")
                return True
                
            self.node.get_logger().info("Triggering node deactivation...")
            result = self.node.trigger_deactivate()
            
            if result == TransitionCallbackReturn.SUCCESS:
                self.node.get_logger().info("Node deactivation successful")
                return True
            else:
                self.node.get_logger().error("Node deactivation failed")
                return False
                
        except Exception as e:
            self.node.get_logger().error(f"Error during deactivation: {e}")
            return False
    
    def cleanup_node(self):
        """
        Cleanup the managed lifecycle node configuration.
        
        Triggers the cleanup transition to return the node to unconfigured
        state. Removes configuration markers and handles the transition
        with appropriate error handling and logging.
        
        Returns:
            bool: True if cleanup was successful, False otherwise.
            
        Examples:
            ```python
            # Clean up for reconfiguration
            if mgr.cleanup_node():
                print("Node cleaned up, ready for reconfiguration")
            ```
        """
        try:
            self.node.get_logger().info("Triggering node cleanup...")
            result = self.node.trigger_cleanup()
            
            if result == TransitionCallbackReturn.SUCCESS:
                self.node.get_logger().info("Node cleanup successful")
                # Mark as not configured anymore
                if hasattr(self.node, '_was_configured'):
                    delattr(self.node, '_was_configured')
                return True
            else:
                self.node.get_logger().error("Node cleanup failed")
                return False
                
        except Exception as e:
            self.node.get_logger().error(f"Error during cleanup: {e}")
            return False
    
    def shutdown_node(self):
        """
        Shutdown the managed lifecycle node completely.
        
        Triggers the shutdown transition and stops all lifecycle management.
        This is a terminal operation that prepares the node for final
        destruction and system shutdown.
        
        Returns:
            bool: True if shutdown was successful, False otherwise.
            
        Examples:
            ```python
            # Graceful system shutdown
            mgr.shutdown_node()
            print("Node shut down completely")
            ```
        """
        try:
            self.stop_auto_management()
            self.node.get_logger().info("Triggering node shutdown...")
            result = self.node.trigger_shutdown()
            
            if result == TransitionCallbackReturn.SUCCESS:
                self.node.get_logger().info("Node shutdown successful")
                return True
            else:
                self.node.get_logger().error("Node shutdown failed")
                return False
                
        except Exception as e:
            self.node.get_logger().error(f"Error during shutdown: {e}")
            return False
    
    def restart_node(self):
        """
        Restart the managed lifecycle node completely.
        
        Performs a complete restart sequence including deactivation, cleanup,
        configuration, and activation. Includes appropriate delays between
        transitions to ensure proper state changes.
        
        Returns:
            bool: True if restart was successful, False if any step failed.
            
        Examples:
            ```python
            # Full node restart for error recovery
            if mgr.restart_node():
                print("Node restarted successfully")
            else:
                print("Restart failed at some step")
            ```
        """
        self.node.get_logger().info("Restarting node...")
        
        # Deactivate if active
        if self.node.active:
            if not self.deactivate_node():
                return False
            time.sleep(0.5)
        
        # Always do cleanup
        if not self.cleanup_node():
            return False
        time.sleep(0.5)
        
        # Configure
        if not self.configure_node():
            return False
        time.sleep(0.5)
        
        # Activate
        if not self.activate_node():
            return False
            
        self.node.get_logger().info("Node restart completed successfully")
        return True
    
    def configure_thresholds(self, battery_low=None, battery_recovery=None):
        """
        Configure battery thresholds for lifecycle management decisions.
        
        Updates the battery level thresholds used for automatic lifecycle
        management including when to enter low-power mode and when to
        recover to active operation.
        
        Args:
            battery_low (float, optional): Battery level triggering low-power
                                          mode. If None, current value is kept.
            battery_recovery (float, optional): Battery level required for
                                               returning to active operation.
                                               If None, current value is kept.
                                               
        Examples:
            ```python
            # Set conservative thresholds
            mgr.configure_thresholds(battery_low=15.0, battery_recovery=25.0)
            
            # Update only low threshold
            mgr.configure_thresholds(battery_low=10.0)
            ```
        """
        if battery_low is not None:
            self.battery_low_threshold = battery_low
            
        if battery_recovery is not None:
            self.battery_recovery_threshold = battery_recovery
            
        self.node.get_logger().info(
            f"Battery thresholds updated: low={self.battery_low_threshold}%, "
            f"recovery={self.battery_recovery_threshold}%"
        )
    
    def set_auto_management_flags(self, auto_configure=None, auto_activate=None, 
                                 battery_aware=None, auto_recovery=None):
        """
        Configure automatic management behavior policies.
        
        Updates the flags that control various aspects of automatic lifecycle
        management including startup behavior, battery awareness, and recovery
        policies. Only specified parameters are updated.
        
        Args:
            auto_configure (bool, optional): Whether to auto-configure on startup.
            auto_activate (bool, optional): Whether to auto-activate after config.
            battery_aware (bool, optional): Whether to use battery-aware deactivation.
            auto_recovery (bool, optional): Whether to automatically recover failures.
            
        Examples:
            ```python
            # Enable all automatic features
            mgr.set_auto_management_flags(
                auto_configure=True,
                auto_activate=True,
                battery_aware=True,
                auto_recovery=True
            )
            
            # Disable only auto-activation
            mgr.set_auto_management_flags(auto_activate=False)
            ```
        """
        if auto_configure is not None:
            self.auto_configure_on_start = auto_configure
            
        if auto_activate is not None:
            self.auto_activate_after_configure = auto_activate
            
        if battery_aware is not None:
            self.battery_aware_deactivation = battery_aware
            
        if auto_recovery is not None:
            self.auto_recovery = auto_recovery
            
        self.node.get_logger().info(
            f"Auto-management flags updated: configure={self.auto_configure_on_start}, "
            f"activate={self.auto_activate_after_configure}, "
            f"battery_aware={self.battery_aware_deactivation}, "
            f"recovery={self.auto_recovery}"
        )
    
    def get_lifecycle_status(self):
        """
        Get comprehensive lifecycle and management status information.
        
        Returns a dictionary containing complete status information about
        the managed node including lifecycle state, battery level, management
        configuration, and threshold settings.
        
        Returns:
            dict: Status dictionary containing:
                - active (bool): Whether node is currently active
                - battery_level (float): Current battery percentage
                - auto_manage (bool): Whether auto-management is enabled
                - monitoring_active (bool): Whether monitoring is running
                - battery_low_threshold (float): Low battery threshold
                - battery_recovery_threshold (float): Recovery threshold
                
        Examples:
            ```python
            status = mgr.get_lifecycle_status()
            print(f"Node active: {status['active']}")
            print(f"Battery: {status['battery_level']:.1f}%")
            print(f"Auto-manage: {status['auto_manage']}")
            ```
        """
        return {
            'active': self.node.active,
            'battery_level': self.get_battery_level(),
            'auto_manage': self.auto_manage,
            'monitoring_active': self.monitoring_active,
            'battery_low_threshold': self.battery_low_threshold,
            'battery_recovery_threshold': self.battery_recovery_threshold
        }