from rclpy.lifecycle import State, TransitionCallbackReturn
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition
import threading
import time

class LifecycleManager:
    """
    Manages the lifecycle of a ROS 2 lifecycle node independently.
    
    Features:
    - Automatic state transitions based on conditions
    - Battery-aware lifecycle management
    - Self-monitoring and recovery
    - Event-driven state changes
    """
    
    def __init__(self, node):
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
        """Start automatic lifecycle management"""
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
        """Stop automatic lifecycle management"""
        self.auto_manage = False
        self.monitoring_active = False
        
        if hasattr(self, 'monitor_timer'):
            self.monitor_timer.cancel()
            
        self.node.get_logger().info("Lifecycle auto-management stopped")
    
    def initial_setup(self):
        """Perform initial configuration and activation"""
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
        """Monitor and manage lifecycle state based on conditions"""
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
        """Get current battery level"""
        try:
            if hasattr(self.node, 'battery_manager') and hasattr(self.node.battery_manager, 'battery'):
                return self.node.battery_manager.battery.current_level
            return 100.0  # Default if no battery manager
        except Exception:
            return 100.0
    
    def configure_node(self):
        """Configure the node"""
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
        """Activate the node"""
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
        """Deactivate the node"""
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
        """Cleanup the node"""
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
        """Shutdown the node"""
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
        """Restart the node (deactivate -> cleanup -> configure -> activate)"""
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
        """Configure battery thresholds for lifecycle management"""
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
        """Configure auto-management behavior"""
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
        """Get comprehensive lifecycle status"""
        return {
            'active': self.node.active,
            'battery_level': self.get_battery_level(),
            'auto_manage': self.auto_manage,
            'monitoring_active': self.monitoring_active,
            'battery_low_threshold': self.battery_low_threshold,
            'battery_recovery_threshold': self.battery_recovery_threshold
        }