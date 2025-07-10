import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import threading
import time
from collections import deque, defaultdict
import math
from .enactor import Enactor
class Controller(Enactor):
    """
    Concrete implementation of Enactor (equivalent to Controller in C++)
    Implements the specific adaptation strategies
    """
    
    def __init__(self):
        super().__init__('enactor')
        
        # Additional parameters specific to Controller
        self.declare_parameter('kp', 0.5)  # Proportional gain for control
        self.kp = self.get_parameter('kp').value
        
        # Individual KP values per component (matching C++ behavior)
        self.kp_individual = {}
        
        # Subscribe to events (specific to Controller)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.event_sub = self.create_subscription(
            Event, 'event', self.receive_event, qos_profile)
        
        self.get_logger().info(f'Controller initialized with kp={self.kp}')
    
    def receive_event(self, msg):
        """Process lifecycle events from components (matching C++ Controller logic)"""
        component = msg.source
        content = msg.content
        
        if content == "activate":
            # Initialize component data structures (matching C++ logic)
            self.invocations[component] = deque(maxlen=100)
            self.exception_buffer[component] = 0
            
            if self.adaptation_parameter == "reliability":
                self.r_curr[component] = 1.0
                self.r_ref[component] = 1.0
            else:
                self.c_curr[component] = 0.0
                self.c_ref[component] = 0.0
            
            # CRITICAL: Use frequency from event message (matching C++ behavior)
            self.freq[component] = msg.freq if hasattr(msg, 'freq') else 1.0
            self.kp_individual[component] = self.kp  # Individual KP per component
            self.replicate_task[component] = 1
            
            self.get_logger().info(
                f"Component {component} activated with freq={self.freq[component]}")
            
        elif content == "deactivate":
            # Remove all component data structures (matching C++ logic)
            attributes = ['invocations', 'exception_buffer', 'freq', 'r_curr', 
                         'c_curr', 'r_ref', 'c_ref', 'replicate_task', 'kp_individual']
            
            for attr in attributes:
                getattr(self, attr).pop(component, None)
            
            self.get_logger().info(f"Component {component} deactivated")
    
    def apply_reli_strategy(self, component):
        """Apply reliability strategy to component (matching C++ Controller logic)"""
        if component not in self.r_curr or component not in self.r_ref:
            return
        
        error = self.r_ref[component] - self.r_curr[component]
        
        # Check if error exceeds stability margin (matching C++ logic)
        stability_threshold = self.stability_margin * self.r_ref[component]
        
        if abs(error) > stability_threshold:
            # Increment exception buffer (matching C++ logic)
            if self.exception_buffer[component] < 0:
                self.exception_buffer[component] = 0
            else:
                self.exception_buffer[component] += 1
            
            # Calculate new frequency using proportional control (matching C++ logic)
            kp_component = self.kp_individual.get(component, self.kp)
            new_freq = self.freq[component] + ((kp_component / 100) * error)
            
            # Apply component-specific frequency limits (matching C++ logic)
            if component == "/g4t1":  # Central Hub
                min_freq = 0.1
                max_freq = float('inf')  # No upper limit for central hub
            else:  # Sensors
                min_freq = 0.1
                max_freq = 40.0
            
            if min_freq <= new_freq <= max_freq:
                self.freq[component] = new_freq
                self._send_adaptation_command(component, f"freq={new_freq:.2f}")
                
                self.get_logger().debug(
                    f"Adapted {component}: error={error:.4f}, new_freq={new_freq:.2f}")
        else:
            # Decrement exception buffer if error is within margin (matching C++ logic)
            if self.exception_buffer[component] > 0:
                self.exception_buffer[component] = 0
            else:
                self.exception_buffer[component] -= 1
        
        # Manage exceptions (matching C++ logic)
        if self.exception_buffer[component] > 4:
            self._publish_exception(component, "1")  # Positive exception
            self.exception_buffer[component] = 0
        elif self.exception_buffer[component] < -4:
            self._publish_exception(component, "-1")  # Negative exception
            self.exception_buffer[component] = 0
        
        # Clear invocations (matching C++ behavior)
        if component in self.invocations:
            self.invocations[component].clear()
    
    def apply_cost_strategy(self, component):
        """Apply cost strategy to component (matching C++ Controller logic)"""
        if component not in self.c_curr or component not in self.c_ref:
            return
        
        error = self.c_ref[component] - self.c_curr[component]
        
        # Check if error exceeds stability margin
        stability_threshold = self.stability_margin * abs(self.c_ref[component])
        
        if abs(error) > stability_threshold:
            # Increment exception buffer
            if self.exception_buffer[component] < 0:
                self.exception_buffer[component] = 0
            else:
                self.exception_buffer[component] += 1
            
            # Apply task replication adaptation (simplified cost strategy)
            kp_component = self.kp_individual.get(component, self.kp)
            new_replicate = max(1, self.replicate_task[component] - int((kp_component / 100) * error))
            
            # Constrain replication to reasonable range
            if new_replicate > 10:
                new_replicate = 10
            
            self.replicate_task[component] = new_replicate
            self._send_adaptation_command(component, f"replicate_collect={new_replicate}")
            
            self.get_logger().debug(
                f"Cost adapted {component}: error={error:.4f}, replicate={new_replicate}")
        else:
            # Decrement exception buffer if error is within margin
            if self.exception_buffer[component] > 0:
                self.exception_buffer[component] = 0
            else:
                self.exception_buffer[component] -= 1
        
        # Manage exceptions (same logic as reliability)
        if self.exception_buffer[component] > 4:
            self._publish_exception(component, "1")
            self.exception_buffer[component] = 0
        elif self.exception_buffer[component] < -4:
            self._publish_exception(component, "-1")
            self.exception_buffer[component] = 0
        
        # Clear invocations
        if component in self.invocations:
            self.invocations[component].clear()
    
    def _send_adaptation_command(self, component, action):
        """Send adaptation command to component via reconfigure topic"""
        command = AdaptationCommand()
        command.source = self.get_name()
        command.target = component
        command.action = action
        
        # Publish to reconfigure topic for ParamAdapter
        self.adapt.publish(command)
        
        self.get_logger().info(f"Sent adaptation to {component}: {action}")
    
    def _publish_exception(self, component, value):
        """Publish exception message (matching C++ format)"""
        exception_msg = ExceptionMsg()
        exception_msg.source = self.get_name()
        exception_msg.target = "/engine"
        exception_msg.content = f"{component}={value}"
        
        self.except_pub.publish(exception_msg)
        
        self.get_logger().warn(f"Published exception for {component}: {value}")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        # Create Controller instance (concrete implementation of Enactor)
        controller = Controller()
        
        # Use MultiThreadedExecutor for handling callbacks
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(controller)
        
        controller.get_logger().info("Controller node started")
        executor.spin()
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        if 'controller' in locals():
            controller.tear_down()
            controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()