import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from bsn_interfaces.msg import Status, Event, Strategy, AdaptationCommand, Exception
from bsn_interfaces.srv import DataAccessRequest, EngineRequest
from std_msgs.msg import String
import time
from collections import deque
import math


class Enactor(Node):
    """
    Enactor component for BSN system in ROS2 Jazzy
    
    Receives adaptation strategies from engines
    Translates strategies into component-specific commands
    Sends adaptation commands to components via ParamAdapter
    """

    def __init__(self):
        super().__init__('enactor')
        self.get_logger().info("Starting BSN Enactor")
        
        # Declare parameters
        self.declare_parameter('frequency', 1.0)
        self.declare_parameter('kp', 0.5)  # Proportional gain for control
        self.declare_parameter('adaptation_parameter', 'reliability')
        self.declare_parameter('stability_margin', 0.02)
        
        # Get parameters
        self.frequency = self.get_parameter('frequency').value
        self.kp = self.get_parameter('kp').value
        self.adaptation_parameter = self.get_parameter('adaptation_parameter').value
        self.stability_margin = self.get_parameter('stability_margin').value
        
        # Internal state tracking
        self.cycles = 0
        self.invocations = {}  # Maps component name to deque of success/failure (1/0)
        self.exception_buffer = {}  # Track exceptions by component
        self.freq = {}  # Current frequency by component
        self.r_curr = {}  # Current reliability by component
        self.c_curr = {}  # Current cost by component
        self.r_ref = {}  # Reference reliability by component
        self.c_ref = {}  # Reference cost by component
        self.replicate_task = {}  # Task replication count by component
        
        # Publishers
        self.adapt_pub = self.create_publisher(
            AdaptationCommand, 'reconfigure', 10)
            
        self.log_adapt_pub = self.create_publisher(
            AdaptationCommand, 'log_adapt', 10)
            
        self.except_pub = self.create_publisher(
            Exception, 'exception', 10)
        
        # Subscribers
        self.event_sub = self.create_subscription(
            Event, 'event', self.receive_event, 10)
            
        self.strategy_sub = self.create_subscription(
            Strategy, 'strategy', self.receive_strategy, 10)
        
        # Create services
        self.engine_client = self.create_client(EngineRequest, 'EngineRequest')
        self.data_access_client = self.create_client(DataAccessRequest, 'DataAccessRequest')
        
        # Wait for service availability
        while not self.engine_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('EngineRequest service not available, waiting...')
        
        while not self.data_access_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('DataAccessRequest service not available, waiting...')
        
        # Get adaptation parameter from engine
        self.receive_adaptation_parameter()
        
        # Main processing timer
        self.create_timer(1.0/self.frequency, self.process_cycle)
        
        self.get_logger().info(f'Enactor initialized with {self.adaptation_parameter} parameter')
    
    def receive_event(self, msg):
        """Process events from components"""
        component = msg.source
        content = msg.content
        
        # Initialize component tracking if needed
        if component not in self.invocations:
            self.invocations[component] = deque(maxlen=100)  # Track last 100 operations
            self.exception_buffer[component] = 0
            self.freq[component] = 1.0
            self.r_curr[component] = 1.0
            self.c_curr[component] = 0.0
            self.r_ref[component] = 1.0
            self.c_ref[component] = 0.0
            self.replicate_task[component] = 1
        
        # Process activation/deactivation events
        if content == "activate":
            self.get_logger().info(f"Component {component} activated")
        elif content == "deactivate":
            self.get_logger().info(f"Component {component} deactivated")
            # Remove from tracking if deactivated
            if component in self.invocations:
                self.invocations.pop(component, None)
        
        # Process success/failure events
        elif content == "success":
            self.invocations[component].append(1)  # Success
        elif content == "fail":
            self.invocations[component].append(0)  # Failure
            self.exception_buffer[component] += 1
            
            # Report exception
            exception_msg = Exception()
            exception_msg.source = "enactor"
            exception_msg.component = component
            exception_msg.content = f"Component {component} failed"
            self.except_pub.publish(exception_msg)
    
    def receive_strategy(self, msg):
        """Process strategy messages from adaptation engines"""
        strategy_content = msg.content
        self.get_logger().info(f"Received strategy: {strategy_content}")
        
        # Parse strategy content (format: "component1=value1,component2=value2")
        strategy_pairs = strategy_content.split(',')
        for pair in strategy_pairs:
            if "=" not in pair:
                continue
                
            component_value = pair.split('=')
            if len(component_value) != 2:
                continue
                
            component = component_value[0]
            value = float(component_value[1])
            
            # Apply strategy based on adaptation parameter
            if self.adaptation_parameter == "reliability":
                self.r_ref[component] = value
                self.apply_reli_strategy(component)
            else:  # cost
                self.c_ref[component] = value
                self.apply_cost_strategy(component)
    
    def apply_reli_strategy(self, component):
        """Apply reliability strategy to component"""
        if component not in self.r_curr or component not in self.r_ref:
            return
        
        error = self.r_ref[component] - self.r_curr[component]
        
        # Only apply changes if error exceeds stability margin
        if abs(error) <= self.stability_margin:
            return
        
        # Apply frequency adaptation proportional to error
        new_freq = self.freq[component] + self.kp * error
        
        # Constrain frequency to reasonable range
        if new_freq < 0.1:
            new_freq = 0.1
        elif new_freq > 10.0:
            new_freq = 10.0
        
        self.freq[component] = new_freq
        
        # Create and publish adaptation command
        self.send_adaptation_command(component, f"freq={new_freq:.2f}")
    
    def apply_cost_strategy(self, component):
        """Apply cost strategy to component"""
        if component not in self.c_curr or component not in self.c_ref:
            return
        
        error = self.c_ref[component] - self.c_curr[component]
        
        # Only apply changes if error exceeds stability margin
        if abs(error) <= self.stability_margin:
            return
        
        # Apply task replication adaptation
        new_replicate = max(1, self.replicate_task[component] - int(self.kp * error))
        
        # Constrain replication to reasonable range
        if new_replicate > 10:
            new_replicate = 10
        
        self.replicate_task[component] = new_replicate
        
        # Create and publish adaptation command
        self.send_adaptation_command(component, f"replicate_collect={new_replicate}")
    
    def receive_adaptation_parameter(self):
        """Get adaptation parameter from engine"""
        request = EngineRequest.Request()
        
        future = self.engine_client.call_async(request)
        future.add_done_callback(self.adaptation_parameter_response)
    
    def adaptation_parameter_response(self, future):
        """Process engine response with adaptation parameter"""
        try:
            response = future.result()
            if response is None:
                self.get_logger().error("Empty response from engine")
                return
                
            parameter = response.content
            if parameter not in ["reliability", "cost"]:
                self.get_logger().error(f"Invalid adaptation parameter: {parameter}")
                return
                
            self.adaptation_parameter = parameter
            self.get_logger().info(f"Adaptation parameter set to: {parameter}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to get adaptation parameter: {str(e)}")
    
    def receive_status(self):
        """Query component status from DataAccess"""
        request = DataAccessRequest.Request()
        request.name = "enactor"
        
        if self.adaptation_parameter == "reliability":
            request.query = "all:reliability:"
        else:
            request.query = "all:cost:"
        
        future = self.data_access_client.call_async(request)
        future.add_done_callback(self.status_response)
    
    def status_response(self, future):
        """Process data access response with component status"""
        try:
            response = future.result()
            if response is None or response.content == "":
                self.get_logger().warn("Empty response from data access")
                return
                
            # Parse response (format: "component1:value1;component2:value2")
            pairs = response.content.split(';')
            
            for pair in pairs:
                component_value = pair.split(':')
                if len(component_value) < 2:
                    continue
                    
                component = component_value[0]
                value = float(component_value[1])
                
                # Update current values based on adaptation parameter
                if self.adaptation_parameter == "reliability":
                    self.r_curr[component] = value
                else:  # cost
                    self.c_curr[component] = value
                    
                # Apply strategy if component exists in references
                if component in self.r_ref and self.adaptation_parameter == "reliability":
                    self.apply_reli_strategy(component)
                elif component in self.c_ref and self.adaptation_parameter != "reliability":
                    self.apply_cost_strategy(component)
                    
        except Exception as e:
            self.get_logger().error(f"Failed to process status: {str(e)}")
    
    def send_adaptation_command(self, component, action):
        """Send adaptation command to component via reconfigure topic"""
        # Create command message
        command = AdaptationCommand()
        command.source = "enactor"
        command.target = component
        command.action = action
        
        # Publish to reconfigure topic for ParamAdapter
        self.adapt_pub.publish(command)
        
        # Also publish to log_adapt for logging
        self.log_adapt_pub.publish(command)
        
        self.get_logger().info(f"Sent adaptation to {component}: {action}")
    
    def process_cycle(self):
        """Main processing cycle"""
        self.cycles += 1
        
        # Only start adaptation after system has stabilized (60 cycles)
        if self.cycles > 60:
            # Get current status from data access
            self.receive_status()


def main(args=None):
    rclpy.init(args=args)
    enactor = Enactor()
    
    try:
        rclpy.spin(enactor)
    except KeyboardInterrupt:
        pass
    finally:
        enactor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()