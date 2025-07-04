"""
Parameter adaptation routing for Body Sensor Network components.

This module provides component registration and adaptation command routing
capabilities for the BSN system. It manages dynamic registration of system
components and routes adaptation commands to appropriate targets.
"""

import rclpy
from rclpy.node import Node
from bsn_interfaces.msg import AdaptationCommand
from bsn_interfaces.srv import EffectorRegister
import threading
import time


class ParamAdapter(Node):
    """
    Parameter adaptation routing component for BSN system.
    
    This class manages component registration and routes adaptation commands
    to appropriate system components. It provides a centralized adaptation
    infrastructure that allows components to register themselves and receive
    targeted configuration changes during runtime.
    
    The adapter maintains a registry of active components and creates dedicated
    communication channels for each registered component to ensure reliable
    adaptation command delivery.
    
    Attributes:
        registered_components (dict): Registry of active components with timestamps.
        _component_publishers (dict): Publishers for component-specific topics.
        check_interval (float): Interval for checking component staleness.
        debug (bool): Whether debug logging is enabled.
        register_service: ROS service for component registration.
        command_sub: Subscriber for incoming adaptation commands.
        
    Examples:
        Basic usage:
        ```python
        import rclpy
        from system_monitor.param_adapter import ParamAdapter
        
        rclpy.init()
        adapter = ParamAdapter()
        rclpy.spin(adapter)
        ```
        
        Component registration via service:
        ```python
        # From a component
        client = node.create_client(EffectorRegister, 'EffectorRegister')
        request = EffectorRegister.Request()
        request.name = "my_component"
        request.connection = True
        response = client.call(request)
        ```
    """

    def __init__(self):
        """
        Initialize the parameter adapter.
        
        Sets up component registration service, adaptation command subscription,
        and periodic component checking. Configures adapter parameters for
        staleness detection and debug logging.
        """
        super().__init__('param_adapter')
        self.get_logger().info("Starting ParamAdapter")
        
        # Track registered components
        self.registered_components = {}
        self._component_publishers = {}  # Changed variable name to avoid property conflict
        
        # Configure adapter parameters
        self.declare_parameter('check_interval', 10.0)  # Interval to check for stale components
        self.declare_parameter('debug_level', False)    # Enable debug logging
        
        # Get parameters
        self.check_interval = self.get_parameter('check_interval').value
        self.debug = self.get_parameter('debug_level').value
        
        # Service for components to register themselves
        self.register_service = self.create_service(
            EffectorRegister, 
            'EffectorRegister', 
            self.module_connect
        )
        
        # Subscribe to adaptation commands
        self.command_sub = self.create_subscription(
            AdaptationCommand,
            'reconfigure',
            self.receive_adaptation_command,
            10
        )
        
        # Start component check timer
        self.create_timer(self.check_interval, self.check_components)
        
        self.get_logger().info("ParamAdapter initialized and ready to register components")
    
    def module_connect(self, request, response):
        """
        Handle component registration and deregistration requests.
        
        Processes service requests from components wanting to register or
        deregister from the adaptation system. Creates dedicated publishers
        for registered components and maintains registration timestamps.
        
        Args:
            request (EffectorRegister.Request): Service request containing
                component name and connection flag (True=register, False=deregister).
            response (EffectorRegister.Response): Service response to populate
                with acknowledgment status.
                
        Returns:
            EffectorRegister.Response: Response with ack field set to True
                for successful operations, False for errors.
        """
        try:
            component_name = request.name
            
            if request.connection:
                # Register component
                if component_name not in self.registered_components:
                    # Create publisher for this component's specific adaptation channel
                    pub = self.create_publisher(
                        AdaptationCommand,
                        f'reconfigure_{component_name}',
                        10
                    )
                    self._component_publishers[component_name] = pub  # Updated variable name
                    
                    # Track when component was registered
                    self.registered_components[component_name] = {
                        'registered_at': time.time(),
                        'last_command': None  # Will track last command sent
                    }
                    
                    self.get_logger().info(f"Component registered: {component_name}")
                else:
                    # Update registration timestamp if already registered
                    self.registered_components[component_name]['registered_at'] = time.time()
                    self.get_logger().info(f"Component registration refreshed: {component_name}")
                
                response.ack = True
                
            else:
                # Deregister component
                if component_name in self.registered_components:
                    # Remove component tracking
                    self.registered_components.pop(component_name)
                    
                    # Keep publisher for potential reconnection
                    self.get_logger().info(f"Component deregistered: {component_name}")
                    response.ack = True
                else:
                    self.get_logger().warn(f"Attempted to deregister unknown component: {component_name}")
                    response.ack = False
                    
        except Exception as e:
            self.get_logger().error(f"Error in module_connect: {e}")
            response.ack = False
            
        return response

    def receive_adaptation_command(self, msg):
        """
        Process and route adaptation commands to target components.
        
        Receives adaptation commands from the enactor and routes them to
        the appropriate registered components using dedicated communication
        channels. Tracks command delivery and provides error handling for
        unknown targets.
        
        Args:
            msg (AdaptationCommand): Adaptation command containing source,
                target component name, and action to perform.
        """
        target = msg.target
        
        if target in self._component_publishers:  # Updated variable name
            # Forward command to component's specific topic
            self._component_publishers[target].publish(msg)  # Updated variable name
            
            # Update tracking
            if target in self.registered_components:
                self.registered_components[target]['last_command'] = {
                    'timestamp': time.time(),
                    'action': msg.action
                }
            
            if self.debug:
                self.get_logger().debug(f"Command routed to {target}: {msg.action}")
        else:
            self.get_logger().warn(f"Received command for unknown component: {target}")
            
            # Check if target has a similar name (might be naming inconsistency)
            for component in self.registered_components:
                if target.lower() in component.lower() or component.lower() in target.lower():
                    self.get_logger().warn(f"Did you mean '{component}' instead of '{target}'?")

    def check_components(self):
        """
        Check for components that haven't been active recently.
        
        Periodically reviews registered components to identify those that
        haven't shown activity for extended periods. Logs stale components
        for system monitoring purposes but doesn't automatically remove them
        as they might still be functional.
        """
        current_time = time.time()
        stale_threshold = 300.0  # Consider component stale after 5 minutes
        
        stale_components = []
        
        for component, info in self.registered_components.items():
            time_since_registration = current_time - info['registered_at']
            
            # Check if component is stale (no recent activity)
            if time_since_registration > stale_threshold:
                stale_components.append(component)
                
        # Log stale components (but don't remove - they might still be active)
        if stale_components and self.debug:
            self.get_logger().info(f"These components were registered >5 minutes ago: {stale_components}")
            
        # Log active components summary
        if self.debug:
            self.get_logger().info(f"Active components: {list(self.registered_components.keys())}")


def main(args=None):
    """
    Main entry point for the parameter adapter node.
    
    Initializes ROS, creates the parameter adapter, and runs it in a
    multi-threaded executor to handle service requests and command
    routing concurrently.
    
    Args:
        args: Command line arguments passed to ROS initialization.
    """
    rclpy.init(args=args)
    node = ParamAdapter()
    
    # Spin in a separate thread
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    try:
        # Keep the main thread alive
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()