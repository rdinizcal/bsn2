#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from bsn_interfaces.msg import AdaptationCommand
from bsn_interfaces.srv import EffectorRegister
import threading
import time

class ParamAdapter(Node):
    """
    Parameter Adaptation routing component for BSN system in ROS2
    
    Key functions:
    - Receives component registrations via service
    - Receives adaptation commands from enactor
    - Routes commands to target components
    """

    def __init__(self):
        super().__init__('param_adapter')
        self.get_logger().info("Starting ParamAdapter")
        
        # Track registered components
        self.registered_components = {}
        self.publishers = {}
        
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
        """Handle component registration and deregistration"""
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
                    self.publishers[component_name] = pub
                    
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
        """Process adaptation command and route to target component"""
        target = msg.target
        
        if target in self.publishers:
            # Forward command to component's specific topic
            self.publishers[target].publish(msg)
            
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
        """Check for components that haven't been active for a while"""
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