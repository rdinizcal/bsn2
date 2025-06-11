#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from bsn_interfaces.msg import AdaptationCommand
from bsn_interfaces.srv import EffectorRegister
from rclpy.callback_groups import ReentrantCallbackGroup

class ParamAdapter(Node):
    """
    Adaptation parameter routing component for BSN
    
    Receives component registrations via service
    Receives adaptation commands from enactor
    Routes commands to their target components
    """
    
    def __init__(self):
        super().__init__('param_adapter')
        self.get_logger().info("Starting ParamAdapter")
        
        # Use a callback group that allows concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # Dictionary of registered components and their publishers
        self.target_publishers = {}
        
        # Create the registration service
        self.register_service = self.create_service(
            EffectorRegister, 
            'EffectorRegister',
            self.module_connect,
            callback_group=self.callback_group
        )
        
        # Subscribe to adaptation commands
        self.adapt_sub = self.create_subscription(
            AdaptationCommand,
            'reconfigure',
            self.receive_adaptation_command,
            10,
            callback_group=self.callback_group
        )
        
        self.get_logger().info("ParamAdapter initialized and ready for component registrations")
    
    def module_connect(self, request, response):
        """Handle component registration/deregistration"""
        try:
            component_name = request.name
            
            if request.connection:  # Register component
                # Create a dedicated publisher for this component
                publisher = self.create_publisher(
                    AdaptationCommand,
                    f'reconfigure_{component_name}',
                    1
                )
                self.target_publishers[component_name] = publisher
                self.get_logger().info(f"Registered component: {component_name}")
                
            else:  # Deregister component
                if component_name in self.target_publishers:
                    # Remove the publisher
                    del self.target_publishers[component_name]
                    self.get_logger().info(f"Deregistered component: {component_name}")
            
            response.ack = True
            
        except Exception as e:
            self.get_logger().error(f"Error in module_connect: {str(e)}")
            response.ack = False
            
        return response
    
    def receive_adaptation_command(self, msg):
        """Handle and route adaptation commands"""
        target = msg.target
        
        if target in self.target_publishers:
            # Forward the command to the specific component
            self.target_publishers[target].publish(msg)
            self.get_logger().debug(f"Forwarded command to {target}: {msg.action}")
        else:
            self.get_logger().warn(f"Target not found for adaptation command: {target}")

def main(args=None):
    rclpy.init(args=args)
    adapter = ParamAdapter()
    
    try:
        rclpy.spin(adapter)
    except KeyboardInterrupt:
        pass
    finally:
        adapter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()