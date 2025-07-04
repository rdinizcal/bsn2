"""
Runtime adaptation and reconfiguration handler for BSN components.

This module provides runtime adaptation capabilities for BSN system components,
enabling dynamic reconfiguration through the ParamAdapter service and processing
of adaptation commands for frequency changes and operational parameters.
"""

from rclpy.node import Node
from bsn_interfaces.msg import AdaptationCommand
from bsn_interfaces.srv import EffectorRegister


class AdaptationHandler:
    """
    Runtime adaptation and parameter reconfiguration handler for BSN components.
    
    This class enables BSN components to register with the parameter adaptation
    system and receive runtime configuration changes. It handles registration
    with the ParamAdapter, processes adaptation commands, and applies parameter
    changes to support dynamic system reconfiguration.
    
    The handler supports frequency adjustment, operational mode changes, and
    other runtime parameters that allow the BSN system to adapt to changing
    conditions, energy constraints, and performance requirements.
    
    Attributes:
        node: Reference to the ROS node using this adaptation handler.
        frequency (float): Current operational frequency after adaptation.
        adaptation_sub: ROS subscription for receiving adaptation commands.
        
    Examples:
        Basic adaptation setup:
        ```python
        from shared_components.adaptation_handler import AdaptationHandler
        
        # In your node's __init__ method
        self.adaptation_handler = AdaptationHandler(self)
        
        # Register with adaptation system
        self.adaptation_handler.register_with_effector()
        ```
        
        Manual adaptation command:
        ```python
        handler = AdaptationHandler(node)
        
        # Process adaptation command manually
        cmd = AdaptationCommand()
        cmd.action = "freq=2.0,mode=power_saving"
        handler.process_adaptation_command(cmd)
        ```
    """
    
    def __init__(self, node):
        """
        Initialize adaptation handler for a BSN component.
        
        Sets up the adaptation handler with reference to the parent node
        for logging and communication purposes. The handler is initialized
        but not registered until explicitly requested.
        
        Args:
            node: ROS node instance that will use this adaptation handler.
                  Must provide get_name() and get_logger() methods.
                  
        Examples:
            ```python
            # In sensor node
            self.adaptation_handler = AdaptationHandler(self)
            
            # In central hub
            self.adaptation_handler = AdaptationHandler(self)
            ```
        """
        self.node = node
    
    def register_with_effector(self):
        """
        Register this component with the ParamAdapter for runtime adaptation.
        
        Initiates the registration process with the system's ParamAdapter service
        to enable runtime parameter changes. Creates a service client, sends
        registration request, and sets up subscription for adaptation commands.
        
        The registration is asynchronous with callback handling to ensure
        proper setup of the adaptation communication channel.
        
        Examples:
            ```python
            # Register during node startup
            adaptation_handler.register_with_effector()
            
            # Check logs for "Successfully registered with ParamAdapter"
            ```
        """
        client = self.node.create_client(EffectorRegister, 'EffectorRegister')

        # Wait for service to become available
        while not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for EffectorRegister service...')

        # Create registration request
        request = EffectorRegister.Request()
        request.name = self.node.get_name()
        request.connection = True

        # Send request
        future = client.call_async(request)
        future.add_done_callback(self.registration_callback)

    def registration_callback(self, future):
        """
        Handle response from ParamAdapter registration service.
        
        Processes the registration response and sets up the adaptation command
        subscription if registration was successful. Handles registration
        failures with appropriate error logging.
        
        Args:
            future: ROS future object containing the service response with
                   acknowledgment status from the ParamAdapter.
                   
        The callback creates a subscription to receive adaptation commands
        on a component-specific topic if registration succeeds.
        """
        try:
            response = future.result()
            if response.ack:
                self.node.get_logger().info("Successfully registered with ParamAdapter")

                # Create subscription to receive adaptation commands
                self.adaptation_sub = self.node.create_subscription(
                    AdaptationCommand,
                    f'reconfigure_{self.node.get_name()}',
                    self.process_adaptation_command,
                    10
                )
            else:
                self.node.get_logger().error("Failed to register with ParamAdapter")
        except Exception as e:
            self.node.get_logger().error(f"Exception during registration: {e}")

    def process_adaptation_command(self, msg):
        """
        Process and apply adaptation commands from the ParamAdapter.
        
        Parses adaptation command messages and applies the requested parameter
        changes to the component. Supports multiple parameter formats and
        provides error handling for invalid values.
        
        Currently supported parameters:
        - freq: Changes operational frequency
        - mode: Operational mode changes (future extension)
        - priority: Priority level changes (future extension)
        
        Args:
            msg (AdaptationCommand): Adaptation command message containing
                                   source, target, and action parameters.
                                   Action format: "param1=value1,param2=value2"
                                   
        Examples:
            Command processing:
            ```python
            # Frequency change: action = "freq=2.5"
            # Multiple params: action = "freq=1.0,mode=power_saving"
            # Invalid value: action = "freq=invalid" (logged as error)
            ```
        """
        action = msg.action
        self.node.get_logger().info(f"Received adaptation command: {action}")

        # Parse action parameters (example: "freq=1.5,replicate=2")
        params = {}
        for param in action.split(','):
            if '=' in param:
                key, value = param.split('=')
                params[key.strip()] = value.strip()

        # Apply parameters
        if 'freq' in params:
            try:
                new_freq = float(params['freq'])
                self.node.get_logger().info(f"Changing frequency to {new_freq}Hz")
                # Set new parameter
                self.frequency = new_freq
            except ValueError:
                self.node.get_logger().error(f"Invalid frequency value: {params['freq']}")
