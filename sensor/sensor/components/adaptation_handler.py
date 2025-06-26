from rclpy.node import Node
from bsn_interfaces.msg import AdaptationCommand
from bsn_interfaces.srv import EffectorRegister
class AdaptationHandler:
    """Manages battery state, charging, and energy reporting"""
    
    def __init__(self, node):
        self.node = node
    def register_with_effector(self):
        """Register this component with the ParamAdapter"""
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
        """Handle response from registration service"""
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
        """Process adaptation command from ParamAdapter"""
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
