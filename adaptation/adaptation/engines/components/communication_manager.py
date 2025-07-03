# /ros_ws/src/adaptation/adaptation/components/communication_manager.py
from bsn_interfaces.msg import Strategy, Exception, EnergyStatus
from bsn_interfaces.srv import DataAccessRequest, EngineRequest

class CommunicationManager:
    """Handles all ROS communication for adaptation engines"""
    
    def __init__(self, engine):
        self.engine = engine
        self.setup_publishers()
        self.setup_subscribers()
        self.setup_services()
        self.setup_clients()
        
    def setup_publishers(self):
        """Set up all publishers"""
        self.strategy_pub = self.engine.create_publisher(
            Strategy, 'strategy', 10)
            
        self.energy_status_pub = self.engine.create_publisher(
            EnergyStatus, 'log_energy_status', 10)
            
    def setup_subscribers(self):
        """Set up all subscribers"""
        self.exception_sub = self.engine.create_subscription(
            Exception, 'exception', self.receive_exception, 1000)
            
    def setup_services(self):
        """Set up all services"""
        self.engine_service = self.engine.create_service(
            EngineRequest, 'EngineRequest', self.send_adaptation_parameter)
            
    def setup_clients(self):
        """Set up all service clients"""
        self.data_access_client = self.engine.create_client(
            DataAccessRequest, 'DataAccessRequest')
            
        # Wait for service availability
        while not self.data_access_client.wait_for_service(timeout_sec=1.0):
            self.engine.get_logger().info('DataAccessRequest service not available, waiting...')
            
    def receive_exception(self, msg):
        """Process exception messages from components"""
        with self.engine.lock:
            content = msg.content
            parts = content.split('=')
            
            if len(parts) != 2:
                self.engine.get_logger().error(f"Invalid exception format: {content}")
                return
                
            component = parts[0]
            value = int(parts[1])
            
            # Update priority through engine
            self.engine.update_component_priority(component, value)
            
    def publish_strategy(self):
        """Publish adaptation strategy"""
        content = self._format_strategy_content()
        
        msg = Strategy()
        msg.source = "/engine"
        msg.target = "/enactor"
        msg.content = content
        
        self.strategy_pub.publish(msg)
        self.engine.get_logger().info(f"Published strategy: {content}")
        
    def publish_energy_status(self, current_value):
        """Publish energy/cost status"""
        energy_msg = EnergyStatus()
        energy_msg.source = "/engine"
        energy_msg.content = f"global:{current_value:.6f};"
        
        # Add individual component values
        for key, value in self.engine.strategy.items():
            if key.startswith(self.engine.get_prefix()):
                component_name = self._format_component_name(key)
                energy_msg.content += f"/{component_name}:{value:.6f};"
        
        # Remove trailing semicolon
        if energy_msg.content.endswith(";"):
            energy_msg.content = energy_msg.content[:-1]
            
        self.energy_status_pub.publish(energy_msg)
        
    def send_adaptation_parameter(self, request, response):
        """Service callback for adaptation parameter requests"""
        response.content = self.engine.qos_attribute
        return response
        
    def _format_strategy_content(self):
        """Format strategy for publication"""
        content = ""
        components = {}
        
        for key, value in self.engine.strategy.items():
            if key.startswith(self.engine.get_prefix()):
                component_name = self._format_component_name(key)
                components[component_name] = value
        
        for component, value in components.items():
            content += f"/{component}:{value:.6f};"
        
        if content.endswith(";"):
            content = content[:-1]
            
        return content
        
    def _format_component_name(self, key):
        """Format component name for publication"""
        component_name = key[2:].lower()  # Remove prefix
        parts = component_name.split('_')
        if len(parts) >= 2:
            component_name = parts[0] + parts[1]
            if len(parts) > 2:
                component_name += "_" + "_".join(parts[2:])
        return component_name