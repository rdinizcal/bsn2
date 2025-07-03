# /ros_ws/src/adaptation/adaptation/components/data_manager.py
import rclpy

class DataManager:
    """Manages data collection from DataAccess service"""
    
    def __init__(self, engine):
        self.engine = engine
        
    def collect_data(self):
        """Collect QoS and context data"""
        self.reset_strategy_values()
        self.collect_qos_data()
        self.collect_context_data()
        
    def reset_strategy_values(self):
        """Reset temporary values in strategy"""
        for key in self.engine.strategy:
            if key.startswith("CTX_"):
                self.engine.strategy[key] = 0.0
            if key.startswith(self.engine.get_prefix()):
                self.engine.strategy[key] = 0.0
                
    def collect_qos_data(self):
        """Collect QoS data (cost, reliability, etc.)"""
        from bsn_interfaces.srv import DataAccessRequest
        
        request = DataAccessRequest.Request()
        request.name = '/engine'
        request.query = f"all:{self.engine.qos_attribute}:{self.engine.info_quant}"
        
        future = self.engine.communication_manager.data_access_client.call_async(request)
        rclpy.spin_until_future_complete(self.engine, future)
        
        response = future.result()
        if response is None or response.content == "":
            self.engine.get_logger().error(f"Empty {self.engine.qos_attribute} data received")
            return
            
        self.parse_qos_data(response.content)
        
    def collect_context_data(self):
        """Collect context data (activation status)"""
        from bsn_interfaces.srv import DataAccessRequest
        
        request = DataAccessRequest.Request()
        request.name = '/engine'
        request.query = "all:event:1"
        
        future = self.engine.communication_manager.data_access_client.call_async(request)
        rclpy.spin_until_future_complete(self.engine, future)
        
        response = future.result()
        if response is None or response.content == "":
            self.engine.get_logger().error("Empty context data received")
            return
            
        self.parse_context_data(response.content)
        
    def parse_qos_data(self, content):
        """Parse QoS data and update strategy"""
        prefix = self.engine.get_prefix()
        
        for pair in content.split(';'):
            if not pair:
                continue
                
            parts = pair.split(':')
            if len(parts) != 2:
                continue
                
            component = self._format_component_name(parts[0])
            value = float(parts[1])
            
            key = f"{prefix}{component}"
            if key in self.engine.strategy:
                self.engine.strategy[key] = value
                self.engine.get_logger().debug(f"Updated {key} = {value}")
                
    def parse_context_data(self, content):
        """Parse context data and update strategy"""
        prefix = self.engine.get_prefix()
        
        for pair in content.split(';'):
            if not pair:
                continue
                
            parts = pair.split(':')
            if len(parts) != 2:
                continue
                
            component = self._format_component_name(parts[0])
            status = parts[1]
            
            if component != "G4_T1":  # Normal components
                self.engine.strategy[f"CTX_{component}"] = 1.0
                
                if status == "deactivate":
                    self.engine.strategy[f"{prefix}{component}"] = 0.0
                    self.engine.deactivated_components[f"{prefix}{component}"] = 1
                    self.engine.get_logger().warn(f"{component} was deactivated")
            else:  # Central hub special case
                if status == "activate":
                    self.engine.strategy[f"CTX_{component}"] = 1.0
                    self.engine.deactivated_components[f"{prefix}{component}"] = 0
                else:
                    self.engine.strategy[f"CTX_{component}"] = 0.0
                    self.engine.deactivated_components[f"{prefix}{component}"] = 1
                    
    def _format_component_name(self, component):
        """Format component name consistently"""
        component = component.upper()
        if component.startswith('/'):
            component = component[1:]
        
        # Insert underscore between G and T
        if 'T' in component and '_T' not in component:
            t_pos = component.find('T')
            component = component[:t_pos] + '_' + component[t_pos:]
            
        return component