# /ros_ws/src/adaptation/adaptation/components/formula_manager.py
import re
import rclpy

class FormulaManager:
    """Manages formula fetching, parsing, and strategy initialization"""
    
    def __init__(self, engine):
        self.engine = engine
        
    def fetch_and_setup_formula(self):
        """Fetch formula from DataAccess and set up model"""
        formula_str = self.fetch_formula(self.engine.qos_attribute)
        if not formula_str:
            self.engine.get_logger().warn("Empty formula received, retrying...")
            return False
        
        self.setup_formula(formula_str)
        return True
        
    def fetch_formula(self, name):
        """Fetch formula from DataAccess service"""
        from bsn_interfaces.srv import DataAccessRequest
        
        request = DataAccessRequest.Request()
        request.name = '/engine'
        request.query = f"{name}_formula"
        
        future = self.engine.communication_manager.data_access_client.call_async(request)
        rclpy.spin_until_future_complete(self.engine, future)
        
        response = future.result()
        if response is None or response.content == "":
            self.engine.get_logger().error("Empty formula string received")
            return ""
            
        return response.content
        
    def setup_formula(self, formula_str):
        """Set up model, strategy and priority based on formula"""
        self.engine.get_logger().info(f"Setting up formula: {formula_str}")
        
        self.engine.target_system_model = formula_str
        terms = self.extract_terms(formula_str)
        
        self.engine.strategy = self.initialize_strategy(terms)
        self.engine.priority = self.initialize_priority(terms)
        
        # Initialize deactivated components
        prefix = self.engine.get_prefix()
        for term in terms:
            if term.startswith(prefix):
                self.engine.deactivated_components[term] = 0
                
    def extract_terms(self, formula_str):
        """Extract terms from formula string"""
        pattern = r'[A-Z]+_[A-Z0-9_]+'
        terms = set(re.findall(pattern, formula_str))
        return list(terms)
        
    def initialize_strategy(self, terms):
        """Initialize strategy with default values - to be overridden"""
        return {}
        
    def initialize_priority(self, terms):
        """Initialize priority for terms - to be overridden"""
        return {}