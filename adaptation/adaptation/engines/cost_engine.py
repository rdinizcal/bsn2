# /ros_ws/src/adaptation/adaptation/engines/cost_engine.py
from .components.base_engine import BaseEngine

class CostEngine(BaseEngine):
    """Cost adaptation engine with component-based architecture"""
    
    def __init__(self):
        super().__init__('cost_engine', 'cost')
        self.prefix = "W_"
        
        # Initialize components
        self.initialize_components()
        
        # Start monitoring
        self.formula_manager.fetch_and_setup_formula()
        self.start_monitoring()
        
        self.get_logger().info(f"CostEngine initialized with setpoint {self.setpoint}")
        
    def get_prefix(self):
        """Return the prefix for this engine type"""
        return self.prefix
        
    def analyze(self):
        """Cost-specific analysis phase"""
        current_cost = self.calculate_qos()
        
        # Publish cost status
        self.communication_manager.publish_energy_status(current_cost)
        
        self.get_logger().info(f"Current system cost: {current_cost:.4f}, target: {self.setpoint:.4f}")
        
        # Calculate error
        error = self.setpoint - current_cost
        
        # Check if adaptation is needed
        if abs(error) > self.setpoint * self.tolerance:
            if self.cycles >= self.monitor_freq / self.actuation_freq:
                self.cycles = 0
                self.plan(current_cost, error)
                
    def calculate_qos(self):
        """Calculate total system cost"""
        total_cost = 0
        
        for key, value in self.strategy.items():
            if key.startswith(self.prefix):
                component = key[2:]  # Remove prefix
                
                # Only include active components
                if self.strategy.get(f"CTX_{component}", 0) != 0:
                    total_cost += value
                    
        return total_cost
        
    def update_component_priority(self, component, value):
        """Update component priority from exception"""
        # Format component name
        component = component.upper()
        if component.startswith('/'):
            component = component[1:]
            
        if 'T' in component and '_T' not in component:
            t_pos = component.find('T')
            component = component[:t_pos] + '_' + component[t_pos:]
            
        if component in self.priority:
            self.priority[component] += value
            self.priority[component] = max(0, min(100, self.priority[component]))
            self.get_logger().info(f"Updated priority for {component}: {self.priority[component]}")
        else:
            self.get_logger().error(f"Component not found: {component}")

# Extend FormulaManager for cost-specific initialization
class CostFormulaManager(FormulaManager):
    def initialize_strategy(self, terms):
        """Initialize cost-specific strategy"""
        strategy = {}
        prefix = self.engine.get_prefix()
        
        for term in terms:
            if term.startswith(prefix):
                strategy[term] = 0.0
            elif term.startswith("CTX_"):
                strategy[term] = 0.0
                
        return strategy
        
    def initialize_priority(self, terms):
        """Initialize cost-specific priorities"""
        priority = {}
        prefix = self.engine.get_prefix()
        
        for term in terms:
            if term.startswith(prefix):
                priority[term[2:]] = 50  # Default middle priority
                
        return priority

# Extend AdaptationPlanner for cost-specific planning
class CostAdaptationPlanner(AdaptationPlanner):
    def execute_planning_strategy(self, components, current_cost, error):
        """Cost-specific adaptation planning"""
        # Implement the cost adaptation algorithm from your original code
        # This would include the normalization, iteration, and solution selection logic
        pass