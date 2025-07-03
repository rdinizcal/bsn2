# /ros_ws/src/adaptation/adaptation/engines/reli_engine.py
from .components.base_engine import BaseEngine
from .components.formula_manager import FormulaManager
from .components.adaptation_planner import AdaptationPlanner

class ReliabilityEngine(BaseEngine):
    """Reliability adaptation engine with component-based architecture"""
    
    def __init__(self):
        super().__init__('reliability_engine', 'reliability')
        self.prefix = "R_"
        
        # Override default setpoint for reliability
        self.setpoint = 0.95  # Target reliability
        
        # Initialize components with reliability-specific implementations
        self.initialize_components()
        
        # Override with reliability-specific managers
        self.formula_manager = ReliabilityFormulaManager(self)
        self.adaptation_planner = ReliabilityAdaptationPlanner(self)
        
        # Start monitoring
        self.formula_manager.fetch_and_setup_formula()
        self.start_monitoring()
        
        self.get_logger().info(f"ReliabilityEngine initialized with setpoint {self.setpoint}")
        
    def get_prefix(self):
        """Return the prefix for this engine type"""
        return self.prefix
        
    def analyze(self):
        """Reliability-specific analysis phase"""
        current_reliability = self.calculate_qos()
        
        # Log current reliability
        self.get_logger().info(f"Current system reliability: {current_reliability:.4f}, target: {self.setpoint:.4f}")
        
        # Calculate error
        error = self.setpoint - current_reliability
        
        # Check if adaptation is needed
        if abs(error) > self.setpoint * self.tolerance:
            if self.cycles >= self.monitor_freq / self.actuation_freq:
                self.cycles = 0
                self.plan(current_reliability, error)
                
    def calculate_qos(self):
        """Calculate total system reliability"""
        # For reliability, we use a weighted average approach
        # In real implementation, you'd evaluate the actual formula
        total_weight = 0
        weighted_sum = 0
        
        for key, value in self.strategy.items():
            if key.startswith(self.prefix):
                component = key[2:]  # Remove prefix
                
                # Only include active components
                if self.strategy.get(f"CTX_{component}", 0) != 0:
                    weight = 1  # Equal weight for simplicity
                    weighted_sum += value * weight
                    total_weight += weight
                    
        if total_weight == 0:
            return 1.0  # Default to perfect reliability if no components
            
        return weighted_sum / total_weight
        
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

# Extend FormulaManager for reliability-specific initialization
class ReliabilityFormulaManager(FormulaManager):
    def initialize_strategy(self, terms):
        """Initialize reliability-specific strategy"""
        strategy = {}
        prefix = self.engine.get_prefix()
        
        for term in terms:
            if term.startswith(prefix):
                # Default reliability is 1.0 (perfect)
                strategy[term] = 1.0
            elif term.startswith("CTX_"):
                # Default context is inactive (0)
                strategy[term] = 0.0
            elif term.startswith("F_"):
                # Default factor is 1.0
                strategy[term] = 1.0
                
        return strategy
        
    def initialize_priority(self, terms):
        """Initialize reliability-specific priorities"""
        priority = {}
        prefix = self.engine.get_prefix()
        
        for term in terms:
            if term.startswith(prefix):
                priority[term[2:]] = 50  # Default middle priority
                
        return priority

# Extend AdaptationPlanner for reliability-specific planning
class ReliabilityAdaptationPlanner(AdaptationPlanner):
    def execute_planning_strategy(self, components, current_reliability, error):
        """Reliability-specific adaptation planning"""
        self.engine.get_logger().info(f"Planning reliability adaptation for error: {error:.4f}")
        
        # List to store potential solutions
        solutions = []
        
        # Try adapting components one at a time, starting with highest priority
        for component in components:
            # Save original strategy
            original_strategy = self.engine.strategy.copy()
            
            # Apply initial offset to all components
            for c in components:
                if error > 0:  # Need to increase reliability
                    self.engine.strategy[c] = current_reliability * (1 - self.engine.offset)
                else:  # Need to decrease reliability
                    self.engine.strategy[c] = min(1.0, current_reliability * (1 + self.engine.offset))
            
            # Calculate new reliability with offset
            r_new = self.engine.calculate_qos()
            
            # Save previous state for iteration
            prev_strategy = self.engine.strategy.copy()
            r_prev = 0
            
            # Adjust current component
            if error > 0:  # Need to increase reliability
                # Keep increasing reliability while improving and not at maximum
                while (r_new < self.engine.setpoint and 
                       r_prev < r_new and 
                       self.engine.strategy[component] > 0 and 
                       self.engine.strategy[component] < 1):
                    prev_strategy = self.engine.strategy.copy()
                    r_prev = r_new
                    self.engine.strategy[component] += self.engine.gain * error
                    if self.engine.strategy[component] > 1.0:
                        self.engine.strategy[component] = 1.0
                    r_new = self.engine.calculate_qos()
            else:  # Need to decrease reliability
                # Keep decreasing reliability while improving and not at minimum
                while (r_new > self.engine.setpoint and 
                       r_prev > r_new and 
                       self.engine.strategy[component] > 0 and 
                       self.engine.strategy[component] < 1):
                    prev_strategy = self.engine.strategy.copy()
                    r_prev = r_new
                    self.engine.strategy[component] += self.engine.gain * error  # Will be negative
                    if self.engine.strategy[component] < 0.0:
                        self.engine.strategy[component] = 0.0
                    r_new = self.engine.calculate_qos()
            
            # Restore best result from loop
            self.engine.strategy = prev_strategy
            r_new = self.engine.calculate_qos()
            
            # Try adjusting other components too
            for other_component in components:
                if other_component == component:
                    continue
                
                if error > 0:  # Need to increase reliability
                    while (r_new < self.engine.setpoint and 
                           r_prev < r_new and 
                           self.engine.strategy[other_component] > 0 and 
                           self.engine.strategy[other_component] < 1):
                        prev_strategy = self.engine.strategy.copy()
                        r_prev = r_new
                        self.engine.strategy[other_component] += self.engine.gain * error
                        if self.engine.strategy[other_component] > 1.0:
                            self.engine.strategy[other_component] = 1.0
                        r_new = self.engine.calculate_qos()
                else:  # Need to decrease reliability
                    while (r_new > self.engine.setpoint and 
                           r_prev > r_new and 
                           self.engine.strategy[other_component] > 0 and 
                           self.engine.strategy[other_component] < 1):
                        prev_strategy = self.engine.strategy.copy()
                        r_prev = r_new
                        self.engine.strategy[other_component] += self.engine.gain * error
                        if self.engine.strategy[other_component] < 0.0:
                            self.engine.strategy[other_component] = 0.0
                        r_new = self.engine.calculate_qos()
            
            # Restore best result
            self.engine.strategy = prev_strategy
            r_new = self.engine.calculate_qos()
            
            # Add this solution to list
            solutions.append((self.engine.strategy.copy(), r_new))
            
            # Restore original strategy for next iteration
            self.engine.strategy = original_strategy
        
        # Choose the best solution (closest to setpoint)
        best_solution = None
        best_distance = float('inf')
        
        for solution, r_value in solutions:
            distance = abs(r_value - self.engine.setpoint)
            tolerance_range = self.engine.setpoint * self.engine.tolerance
            
            if (distance < best_distance and 
                self.engine.setpoint * (1 - self.engine.tolerance) <= r_value <= 
                self.engine.setpoint * (1 + self.engine.tolerance)):
                best_distance = distance
                best_solution = solution
        
        if best_solution:
            # Apply best solution
            self.engine.strategy = best_solution
            self.engine.execute()
        else:
            self.engine.get_logger().warn("Could not find a suitable reliability adaptation")