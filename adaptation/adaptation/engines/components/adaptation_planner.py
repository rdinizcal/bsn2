# /ros_ws/src/adaptation/adaptation/components/adaptation_planner.py

class AdaptationPlanner:
    """Plans adaptation strategies based on QoS requirements"""
    
    def __init__(self, engine):
        self.engine = engine
        
    def plan_adaptation(self, current_value, error):
        """Plan adaptation strategy - template method for different engines"""
        self.engine.get_logger().info(f"Planning adaptation for error: {error:.4f}")
        
        # Get adjustable components
        adjustable_components = self.get_adjustable_components()
        
        if not adjustable_components:
            self.engine.get_logger().warn("No components available for adaptation")
            return
            
        # Plan using specific strategy (to be overridden by subclasses)
        self.execute_planning_strategy(adjustable_components, current_value, error)
        
    def get_adjustable_components(self):
        """Get list of components that can be adjusted"""
        prefix = self.engine.get_prefix()
        adjustable = []
        
        for key in self.engine.strategy:
            if key.startswith(prefix):
                task = key[2:]  # Remove prefix
                
                # Only include active components
                if (self.engine.strategy.get(f"CTX_{task}", 0) != 0 and
                    self.engine.deactivated_components.get(key, 0) == 0):
                    adjustable.append(key)
                    
        return self.sort_by_priority(adjustable)
        
    def sort_by_priority(self, components):
        """Sort components by priority (higher first)"""
        with_priority = [(comp, self.engine.priority.get(comp[2:], 0)) for comp in components]
        with_priority.sort(key=lambda x: x[1], reverse=True)
        return [comp for comp, _ in with_priority]
        
    def execute_planning_strategy(self, components, current_value, error):
        """Execute specific planning strategy - to be overridden"""
        raise NotImplementedError("Subclasses must implement execute_planning_strategy()")