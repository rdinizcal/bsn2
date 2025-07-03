from .node import Node

class Goal(Node):
    """
    Represents a goal in the goal model hierarchy
    Goals can contain other Goals or Tasks as children
    """
    
    def __init__(self, goal_id: str = "", description: str = ""):
        """
        Initialize Goal
        
        Args:
            goal_id: Unique identifier for the goal
            description: Human-readable description
        """
        super().__init__(goal_id, description)
    
    def __del__(self):
        """Destructor"""
        pass