from .node import Node
from .goal import Goal

class Task(Node):
    """
    Represents a task in the goal model hierarchy
    Tasks can contain other Tasks as children, but NOT Goals
    """
    
    def __init__(self, task_id: str = "", description: str = ""):
        """
        Initialize Task
        
        Args:
            task_id: Unique identifier for the task
            description: Human-readable description
        """
        super().__init__(task_id, description)
    
    def __del__(self):
        """Destructor"""
        pass
    
    def add_child_task(self, task: 'Task'):
        """Add a Task as child"""
        self.add_child(task)
    
    def add_child_goal(self, goal: Goal):
        """Add a Goal as child - NOT ALLOWED"""
        raise ValueError("Tasks cannot contain goals as children")