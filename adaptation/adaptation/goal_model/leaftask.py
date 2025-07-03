from .task import Task
from .context import Context
from .property import Property
from .node import Node

class LeafTask(Task):
    """
    Represents a leaf task in the goal model hierarchy
    LeafTasks cannot have children and contain QoS properties
    """
    
    def __init__(self, task_id: str = "", description: str = "", 
                 context: Context = None, cost: Property = None, 
                 reliability: Property = None, frequency: Property = None):
        """
        Initialize LeafTask
        
        Args:
            task_id: Unique identifier for the task
            description: Human-readable description
            context: Context condition for task execution
            cost: Cost property (energy consumption)
            reliability: Reliability property
            frequency: Frequency property
        """
        super().__init__(task_id, description)
        
        self._context = context if context is not None else Context()
        self._cost = cost if cost is not None else Property()
        self._reliability = reliability if reliability is not None else Property()
        self._frequency = frequency if frequency is not None else Property()
    
    def __del__(self):
        """Destructor"""
        pass
    
    def __copy__(self):
        """Copy constructor"""
        new_task = LeafTask(
            self._id, 
            self._description,
            self._context.__copy__(),
            self._cost.__copy__(),
            self._reliability.__copy__(),
            self._frequency.__copy__()
        )
        return new_task
    
    def __deepcopy__(self, memo):
        """Deep copy constructor"""
        return self.__copy__()
    
    # Context management
    def set_context(self, context: Context):
        """Set context"""
        self._context = context
    
    def get_context(self) -> Context:
        """Get context"""
        return self._context
    
    # Cost management
    def set_cost(self, cost: Property):
        """Set cost property"""
        self._cost = cost
    
    def get_cost(self) -> Property:
        """Get cost property"""
        return self._cost
    
    # Reliability management
    def set_reliability(self, reliability: Property):
        """Set reliability property"""
        self._reliability = reliability
    
    def get_reliability(self) -> Property:
        """Get reliability property"""
        return self._reliability
    
    # Frequency management
    def set_frequency(self, frequency: Property):
        """Set frequency property"""
        self._frequency = frequency
    
    def get_frequency(self) -> Property:
        """Get frequency property"""
        return self._frequency
    
    def add_child(self, node: Node):
        """Override to prevent adding children"""
        raise ValueError("Leaf Tasks cannot contain children")