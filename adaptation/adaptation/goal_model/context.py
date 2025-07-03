"""
Context.py - Context class for goal model
"""

class Context:
    """
    Represents a context condition for task execution
    """
    
    def __init__(self, context_id: str = "", description: str = "", value: bool = False):
        """
        Initialize Context
        
        Args:
            context_id: Unique identifier for the context
            description: Human-readable description
            value: Boolean value of the context
        """
        self._id = context_id
        self._description = description
        self._value = value
    
    def __del__(self):
        """Destructor"""
        pass
    
    def __copy__(self):
        """Copy constructor"""
        return Context(self._id, self._description, self._value)
    
    def __deepcopy__(self, memo):
        """Deep copy constructor"""
        return self.__copy__()
    
    def __eq__(self, other: 'Context') -> bool:
        """Equality operator"""
        if not isinstance(other, Context):
            return False
        return self._id == other._id
    
    # Getters and Setters
    def set_id(self, context_id: str):
        """Set context ID"""
        self._id = context_id
    
    def get_id(self) -> str:
        """Get context ID"""
        return self._id
    
    def set_description(self, description: str):
        """Set context description"""
        self._description = description
    
    def get_description(self) -> str:
        """Get context description"""
        return self._description
    
    def set_value(self, value: bool):
        """Set context value"""
        self._value = value
    
    def get_value(self) -> bool:
        """Get context value"""
        return self._value