"""
Property.py - Property class for goal model
"""

class Property:
    """
    Represents a property with ID and numeric value (cost, reliability, frequency)
    """
    
    def __init__(self, prop_id: str = "", value: float = 0.0):
        """
        Initialize Property
        
        Args:
            prop_id: Unique identifier for the property
            value: Numeric value of the property
        """
        self._id = prop_id
        self._value = value
    
    def __del__(self):
        """Destructor"""
        pass
    
    def __copy__(self):
        """Copy constructor"""
        return Property(self._id, self._value)
    
    def __deepcopy__(self, memo):
        """Deep copy constructor"""
        return self.__copy__()
    
    def __eq__(self, other: 'Property') -> bool:
        """Equality operator"""
        if not isinstance(other, Property):
            return False
        return self._id == other._id
    
    # Getters and Setters
    def set_id(self, prop_id: str):
        """Set property ID"""
        self._id = prop_id
    
    def get_id(self) -> str:
        """Get property ID"""
        return self._id
    
    def set_value(self, value: float):
        """Set property value"""
        self._value = value
    
    def get_value(self) -> float:
        """Get property value"""
        return self._value