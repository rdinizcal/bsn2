"""
Goal Model implementation for BSN system - ROS2 Jazzy migration
Node.py - Base class for all goal model elements
"""

from typing import List, Optional, Dict, Any
from abc import ABC
import copy

class Node(ABC):
    """
    Base class for all goal model elements (Goals, Tasks, LeafTasks)
    Represents a node in the goal tree hierarchy
    """
    
    def __init__(self, node_id: str = "", description: str = ""):
        """
        Initialize Node
        
        Args:
            node_id: Unique identifier for the node
            description: Human-readable description
        """
        self._id = node_id
        self._description = description
        self._children: List['Node'] = []
    
    def __del__(self):
        """Destructor"""
        pass
    
    def __copy__(self):
        """Copy constructor"""
        new_node = type(self)(self._id, self._description)
        new_node._children = [copy.copy(child) for child in self._children]
        return new_node
    
    def __deepcopy__(self, memo):
        """Deep copy constructor"""
        new_node = type(self)(self._id, self._description)
        new_node._children = [copy.deepcopy(child, memo) for child in self._children]
        return new_node
    
    def __eq__(self, other: 'Node') -> bool:
        """Equality operator"""
        if not isinstance(other, Node):
            return False
        return self._id == other._id and self._description == other._description
    
    def __str__(self) -> str:
        """String representation"""
        return f"bsn::goalmodel::Node({self._id}, {self._description}, has {len(self._children)} children)"
    
    def __repr__(self) -> str:
        """Repr representation"""
        return self.__str__()
    
    # Getters and Setters
    def set_id(self, node_id: str):
        """Set node ID"""
        self._id = node_id
    
    def get_id(self) -> str:
        """Get node ID"""
        return self._id
    
    def set_description(self, description: str):
        """Set node description"""
        self._description = description
    
    def get_description(self) -> str:
        """Get node description"""
        return self._description
    
    # Children management
    def has_children(self) -> bool:
        """Check if node has children"""
        return len(self._children) > 0
    
    def get_children(self) -> List['Node']:
        """Get list of children (copy)"""
        return self._children.copy()
    
    def add_child(self, node: 'Node'):
        """Add a child node"""
        self._children.append(node)
    
    def remove_child(self, node_id: str):
        """Remove child by ID"""
        pos = self._find_child(node_id)
        if pos >= 0:
            del self._children[pos]
        else:
            raise ValueError("Child Not Found")
    
    def get_child(self, node_id: str) -> 'Node':
        """Get child by ID"""
        pos = self._find_child(node_id)
        if pos >= 0:
            return self._children[pos]
        else:
            raise ValueError("Child Not Found")
    
    def _find_child(self, node_id: str) -> int:
        """Find child position by ID"""
        for i, child in enumerate(self._children):
            if child.get_id() == node_id:
                return i
        return -1