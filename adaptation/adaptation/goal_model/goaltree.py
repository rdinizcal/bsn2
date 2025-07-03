from typing import Dict, List, Optional
from .node import Node
from .goal import Goal
from .task import Task
from .leaftask import LeafTask

class GoalTree:
    """
    Represents a complete goal tree with actor and nodes
    """
    
    def __init__(self, actor: str = ""):
        """
        Initialize GoalTree
        
        Args:
            actor: The actor/system that owns this goal tree
        """
        self._actor = actor
        self._nodes: Dict[str, Node] = {}
    
    def __del__(self):
        """Destructor"""
        pass
    
    def __copy__(self):
        """Copy constructor"""
        new_tree = GoalTree(self._actor)
        new_tree._nodes = {k: v.__copy__() for k, v in self._nodes.items()}
        return new_tree
    
    def __deepcopy__(self, memo):
        """Deep copy constructor"""
        new_tree = GoalTree(self._actor)
        new_tree._nodes = {k: v.__deepcopy__(memo) for k, v in self._nodes.items()}
        return new_tree
    
    def __eq__(self, other: 'GoalTree') -> bool:
        """Equality operator"""
        if not isinstance(other, GoalTree):
            return False
        return self._actor == other._actor
    
    # Actor management
    def set_actor(self, actor: str):
        """Set actor"""
        self._actor = actor
    
    def get_actor(self) -> str:
        """Get actor"""
        return self._actor
    
    # Private methods
    def _get_nodes(self) -> Dict[str, Node]:
        """Get nodes dictionary (private)"""
        return self._nodes
    
    def _add_children(self, children: List[Node]):
        """Add children recursively (private)"""
        for child in children:
            self._add_node(child)
    
    def _add_node(self, node: Node):
        """Add node to tree (private)"""
        self._nodes[node.get_id()] = node
        if node.has_children():
            self._add_children(node.get_children())
    
    # Public methods
    def add_root_goal(self, root_goal: Goal):
        """Add root goal to the tree"""
        if len(self._nodes) >= 1:
            raise ValueError("No more than 1 root goals allowed")
        
        self._add_node(root_goal)
    
    def get_node(self, node_id: str) -> Optional[Node]:
        """Get node by ID"""
        try:
            return self._nodes[node_id]
        except KeyError:
            raise ValueError("Could not find node.")
    
    def get_size(self) -> int:
        """Get number of nodes in the tree"""
        return len(self._nodes)
    
    def get_leaf_tasks(self) -> List[LeafTask]:
        """Get all leaf tasks in the tree"""
        leaf_tasks = []
        
        for node in self._nodes.values():
            if not node.has_children() and isinstance(node, LeafTask):
                leaf_tasks.append(node)
        
        return leaf_tasks