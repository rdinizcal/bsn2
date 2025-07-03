"""
Unit tests for Goal Model using pytest
"""

import pytest
import sys
import os

# Add the module path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'adaptation'))

from adaptation.goal_model import *


class TestGoalModel:
    """Test class for Goal Model functionality"""
    
    def test_context_construction(self):
        """Test Context construction"""
        context_id = "CTX_G3_T1_1"
        description = "SaO2_available"
        value = False
        
        context = Context(context_id, description, value)
        
        assert context.get_id() == context_id
        assert context.get_description() == description
        assert context.get_value() == value
    
    def test_property_construction(self):
        """Test Property construction"""
        prop_id = "R_G3_T1_1"
        value = 0.95
        
        prop = Property(prop_id, value)
        
        assert prop.get_id() == prop_id
        assert prop.get_value() == value
    
    def test_goal_construction(self):
        """Test Goal construction"""
        goal_id = "G1"
        description = "Emergency is detected"
        
        goal = Goal(goal_id, description)
        
        assert goal.get_id() == goal_id
        assert goal.get_description() == description
        assert not goal.has_children()
    
    def test_task_construction(self):
        """Test Task construction"""
        task_id = "T1"
        description = "Monitor vital signs"
        
        task = Task(task_id, description)
        
        assert task.get_id() == task_id
        assert task.get_description() == description
        assert not task.has_children()
    
    def test_leaf_task_construction(self):
        """Test LeafTask construction"""
        task_id = "T1.1"
        description = "Collect temperature data"
        context = Context("CTX_T1_1", "Temperature sensor available", True)
        cost = Property("W_G3_T1_1", 5.2)
        reliability = Property("R_G3_T1_1", 0.95)
        frequency = Property("F_G3_T1_1", 1.0)
        
        leaf_task = LeafTask(task_id, description, context, cost, reliability, frequency)
        
        assert leaf_task.get_id() == task_id
        assert leaf_task.get_description() == description
        assert leaf_task.get_context().get_id() == context.get_id()
        assert leaf_task.get_cost().get_value() == 5.2
        assert leaf_task.get_reliability().get_value() == 0.95
        assert leaf_task.get_frequency().get_value() == 1.0
    
    def test_goal_tree_construction(self):
        """Test GoalTree construction and operations"""
        actor = "Body Sensor Network"
        goal_tree = GoalTree(actor)
        
        # Create goal hierarchy
        root_goal = Goal("G1", "Emergency is detected")
        sub_goal = Goal("G2", "Patient status is monitored")
        task = Task("T1", "Monitor vital signs")
        leaf_task = LeafTask(
            "T1.1", 
            "Collect temperature data",
            Context("CTX_T1_1", "Temperature sensor available", True),
            Property("W_G3_T1_1", 5.2),
            Property("R_G3_T1_1", 0.95),
            Property("F_G3_T1_1", 1.0)
        )
        
        # Build hierarchy
        task.add_child(leaf_task)
        sub_goal.add_child(task)
        root_goal.add_child(sub_goal)
        
        # Add to tree
        goal_tree.add_root_goal(root_goal)
        
        # Test tree properties
        assert goal_tree.get_actor() == actor
        assert goal_tree.get_size() == 4  # 4 nodes total
        
        # Test node retrieval
        retrieved_node = goal_tree.get_node("G1")
        assert retrieved_node.get_id() == "G1"
        
        # Test leaf tasks retrieval
        leaf_tasks = goal_tree.get_leaf_tasks()
        assert len(leaf_tasks) == 1
        assert leaf_tasks[0].get_id() == "T1.1"
    
    def test_task_cannot_contain_goals(self):
        """Test that Tasks cannot contain Goals"""
        task = Task("T1", "Monitor vital signs")
        goal = Goal("G1", "Emergency is detected")
        
        with pytest.raises(ValueError):
            task.add_child_goal(goal)
    
    def test_leaf_task_cannot_have_children(self):
        """Test that LeafTasks cannot have children"""
        leaf_task = LeafTask("T1.1", "Collect data")
        child_task = Task("T1.2", "Another task")
        
        with pytest.raises(ValueError):
            leaf_task.add_child(child_task)
    
    def test_goal_tree_single_root(self):
        """Test that GoalTree allows only one root goal"""
        goal_tree = GoalTree("BSN")
        root_goal1 = Goal("G1", "First goal")
        root_goal2 = Goal("G2", "Second goal")
        
        goal_tree.add_root_goal(root_goal1)
        
        with pytest.raises(ValueError):
            goal_tree.add_root_goal(root_goal2)


# Additional pytest-specific tests with fixtures and parametrization

@pytest.fixture
def sample_context():
    """Fixture providing a sample context"""
    return Context("CTX_TEST", "Test context", True)

@pytest.fixture
def sample_property():
    """Fixture providing a sample property"""
    return Property("PROP_TEST", 0.85)

@pytest.fixture
def sample_goal():
    """Fixture providing a sample goal"""
    return Goal("G_TEST", "Test goal")

@pytest.fixture
def sample_task():
    """Fixture providing a sample task"""
    return Task("T_TEST", "Test task")

@pytest.fixture
def sample_leaf_task():
    """Fixture providing a complete leaf task"""
    context = Context("CTX_LEAF", "Leaf context", True)
    cost = Property("W_LEAF", 10.0)
    reliability = Property("R_LEAF", 0.9)
    frequency = Property("F_LEAF", 2.0)
    return LeafTask("T_LEAF", "Leaf task", context, cost, reliability, frequency)

@pytest.fixture
def sample_goal_tree():
    """Fixture providing a sample goal tree"""
    return GoalTree("Test Actor")


class TestGoalModelWithFixtures:
    """Tests using pytest fixtures"""
    
    def test_context_with_fixture(self, sample_context):
        """Test context using fixture"""
        assert sample_context.get_id() == "CTX_TEST"
        assert sample_context.get_description() == "Test context"
        assert sample_context.get_value() is True
    
    def test_property_with_fixture(self, sample_property):
        """Test property using fixture"""
        assert sample_property.get_id() == "PROP_TEST"
        assert sample_property.get_value() == 0.85
    
    def test_goal_hierarchy(self, sample_goal, sample_task, sample_leaf_task):
        """Test building goal hierarchy"""
        # Build hierarchy
        sample_task.add_child(sample_leaf_task)
        sample_goal.add_child(sample_task)
        
        # Test hierarchy
        assert sample_goal.has_children()
        assert len(sample_goal.get_children()) == 1
        assert sample_goal.get_children()[0].get_id() == "T_TEST"
        
        assert sample_task.has_children()
        assert len(sample_task.get_children()) == 1
        assert sample_task.get_children()[0].get_id() == "T_LEAF"
    
    def test_goal_tree_operations(self, sample_goal_tree, sample_goal):
        """Test goal tree operations"""
        sample_goal_tree.add_root_goal(sample_goal)
        
        assert sample_goal_tree.get_size() == 1
        assert sample_goal_tree.get_actor() == "Test Actor"
        
        retrieved = sample_goal_tree.get_node("G_TEST")
        assert retrieved.get_id() == "G_TEST"


class TestGoalModelParametrized:
    """Parametrized tests for Goal Model"""
    
    @pytest.mark.parametrize("context_id,description,value", [
        ("CTX_1", "First context", True),
        ("CTX_2", "Second context", False),
        ("CTX_3", "Third context", True),
    ])
    def test_context_creation(self, context_id, description, value):
        """Test context creation with different parameters"""
        context = Context(context_id, description, value)
        
        assert context.get_id() == context_id
        assert context.get_description() == description
        assert context.get_value() == value
    
    @pytest.mark.parametrize("prop_id,value", [
        ("COST_1", 5.5),
        ("RELIABILITY_1", 0.95),
        ("FREQUENCY_1", 1.0),
        ("COST_2", 10.2),
    ])
    def test_property_creation(self, prop_id, value):
        """Test property creation with different parameters"""
        prop = Property(prop_id, value)
        
        assert prop.get_id() == prop_id
        assert prop.get_value() == value
    
    @pytest.mark.parametrize("node_type,node_id,description", [
        (Goal, "G1", "Goal 1"),
        (Goal, "G2", "Goal 2"),
        (Task, "T1", "Task 1"),
        (Task, "T2", "Task 2"),
    ])
    def test_node_creation(self, node_type, node_id, description):
        """Test node creation for different types"""
        node = node_type(node_id, description)
        
        assert node.get_id() == node_id
        assert node.get_description() == description
        assert not node.has_children()


class TestGoalModelEdgeCases:
    """Test edge cases and error conditions"""
    
    def test_empty_context_construction(self):
        """Test context with default values"""
        context = Context()
        
        assert context.get_id() == ""
        assert context.get_description() == ""
        assert context.get_value() is False
    
    def test_empty_property_construction(self):
        """Test property with default values"""
        prop = Property()
        
        assert prop.get_id() == ""
        assert prop.get_value() == 0.0
    
    def test_empty_goal_construction(self):
        """Test goal with default values"""
        goal = Goal()
        
        assert goal.get_id() == ""
        assert goal.get_description() == ""
        assert not goal.has_children()
    
    def test_leaf_task_with_minimal_params(self):
        """Test leaf task with minimal parameters"""
        leaf_task = LeafTask("MIN_TASK", "Minimal task")
        
        assert leaf_task.get_id() == "MIN_TASK"
        assert leaf_task.get_description() == "Minimal task"
        assert leaf_task.get_context() is not None
        assert leaf_task.get_cost() is not None
        assert leaf_task.get_reliability() is not None
        assert leaf_task.get_frequency() is not None
    
    def test_goal_tree_get_nonexistent_node(self):
        """Test getting nonexistent node from goal tree"""
        goal_tree = GoalTree("Test")
        
        with pytest.raises(ValueError, match="Could not find node"):
            goal_tree.get_node("NONEXISTENT")
    
    def test_node_child_operations(self):
        """Test node child addition and removal"""
        parent_goal = Goal("PARENT", "Parent goal")
        child_goal = Goal("CHILD", "Child goal")
        
        # Add child
        parent_goal.add_child(child_goal)
        assert parent_goal.has_children()
        assert len(parent_goal.get_children()) == 1
        
        # Get child
        retrieved_child = parent_goal.get_child("CHILD")
        assert retrieved_child.get_id() == "CHILD"
        
        # Remove child
        parent_goal.remove_child("CHILD")
        assert not parent_goal.has_children()
        
        # Try to remove nonexistent child
        with pytest.raises(ValueError, match="Child Not Found"):
            parent_goal.remove_child("NONEXISTENT")
        
        # Try to get nonexistent child
        with pytest.raises(ValueError, match="Child Not Found"):
            parent_goal.get_child("NONEXISTENT")


class TestGoalModelCopyOperations:
    """Test copy and deepcopy operations"""
    
    def test_context_copy(self):
        """Test context copy operations"""
        original = Context("CTX_ORIG", "Original context", True)
        
        # Test copy
        copied = original.__copy__()
        assert copied.get_id() == original.get_id()
        assert copied.get_description() == original.get_description()
        assert copied.get_value() == original.get_value()
        
        # Test deepcopy
        deep_copied = original.__deepcopy__({})
        assert deep_copied.get_id() == original.get_id()
        assert deep_copied.get_description() == original.get_description()
        assert deep_copied.get_value() == original.get_value()
    
    def test_property_copy(self):
        """Test property copy operations"""
        original = Property("PROP_ORIG", 42.0)
        
        # Test copy
        copied = original.__copy__()
        assert copied.get_id() == original.get_id()
        assert copied.get_value() == original.get_value()
        
        # Test deepcopy
        deep_copied = original.__deepcopy__({})
        assert deep_copied.get_id() == original.get_id()
        assert deep_copied.get_value() == original.get_value()
    
    def test_leaf_task_copy(self):
        """Test leaf task copy operations"""
        context = Context("CTX_COPY", "Copy context", True)
        cost = Property("W_COPY", 15.0)
        reliability = Property("R_COPY", 0.88)
        frequency = Property("F_COPY", 3.0)
        
        original = LeafTask("T_COPY", "Copy task", context, cost, reliability, frequency)
        
        # Test copy
        copied = original.__copy__()
        assert copied.get_id() == original.get_id()
        assert copied.get_description() == original.get_description()
        assert copied.get_context().get_id() == original.get_context().get_id()
        assert copied.get_cost().get_value() == original.get_cost().get_value()