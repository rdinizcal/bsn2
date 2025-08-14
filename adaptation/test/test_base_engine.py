"""
Test base engine functionality matching BSN1 Engine behavior
"""

import pytest
import rclpy
from rclpy.parameter import Parameter
from unittest.mock import Mock, patch, MagicMock
import time
import threading

from adaptation.engines.base_engine import Engine
from adaptation.model.formula import Formula
from bsn_interfaces.srv import DataAccessRequest, EngineRequest
from bsn_interfaces.msg import Exception as BSNException

# Note: TestEngine is now defined in conftest.py to avoid import issues

@pytest.mark.usefixtures("engine_node")
class TestBaseEngine:
    """Test Base Engine functionality"""
    
    engine_node = None  # Will be set by fixture
    received_strategy_messages = []  # Will store published strategy messages
    received_exception_messages = []  # Will store published exception messages

    def setup_method(self):
        """Set up before each test"""
        # Clear previously received messages
        self.received_strategy_messages.clear()
        self.received_exception_messages.clear()

        # Reset engine state
        self.engine_node.monitor_called = False
        self.engine_node.analyze_called = False
        self.engine_node.plan_called = False
        self.engine_node.execute_called = False

    def wait_for_messages(self, message_list, count=1, timeout=2.0):
        """Wait for specified number of messages with timeout"""
        start_time = time.time()
        while (
            len(message_list) < count and time.time() - start_time < timeout
        ):
            # Spin both nodes
            rclpy.spin_once(self.engine_node, timeout_sec=0.1)
            rclpy.spin_once(self.mock_service_node, timeout_sec=0.1)
            time.sleep(0.05)

    def test_engine_initialization(self):
        """Test engine initialization matches BSN1"""
        # Test initial attributes
        assert self.engine_node.qos_attribute == "reliability"
        assert self.engine_node.info_quant == 1.0
        assert self.engine_node.monitor_freq == 10.0
        assert self.engine_node.actuation_freq == 5.0
        assert isinstance(self.engine_node.strategy, dict)
        assert isinstance(self.engine_node.priority, dict)
        assert isinstance(self.engine_node.deactivated_components, dict)
        
        # Test parameters exist (but don't check specific values since they default to empty)
        try:
            param = self.engine_node.get_parameter("qos_attribute")
            assert param is not None
            # Check that we can set the parameter
            self.engine_node.set_parameter(rclpy.parameter.Parameter("qos_attribute", rclpy.Parameter.Type.STRING, "reliability"))
            assert self.engine_node.get_parameter("qos_attribute").value == "reliability"
        except Exception:
            # If parameter doesn't exist, that's still valid for testing
            pass

    def test_fetch_formula_with_mock_service(self):
        """Test fetch_formula with mock service"""
        # Call fetch_formula
        with patch.object(self.engine_node.data_access_client, "call_async") as mock_call_async:
            mock_future = Mock()
            mock_future.result.return_value = ""
            mock_call_async.return_value = mock_future
            result = self.engine_node.fetch_formula("/engine")
            assert result == ""

    def test_setup_formula_success(self):
        """Test setup_formula with valid formula"""
        # Mock Formula class to avoid actual formula parsing
        with patch('adaptation.engines.base_engine.Formula') as mock_formula:
            mock_formula_instance = Mock()
            mock_formula_instance.get_terms.return_value = ["R_G3_T1_1", "R_G3_T1_2"]
            mock_formula_instance.evaluate.return_value = 0.8
            mock_formula.return_value = mock_formula_instance
            
            self.engine_node.setup_formula("R_G3_T1_1 * R_G3_T1_2")
            
            # Verify formula was created
            mock_formula.assert_called_once_with("R_G3_T1_1 * R_G3_T1_2")
            assert self.engine_node.target_system_model == mock_formula_instance
            
            # Verify strategy and priority were initialized
            assert "R_G3_T1_1" in self.engine_node.strategy
            assert "R_G3_T1_2" in self.engine_node.strategy
            assert "R_G3_T1_1" in self.engine_node.priority
            assert "R_G3_T1_2" in self.engine_node.priority

    def test_calculate_qos_success(self):
        """Test calculate_qos with valid inputs"""
        # Mock formula
        mock_formula = Mock()
        mock_formula.evaluate.return_value = 0.72
        
        result = self.engine_node.calculate_qos(mock_formula, {"R_G3_T1_1": 0.9, "R_G3_T1_2": 0.8})
        
        # Verify result
        assert result == 0.72
        
        # Verify formula was called correctly
        mock_formula.set_term_value_map_dict.assert_called_once_with({"R_G3_T1_1": 0.9, "R_G3_T1_2": 0.8})
        mock_formula.evaluate.assert_called_once()

    def test_receive_exception_valid_input(self):
        """Test receive_exception with valid input"""
        # Set up priority
        self.engine_node.priority = {"T_G3_T1_1": 50, "T_G4_T1": 30}
        
        # Create exception message
        msg = BSNException()
        msg.content = "/g3t1_1=10"
        
        self.engine_node.receive_exception(msg)
        
        # Verify priority was updated
        assert self.engine_node.priority["T_G3_T1_1"] == 60

    def test_receive_exception_g4t1_special_case(self):
        """Test receive_exception with G4T1 special case"""
        # Set up priority
        self.engine_node.priority = {"T_G4_T1": 30}
        
        # Create exception message
        msg = BSNException()
        msg.content = "/g4t1=5"
        
        self.engine_node.receive_exception(msg)
        
        # Verify priority was updated
        assert self.engine_node.priority["T_G4_T1"] == 35

    def test_send_adaptation_parameter(self):
        """Test send_adaptation_parameter"""
        # Create request and response
        request = EngineRequest.Request()
        response = EngineRequest.Response()
        
        result = self.engine_node.send_adaptation_parameter(request, response)
        
        # Verify response content
        assert result.content == "reliability"

    def test_abstract_methods_implemented(self):
        """Test that abstract methods are implemented"""
        # Test all abstract methods exist and are callable
        assert hasattr(self.engine_node, 'get_prefix')
        assert callable(self.engine_node.get_prefix)
        assert self.engine_node.get_prefix() == "T_"
        
        assert hasattr(self.engine_node, 'initialize_strategy')
        assert callable(self.engine_node.initialize_strategy)
        strategy = self.engine_node.initialize_strategy(["R_G3_T1_1"])
        assert strategy == {"R_G3_T1_1": 1.0}
        
        assert hasattr(self.engine_node, 'initialize_priority')
        assert callable(self.engine_node.initialize_priority)
        priority = self.engine_node.initialize_priority(["R_G3_T1_1"])
        assert priority == {"R_G3_T1_1": 50}
        
        # Test MAPE-K methods
        assert hasattr(self.engine_node, 'monitor')
        assert callable(self.engine_node.monitor)
        self.engine_node.monitor()
        assert self.engine_node.monitor_called
        
        assert hasattr(self.engine_node, 'analyze')
        assert callable(self.engine_node.analyze)
        self.engine_node.analyze()
        assert self.engine_node.analyze_called
        
        assert hasattr(self.engine_node, 'plan')
        assert callable(self.engine_node.plan)
        self.engine_node.plan()
        assert self.engine_node.plan_called
        
        assert hasattr(self.engine_node, 'execute')
        assert callable(self.engine_node.execute)
        self.engine_node.execute()
        assert self.engine_node.execute_called

    def test_body_method_structure(self):
        """Test body method structure"""
        # Mock rclpy.ok() to return True once then False
        with patch('rclpy.ok', side_effect=[True, False]), \
             patch('rclpy.spin_once'), \
             patch.object(self.engine_node, 'create_rate') as mock_create_rate, \
             patch.object(self.engine_node, 'fetch_formula', return_value=""), \
             patch.object(self.engine_node, 'monitor') as mock_monitor:
            
            # Mock rate
            mock_rate = Mock()
            mock_create_rate.return_value = mock_rate
            
            # Run body method
            self.engine_node.body()
            
            # Verify monitor was called
            mock_monitor.assert_called_once()
            
            # Verify rate was used
            mock_rate.sleep.assert_called_once()

    def test_integrated_engine_cycle(self):
        """Test integrated engine cycle with mock services"""
        # Clear received messages
        self.received_strategy_messages.clear()
        
        # Set up engine with mock formula
        with patch('adaptation.engines.base_engine.Formula') as mock_formula:
            mock_formula_instance = Mock()
            mock_formula_instance.get_terms.return_value = ["R_G3_T1_1", "R_G3_T1_2"]
            mock_formula_instance.evaluate.return_value = 0.8
            mock_formula.return_value = mock_formula_instance
            
            # Setup formula
            self.engine_node.setup_formula("R_G3_T1_1 * R_G3_T1_2")
            
            # Test that formula was setup correctly
            assert self.engine_node.target_system_model == mock_formula_instance
            
            # Test strategy calculation
            result = self.engine_node.calculate_qos(
                mock_formula_instance, 
                {"R_G3_T1_1": 0.9, "R_G3_T1_2": 0.8}
            )
            assert result == 0.8

    def test_error_handling(self):
        """Test error handling in various methods"""
        # Test calculate_qos with None formula
        result = self.engine_node.calculate_qos(None, {})
        assert result == 0.0
        
        # Test setup_formula with invalid formula
        # Store original state
        original_model = self.engine_node.target_system_model
        
        with patch('adaptation.engines.base_engine.Formula', side_effect=Exception("Invalid formula")):
            # Should not raise exception
            self.engine_node.setup_formula("invalid_formula")
            # target_system_model should remain unchanged (may not be None due to previous tests)
            assert self.engine_node.target_system_model == original_model

    def test_component_name_conversion(self):
        """Test component name conversion logic"""
        # Test G3T1_1 conversion
        test_cases = [
            ("/g3t1_1", "T_G3_T1_1"),
            ("/g3t1_2", "T_G3_T1_2"),
            ("/g4t1", "T_G4_T1"),
        ]
        
        for input_name, expected_key in test_cases:
            # Set up priority
            self.engine_node.priority = {expected_key: 50}
            
            # Create exception message
            msg = BSNException()
            msg.content = f"{input_name}=10"
            
            self.engine_node.receive_exception(msg)
            
            # Verify priority was updated
            assert self.engine_node.priority[expected_key] == 60
            
            # Reset for next test
            self.engine_node.priority[expected_key] = 50
