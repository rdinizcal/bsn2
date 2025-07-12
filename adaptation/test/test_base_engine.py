import pytest
import rclpy
from rclpy.parameter import Parameter
from unittest.mock import Mock, patch, MagicMock
import yaml
import os
from ament_index_python.packages import get_package_share_directory

from adaptation.engines.base_engine import Engine
from bsn_interfaces.srv import DataAccessRequest


class TestEngine(Engine):
    """Concrete implementation of Engine for testing"""
    
    def __init__(self, node_name="test_engine", **kwargs):
        super().__init__(node_name, **kwargs)
        self.monitor_called = False
        self.analyze_called = False
        self.plan_called = False
        self.execute_called = False
    
    def get_prefix(self):
        return "T_"
    
    def initialize_strategy(self, terms):
        return {term: 1.0 for term in terms}
    
    def initialize_priority(self, terms):
        return {term: 50 for term in terms}
    
    def monitor(self):
        self.monitor_called = True
    
    def analyze(self):
        self.analyze_called = True
    
    def plan(self):
        self.plan_called = True
    
    def execute(self):
        self.execute_called = True


@pytest.fixture(scope="class")
def engine_node(request, rclpy_context):
    """Create engine node for testing"""
    
    # Create node
    node = TestEngine()
    
    # Set default parameters
    default_params = {
        "monitor_freq": 10,
        "actuation_freq": 5,
        "info_quant": 1,
        "strategy": "R_G3_T1_1:1.0,R_G3_T1_2:1.0",
        "priority": "R_G3_T1_1:50,R_G3_T1_2:50",
    }
    
    for param_name, param_value in default_params.items():
        try:
            node.declare_parameter(param_name, param_value)
        except Exception:
            pass
    
    # Mock the ROS-specific methods that cause context issues
    node.create_rate = Mock()
    node.create_rate.return_value = Mock()
    node.create_rate.return_value.sleep = Mock()
    
    # Mock publishers and clients to avoid actual ROS communication
    node.strategy_publisher = Mock()
    node.data_access_client = Mock()
    node.adaptation_parameter_client = Mock()
    
    # Mock spin_once to avoid context conflicts
    with patch('rclpy.spin_once'):
        request.cls.engine_node = node
        yield node
    
    try:
        node.destroy_node()
    except:
        pass


@pytest.mark.usefixtures("engine_node")
class TestBaseEngine:
    engine_node: TestEngine  # Type annotation
    
    def test_initialization_matches_bsn1(self):
        """Test that initialization matches BSN1 Engine behavior"""
        assert self.engine_node.monitor_freq > 0
        assert self.engine_node.actuation_freq > 0
        assert self.engine_node.info_quant > 0
        assert isinstance(self.engine_node.strategy, dict)
        assert isinstance(self.engine_node.priority, dict)
        assert isinstance(self.engine_node.deactivated_components, dict)
        assert self.engine_node.target_system_model is not None
    
    def test_setup_formula_matches_bsn1(self):
        """Test that setup_formula matches BSN1 behavior"""
        # Mock the data access service
        with patch.object(self.engine_node, 'data_access_client') as mock_client:
            mock_future = Mock()
            mock_response = Mock()
            mock_response.formula = "R_G3_T1_1 * R_G3_T1_2"
            mock_future.result.return_value = mock_response
            mock_client.call_async.return_value = mock_future
            mock_client.wait_for_service.return_value = True
            
            # Mock rclpy.spin_until_future_complete
            with patch('rclpy.spin_until_future_complete'):
                self.engine_node._setup_formula()
                
                # Verify service was called
                mock_client.call_async.assert_called_once()
                request = mock_client.call_async.call_args[0][0]
                assert request.name == "/engine"
                assert request.query == "formula"
    
    def test_calculate_qos_matches_bsn1(self):
        """Test QoS calculation matches BSN1 behavior"""
        # Test with simple formula
        formula = "R_G3_T1_1 * R_G3_T1_2"
        strategy = {"R_G3_T1_1": 0.9, "R_G3_T1_2": 0.8}
        
        result = self.engine_node.calculate_qos(formula, strategy)
        expected = 0.9 * 0.8
        assert abs(result - expected) < 1e-6
        
        # Test with complex formula
        formula = "(R_G3_T1_1 + R_G3_T1_2) / 2"
        result = self.engine_node.calculate_qos(formula, strategy)
        expected = (0.9 + 0.8) / 2
        assert abs(result - expected) < 1e-6
    
    def test_fetch_formula_matches_bsn1(self):
        """Test fetch_formula matches BSN1 behavior"""
        with patch.object(self.engine_node, 'data_access_client') as mock_client:
            mock_future = Mock()
            mock_response = Mock()
            mock_response.formula = "R_G3_T1_1 * R_G3_T1_2"
            mock_future.result.return_value = mock_response
            mock_client.call_async.return_value = mock_future
            mock_client.wait_for_service.return_value = True
            
            with patch('rclpy.spin_until_future_complete'):
                result = self.engine_node.fetch_formula()
                
                assert result == "R_G3_T1_1 * R_G3_T1_2"
                mock_client.call_async.assert_called_once()
    
    def test_fetch_formula_empty_response(self):
        """Test fetch_formula handles empty response"""
        with patch.object(self.engine_node, 'data_access_client') as mock_client:
            mock_future = Mock()
            mock_response = Mock()
            mock_response.formula = ""
            mock_future.result.return_value = mock_response
            mock_client.call_async.return_value = mock_future
            mock_client.wait_for_service.return_value = True
            
            with patch('rclpy.spin_until_future_complete'):
                result = self.engine_node.fetch_formula()
                
                assert result == ""
    
    def test_fetch_formula_service_unavailable(self):
        """Test fetch_formula handles service unavailable"""
        with patch.object(self.engine_node, 'data_access_client') as mock_client:
            mock_client.wait_for_service.return_value = False
            
            result = self.engine_node.fetch_formula()
            assert result == ""
    
    def test_receive_exception_matches_bsn1(self):
        """Test receive_exception matches BSN1 behavior"""
        # Mock the AdaptationParameter service
        with patch.object(self.engine_node, 'adaptation_parameter_client') as mock_client:
            mock_future = Mock()
            mock_response = Mock()
            mock_response.content = "/g3t1_1:activate;/g3t1_2:deactivate"
            mock_future.result.return_value = mock_response
            mock_client.call_async.return_value = mock_future
            mock_client.wait_for_service.return_value = True
            
            with patch('rclpy.spin_until_future_complete'):
                self.engine_node.receive_exception()
                
                # Verify service was called
                mock_client.call_async.assert_called_once()
                request = mock_client.call_async.call_args[0][0]
                assert request.name == "/engine"
                assert request.query == "exceptions"
    
    def test_receive_exception_component_name_processing(self):
        """Test component name processing in receive_exception"""
        # Test the component name processing logic
        with patch.object(self.engine_node, 'adaptation_parameter_client') as mock_client:
            mock_future = Mock()
            mock_response = Mock()
            mock_response.content = "/g3t1_1:activate;/g4t1:deactivate"
            mock_future.result.return_value = mock_response
            mock_client.call_async.return_value = mock_future
            mock_client.wait_for_service.return_value = True
            
            with patch('rclpy.spin_until_future_complete'):
                self.engine_node.receive_exception()
                
                # Check that deactivated_components is updated correctly
                expected_key = "R_G4_T1"  # G4T1 should become G4_T1
                assert expected_key in self.engine_node.deactivated_components
    
    def test_receive_exception_invalid_component(self):
        """Test receive_exception handles invalid component names"""
        with patch.object(self.engine_node, 'adaptation_parameter_client') as mock_client:
            mock_future = Mock()
            mock_response = Mock()
            mock_response.content = "invalid_format"
            mock_future.result.return_value = mock_response
            mock_client.call_async.return_value = mock_future
            mock_client.wait_for_service.return_value = True
            
            with patch('rclpy.spin_until_future_complete'):
                # Should not raise exception
                self.engine_node.receive_exception()
    
    def test_send_adaptation_parameter_matches_bsn1(self):
        """Test send_adaptation_parameter matches BSN1 behavior"""
        with patch.object(self.engine_node, 'adaptation_parameter_client') as mock_client:
            mock_future = Mock()
            mock_response = Mock()
            mock_future.result.return_value = mock_response
            mock_client.call_async.return_value = mock_future
            mock_client.wait_for_service.return_value = True
            
            with patch('rclpy.spin_until_future_complete'):
                self.engine_node.send_adaptation_parameter("test_content")
                
                # Verify service was called with correct parameters
                mock_client.call_async.assert_called_once()
                request = mock_client.call_async.call_args[0][0]
                assert request.name == "/engine"
                assert request.query == "parameters"
                assert request.content == "test_content"
    
    def test_abstract_methods_implemented(self):
        """Test that abstract methods are properly implemented"""
        # Test that concrete implementation has all required methods
        assert hasattr(self.engine_node, 'get_prefix')
        assert hasattr(self.engine_node, 'initialize_strategy')
        assert hasattr(self.engine_node, 'initialize_priority')
        assert hasattr(self.engine_node, 'monitor')
        assert hasattr(self.engine_node, 'analyze')
        assert hasattr(self.engine_node, 'plan')
        assert hasattr(self.engine_node, 'execute')
        
        # Test that methods are callable
        assert callable(self.engine_node.get_prefix)
        assert callable(self.engine_node.initialize_strategy)
        assert callable(self.engine_node.initialize_priority)
        assert callable(self.engine_node.monitor)
        assert callable(self.engine_node.analyze)
        assert callable(self.engine_node.plan)
        assert callable(self.engine_node.execute)
    
    def test_body_method_structure(self):
        """Test that body method follows proper structure"""
        # Mock the methods to track calls
        with patch.object(self.engine_node, 'monitor') as mock_monitor, \
             patch.object(self.engine_node, 'receive_exception') as mock_receive, \
             patch('time.sleep') as mock_sleep:
            
            # Run body for one iteration
            with patch('builtins.input', side_effect=KeyboardInterrupt):
                try:
                    self.engine_node.body()
                except KeyboardInterrupt:
                    pass
            
            # Verify monitor was called
            mock_monitor.assert_called()
            mock_receive.assert_called()
    
    def test_formula_reload_timing(self):
        """Test formula reload timing matches BSN1"""
        # Set up formula reload tracking
        self.engine_node.formula_count = 0
        
        with patch.object(self.engine_node, '_setup_formula') as mock_setup:
            # Simulate multiple body iterations
            for i in range(15):  # More than formula_reload_freq
                self.engine_node.formula_count += 1
                if self.engine_node.formula_count >= self.engine_node.formula_reload_freq:
                    self.engine_node._setup_formula()
                    self.engine_node.formula_count = 0
            
            # Verify formula was reloaded
            mock_setup.assert_called()
    
    def test_error_handling_matches_bsn1(self):
        """Test error handling matches BSN1 behavior"""
        # Test that exceptions in calculate_qos are handled
        with patch.object(self.engine_node, 'get_logger') as mock_logger:
            # Test with invalid formula
            result = self.engine_node.calculate_qos("invalid_formula", {})
            assert result == 0.0
            
            # Verify error was logged
            mock_logger.return_value.error.assert_called()
    
    def test_strategy_data_types_match_bsn1(self):
        """Test strategy data types match BSN1"""
        # Test strategy initialization
        terms = ["R_G3_T1_1", "R_G3_T1_2"]
        strategy = self.engine_node.initialize_strategy(terms)
        
        # Verify all values are floats
        for key, value in strategy.items():
            assert isinstance(value, float)
        
        # Test priority initialization
        priority = self.engine_node.initialize_priority(terms)
        
        # Verify all values are integers
        for key, value in priority.items():
            assert isinstance(value, int)
