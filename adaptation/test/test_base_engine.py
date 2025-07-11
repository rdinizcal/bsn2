import pytest
import rclpy
from rclpy.parameter import Parameter
from unittest.mock import Mock, patch, MagicMock, AsyncMock
import yaml
import os
from ament_index_python.packages import get_package_share_directory
import asyncio

from adaptation.engines.base_engine import Engine
from adaptation.model.formula import Formula
from bsn_interfaces.srv import DataAccessRequest, EngineRequest
from bsn_interfaces.msg import Strategy, Exception as BSNException


class TestEngine(Engine):
    """Concrete implementation of Engine for testing"""

    def __init__(self, node_name="test_engine"):
        super().__init__(node_name)
        self.prefix = "T_"
        self.monitor_called = False
        self.analyze_called = False
        self.plan_called = False
        self.execute_called = False

    def get_prefix(self):
        return self.prefix

    def initialize_strategy(self, terms):
        strategy = {}
        for term in terms:
            strategy[term] = 1.0
        return strategy

    def initialize_priority(self, terms):
        priority = {}
        for term in terms:
            if term.startswith("T_"):
                priority[term] = 50
        return priority

    def monitor(self):
        self.monitor_called = True

    def analyze(self):
        self.analyze_called = True

    def plan(self):
        self.plan_called = True

    def execute(self):
        self.execute_called = True


@pytest.fixture(scope="class")
def engine_node(request):
    """Setup Engine node for testing - matching BSN1 parameters"""
    rclpy.init()

    # Create test parameters matching BSN1 Engine.cpp setUp()
    params = [
        Parameter(name="qos_attribute", value="reliability"),
        Parameter(name="info_quant", value=10.0),
        Parameter(name="monitor_freq", value=1.0),
        Parameter(name="actuation_freq", value=1.0),
    ]

    # Create engine node
    node = TestEngine("test_engine")
    node.set_parameters(params)

    # Mock ROS2 interfaces
    node.data_access_client = Mock()
    node.exception_subscriber = Mock()
    node.enactor_server = Mock()

    # Setup test formula matching BSN1 format
    formula_text = (
        "R_G3_T1_1 * CTX_G3_T1_1 * F_G3_T1_1 + R_G3_T1_2 * CTX_G3_T1_2 * F_G3_T1_2"
    )
    node.target_system_model = Formula(formula_text)

    # Initialize strategy and priority
    terms = node.target_system_model.get_terms()
    node.strategy = node.initialize_strategy(terms)
    node.priority = node.initialize_priority(terms)

    request.cls.engine_node = node
    yield node
    node.destroy_node()
    rclpy.shutdown()


@pytest.mark.usefixtures("engine_node")
class TestBaseEngine:
    """Test Base Engine functionality matching BSN1 Engine.cpp"""

    engine_node: TestEngine

    def test_initialization_matches_bsn1(self):
        """Test Engine initialization matches BSN1 Engine.cpp constructor"""
        # Check core attributes match Engine.hpp
        assert hasattr(self.engine_node, "qos_attribute")
        assert hasattr(self.engine_node, "info_quant")
        assert hasattr(self.engine_node, "monitor_freq")
        assert hasattr(self.engine_node, "actuation_freq")
        assert hasattr(self.engine_node, "target_system_model")
        assert hasattr(self.engine_node, "strategy")
        assert hasattr(self.engine_node, "priority")
        assert hasattr(self.engine_node, "deactivated_components")

        # Check parameters match BSN1 defaults
        assert self.engine_node.qos_attribute == "reliability"
        assert self.engine_node.info_quant == 10.0
        assert self.engine_node.monitor_freq == 1.0
        assert self.engine_node.actuation_freq == 1.0

        # Check Formula object exists (matching BSN1 target_system_model)
        assert isinstance(self.engine_node.target_system_model, Formula)

        # Check strategy and priority are dictionaries
        assert isinstance(self.engine_node.strategy, dict)
        assert isinstance(self.engine_node.priority, dict)
        assert isinstance(self.engine_node.deactivated_components, dict)

    def test_setup_formula_matches_bsn1(self):
        """Test setUp_formula matches BSN1 Engine.cpp setUp_formula()"""
        # Test formula setup with known formula
        formula_text = "R_G3_T1_1 * CTX_G3_T1_1 + R_G3_T1_2 * CTX_G3_T1_2"

        # Call setup_formula (matching BSN1)
        self.engine_node.setup_formula(formula_text)

        # Check Formula object was created
        assert isinstance(self.engine_node.target_system_model, Formula)

        # Check strategy was initialized with terms
        expected_terms = ["R_G3_T1_1", "CTX_G3_T1_1", "R_G3_T1_2", "CTX_G3_T1_2"]
        for term in expected_terms:
            assert term in self.engine_node.strategy
            assert (
                self.engine_node.strategy[term] == 1.0
            )  # initialize_strategy returns 1.0

        # Check priority was initialized for R_ terms only
        assert "R_G3_T1_1" in self.engine_node.priority
        assert "R_G3_T1_2" in self.engine_node.priority
        assert "CTX_G3_T1_1" not in self.engine_node.priority
        assert "CTX_G3_T1_2" not in self.engine_node.priority

    def test_calculate_qos_matches_bsn1(self):
        """Test calculate_qos matches BSN1 Engine.cpp calculate_qos()"""
        # Setup test formula and strategy
        formula_text = "R_G3_T1_1 * CTX_G3_T1_1"
        formula = Formula(formula_text)
        strategy = {"R_G3_T1_1": 0.85, "CTX_G3_T1_1": 1.0}

        # Call calculate_qos (matching BSN1)
        result = self.engine_node.calculate_qos(formula, strategy)

        # Check result matches expected calculation
        expected = 0.85 * 1.0  # 0.85
        assert result == expected

        # Test with different values
        strategy = {"R_G3_T1_1": 0.90, "CTX_G3_T1_1": 0.0}
        result = self.engine_node.calculate_qos(formula, strategy)
        expected = 0.90 * 0.0  # 0.0
        assert result == expected

    @patch("rclpy.spin_until_future_complete")
    def test_fetch_formula_matches_bsn1(self, mock_spin):
        """Test fetch_formula matches BSN1 Engine.cpp fetch_formula()"""
        # Setup mock response
        mock_response = Mock()
        mock_response.content = "R_G3_T1_1 * CTX_G3_T1_1"

        mock_future = Mock()
        mock_future.result.return_value = mock_response

        # Mock client behavior
        self.engine_node.data_access_client.wait_for_service.return_value = True
        self.engine_node.data_access_client.call_async.return_value = mock_future

        # Call fetch_formula (matching BSN1)
        result = self.engine_node.fetch_formula("reliability")

        # Check request was made correctly
        self.engine_node.data_access_client.call_async.assert_called_once()
        call_args = self.engine_node.data_access_client.call_async.call_args[0][0]
        assert call_args.name == "/engine"
        assert call_args.query == "reliability_formula"

        # Check result matches response
        assert result == "R_G3_T1_1 * CTX_G3_T1_1"

    @patch("rclpy.spin_until_future_complete")
    def test_fetch_formula_empty_response(self, mock_spin):
        """Test fetch_formula with empty response (matching BSN1 error handling)"""
        # Setup mock empty response
        mock_response = Mock()
        mock_response.content = ""

        mock_future = Mock()
        mock_future.result.return_value = mock_response

        self.engine_node.data_access_client.wait_for_service.return_value = True
        self.engine_node.data_access_client.call_async.return_value = mock_future

        # Call fetch_formula
        result = self.engine_node.fetch_formula("reliability")

        # Should return empty string (matching BSN1 behavior)
        assert result == ""

    def test_fetch_formula_service_unavailable(self):
        """Test fetch_formula when service unavailable (matching BSN1 error handling)"""
        # Mock service unavailable
        self.engine_node.data_access_client.wait_for_service.return_value = False

        # Call fetch_formula
        result = self.engine_node.fetch_formula("reliability")

        # Should return empty string (matching BSN1 behavior)
        assert result == ""

    def test_receive_exception_matches_bsn1(self):
        """Test receiveException matches BSN1 Engine.cpp receiveException()"""
        # Setup test priority
        self.engine_node.priority = {"T_G3_T1_1": 50, "T_G3_T1_2": 30}

        # Create exception message (matching BSN1 format)
        exception_msg = Mock()
        exception_msg.content = "/g3t1_1=10"

        # Call receive_exception (matching BSN1)
        self.engine_node.receive_exception(exception_msg)

        # Check priority was updated (matching BSN1 logic)
        assert self.engine_node.priority["T_G3_T1_1"] == 60  # 50 + 10

        # Test negative adjustment
        exception_msg.content = "/g3t1_2=-20"
        self.engine_node.receive_exception(exception_msg)
        assert self.engine_node.priority["T_G3_T1_2"] == 10  # 30 - 20

        # Test boundary conditions (matching BSN1 clamping)
        exception_msg.content = "/g3t1_1=50"
        self.engine_node.receive_exception(exception_msg)
        assert self.engine_node.priority["T_G3_T1_1"] == 100  # Clamped to 100

        exception_msg.content = "/g3t1_2=-20"
        self.engine_node.receive_exception(exception_msg)
        assert self.engine_node.priority["T_G3_T1_2"] == 0  # Clamped to 0

    def test_receive_exception_component_name_processing(self):
        """Test component name processing in receiveException (matching BSN1 logic)"""
        # Setup priority with processed names
        self.engine_node.priority = {"T_G3_T1_1": 50, "T_G4_T1": 40}

        # Test various component name formats
        test_cases = [
            ("/g3t1_1=5", "T_G3_T1_1", 55),  # Standard format
            ("/g4t1=10", "T_G4_T1", 50),  # No underscore
            ("/G3T1_1=15", "T_G3_T1_1", 70),  # Already uppercase
        ]

        for msg_content, expected_key, expected_value in test_cases:
            exception_msg = Mock()
            exception_msg.content = msg_content

            self.engine_node.receive_exception(exception_msg)
            assert self.engine_node.priority[expected_key] == expected_value

    def test_receive_exception_invalid_component(self):
        """Test receiveException with invalid component (matching BSN1 error handling)"""
        # Setup priority
        self.engine_node.priority = {"T_G3_T1_1": 50}

        # Create exception for non-existent component
        exception_msg = Mock()
        exception_msg.content = "/nonexistent=10"

        # Should not crash (matching BSN1 behavior)
        try:
            self.engine_node.receive_exception(exception_msg)
            # Should continue without error
            assert True
        except Exception as e:
            pytest.fail(
                f"receive_exception should handle invalid component gracefully: {e}"
            )

    def test_send_adaptation_parameter_matches_bsn1(self):
        """Test sendAdaptationParameter matches BSN1 Engine.cpp"""
        # Create request and response
        request = Mock()
        response = Mock()

        # Set qos_attribute
        self.engine_node.qos_attribute = "reliability"

        # Call send_adaptation_parameter
        result = self.engine_node.send_adaptation_parameter(request, response)

        # Check response content (matching BSN1)
        assert result.content == "reliability"

    def test_abstract_methods_implemented(self):
        """Test abstract methods are properly implemented"""
        # Check abstract methods exist and are callable
        assert callable(self.engine_node.get_prefix)
        assert callable(self.engine_node.initialize_strategy)
        assert callable(self.engine_node.initialize_priority)
        assert callable(self.engine_node.monitor)
        assert callable(self.engine_node.analyze)
        assert callable(self.engine_node.plan)
        assert callable(self.engine_node.execute)

        # Test return values
        assert self.engine_node.get_prefix() == "T_"

        terms = ["T_G3_T1_1", "CTX_G3_T1_1"]
        strategy = self.engine_node.initialize_strategy(terms)
        assert len(strategy) == 2
        assert all(v == 1.0 for v in strategy.values())

        priority = self.engine_node.initialize_priority(terms)
        assert "T_G3_T1_1" in priority
        assert "CTX_G3_T1_1" not in priority

    def test_body_method_structure(self):
        """Test body method structure matches BSN1 Engine.cpp body()"""
        # Mock the rate and spinning
        with patch("rclpy.spin_once") as mock_spin_once, patch(
            "rclpy.ok", side_effect=[True, True, False]
        ) as mock_ok, patch.object(
            self.engine_node, "create_rate"
        ) as mock_create_rate, patch.object(
            self.engine_node, "fetch_formula", return_value="test_formula"
        ) as mock_fetch, patch.object(
            self.engine_node, "setup_formula"
        ) as mock_setup:

            # Setup mock rate
            mock_rate = Mock()
            mock_create_rate.return_value = mock_rate

            # Call body method
            self.engine_node.body()

            # Check rate was created with correct frequency
            mock_create_rate.assert_called_with(1.0)  # monitor_freq

            # Check monitor was called
            assert self.engine_node.monitor_called

            # Check spin_once was called
            assert mock_spin_once.called

            # Check rate.sleep was called
            assert mock_rate.sleep.called

    def test_formula_reload_timing(self):
        """Test formula reload timing matches BSN1 (every 10 seconds)"""
        # Mock dependencies
        with patch("rclpy.spin_once"), patch(
            "rclpy.ok", side_effect=[True] * 12 + [False]
        ), patch.object(
            self.engine_node, "create_rate"
        ) as mock_create_rate, patch.object(
            self.engine_node, "fetch_formula", return_value="new_formula"
        ) as mock_fetch, patch.object(
            self.engine_node, "setup_formula"
        ) as mock_setup:

            # Setup mock rate
            mock_rate = Mock()
            mock_create_rate.return_value = mock_rate

            # Set monitor frequency to 1.0 (so 10 calls = 10 seconds)
            self.engine_node.monitor_freq = 1.0

            # Call body method
            self.engine_node.body()

            # Check formula was fetched after 10 iterations
            # Should be called at least once during the 12 iterations
            assert mock_fetch.call_count >= 1
            assert mock_setup.call_count >= 1

    def test_error_handling_matches_bsn1(self):
        """Test error handling matches BSN1 robustness"""
        # Test with malformed exception message
        exception_msg = Mock()
        exception_msg.content = "malformed_message"

        # Should not crash
        try:
            self.engine_node.receive_exception(exception_msg)
            assert True
        except Exception as e:
            pytest.fail(f"Should handle malformed exception gracefully: {e}")

        # Test with None formula
        try:
            result = self.engine_node.calculate_qos(None, {})
            # Should return 0.0 or handle gracefully
            assert isinstance(result, (int, float))
        except Exception as e:
            pytest.fail(f"Should handle None formula gracefully: {e}")

    def test_strategy_data_types_match_bsn1(self):
        """Test strategy data types match BSN1 (std::map<std::string, double>)"""
        # Check strategy values are floats (matching C++ double)
        for key, value in self.engine_node.strategy.items():
            assert isinstance(key, str)
            assert isinstance(value, (int, float))

        # Check priority values are ints (matching C++ int)
        for key, value in self.engine_node.priority.items():
            assert isinstance(key, str)
            assert isinstance(value, int)

        # Check deactivated_components values are ints (matching C++ int)
        for key, value in self.engine_node.deactivated_components.items():
            assert isinstance(key, str)
            assert isinstance(value, int)
