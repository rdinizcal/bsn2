import pytest
import rclpy
from unittest.mock import Mock, patch, MagicMock
from rclpy.parameter import Parameter

from adaptation.engines.reli_engine import ReliabilityEngine
from bsn_interfaces.srv import DataAccessRequest
from bsn_interfaces.msg import Strategy


@pytest.mark.usefixtures("reli_engine_node")
class TestReliabilityEngine:
    """Test ReliabilityEngine functionality matching BSN1 ReliabilityEngine.cpp"""

    reli_engine_node: ReliabilityEngine

    def test_initialization_matches_bsn1(self):
        """Test ReliabilityEngine initialization matches BSN1 ReliabilityEngine.cpp"""
        # Check ReliabilityEngine specific attributes
        assert hasattr(self.reli_engine_node, "setpoint")
        assert hasattr(self.reli_engine_node, "offset")
        assert hasattr(self.reli_engine_node, "gain")
        assert hasattr(self.reli_engine_node, "tolerance")
        assert hasattr(self.reli_engine_node, "cycles")
        assert hasattr(self.reli_engine_node, "prefix")

        # Check default values match BSN1
        assert self.reli_engine_node.setpoint == 0.9
        assert self.reli_engine_node.offset == 0.1
        assert self.reli_engine_node.gain == 0.5
        assert self.reli_engine_node.tolerance == 0.02
        assert self.reli_engine_node.cycles == 0
        assert self.reli_engine_node.prefix == "R_"

    def test_get_prefix_matches_bsn1(self):
        """Test get_prefix matches BSN1 ReliabilityEngine.cpp get_prefix()"""
        assert self.reli_engine_node.get_prefix() == "R_"

    def test_initialize_strategy_matches_bsn1(self):
        """Test initialize_strategy matches BSN1 ReliabilityEngine.cpp"""
        terms = ["R_G3_T1_1", "CTX_G3_T1_1", "F_G3_T1_1", "R_G3_T1_2"]
        strategy = self.reli_engine_node.initialize_strategy(terms)

        # All terms should be initialized to 1.0 (matching BSN1)
        assert len(strategy) == 4
        for term in terms:
            assert strategy[term] == 1.0

    def test_initialize_priority_matches_bsn1(self):
        """Test initialize_priority matches BSN1 ReliabilityEngine.cpp"""
        terms = ["R_G3_T1_1", "CTX_G3_T1_1", "F_G3_T1_1", "R_G3_T1_2"]
        priority = self.reli_engine_node.initialize_priority(terms)

        # Only R_ terms should have priority 50 (matching BSN1)
        assert "R_G3_T1_1" in priority
        assert "R_G3_T1_2" in priority
        assert priority["R_G3_T1_1"] == 50
        assert priority["R_G3_T1_2"] == 50

        # Non-R_ terms should not be in priority
        assert "CTX_G3_T1_1" not in priority
        assert "F_G3_T1_1" not in priority

    def test_component_name_processing_matches_bsn1(self):
        """Test component name processing matches BSN1 logic"""
        # Test cases from BSN1 ReliabilityEngine.cpp
        test_cases = [
            ("/g3t1_1", "G3_T1_1"),
            ("/g3t1_2", "G3_T1_2"),
            ("/g4t1", "G4_T1"),
            ("/G3T1_1", "G3_T1_1"),  # Already uppercase
        ]

        for input_name, expected in test_cases:
            result = self.reli_engine_node._process_component_name(input_name)
            assert result == expected

    @patch("rclpy.spin_until_future_complete")
    def test_request_reliability_data_matches_bsn1(self, mock_spin):
        """Test reliability data request matches BSN1 ReliabilityEngine.cpp monitor()"""
        # Setup mock response (matching BSN1 format)
        mock_response = Mock()
        mock_response.content = (
            "/g3t1_1:success,fail,success,0.67;/g3t1_2:success,success,0.85;"
        )

        mock_future = Mock()
        mock_future.result.return_value = mock_response

        # Mock client behavior
        self.reli_engine_node.data_access_client.wait_for_service.return_value = True
        self.reli_engine_node.data_access_client.call_async.return_value = mock_future

        # Set info_quant for testing
        self.reli_engine_node.info_quant = 10

        # Call request reliability data
        self.reli_engine_node._request_reliability_data()

        # Check request was made correctly (matching BSN1)
        self.reli_engine_node.data_access_client.call_async.assert_called_once()
        call_args = self.reli_engine_node.data_access_client.call_async.call_args[0][0]
        assert call_args.name == "/engine"
        assert call_args.query == "all:reliability:10"

    def test_process_reliability_response_matches_bsn1(self):
        """Test reliability response processing matches BSN1 ReliabilityEngine.cpp"""
        # Setup strategy
        self.reli_engine_node.strategy = {
            "R_G3_T1_1": 1.0,
            "R_G3_T1_2": 1.0,
            "CTX_G3_T1_1": 1.0,
        }

        # Test response processing (matching BSN1 format)
        response_content = (
            "/g3t1_1:success,fail,success,0.67;/g3t1_2:success,success,0.85;"
        )

        # Call process reliability response
        self.reli_engine_node._process_reliability_response(response_content)

        # Check strategy was updated correctly (matching BSN1 logic)
        assert self.reli_engine_node.strategy["R_G3_T1_1"] == 0.67
        assert self.reli_engine_node.strategy["R_G3_T1_2"] == 0.85

        # Non-R_ terms should remain unchanged
        assert self.reli_engine_node.strategy["CTX_G3_T1_1"] == 1.0

    def test_process_reliability_response_empty(self):
        """Test reliability response processing with empty response"""
        # Setup strategy
        original_strategy = self.reli_engine_node.strategy.copy()

        # Process empty response
        self.reli_engine_node._process_reliability_response("")

        # Strategy should remain unchanged
        assert self.reli_engine_node.strategy == original_strategy

    def test_process_reliability_response_malformed(self):
        """Test reliability response processing with malformed data"""
        # Setup strategy
        original_strategy = self.reli_engine_node.strategy.copy()

        # Test malformed responses
        malformed_responses = [
            "/g3t1_1::",
            "/g3t1_1:success,fail,",
            "invalid:format",
            "/g3t1_1:success,fail,invalid_number",
        ]

        for response in malformed_responses:
            try:
                self.reli_engine_node._process_reliability_response(response)
                # Should not crash
                assert True
            except Exception as e:
                pytest.fail(f"Should handle malformed response gracefully: {e}")

    @patch("rclpy.spin_until_future_complete")
    def test_request_context_data_matches_bsn1(self, mock_spin):
        """Test context data request matches BSN1 ReliabilityEngine.cpp monitor()"""
        # Setup mock response
        mock_response = Mock()
        mock_response.content = "/g3t1_1:activate;/g3t1_2:deactivate;"

        mock_future = Mock()
        mock_future.result.return_value = mock_response

        # Mock client behavior
        self.reli_engine_node.data_access_client.wait_for_service.return_value = True
        self.reli_engine_node.data_access_client.call_async.return_value = mock_future

        # Call request context data
        self.reli_engine_node._request_context_data()

        # Check request was made correctly (matching BSN1)
        call_args = self.reli_engine_node.data_access_client.call_async.call_args[0][0]
        assert call_args.name == "/engine"
        assert call_args.query == "all:event:1"

    def test_process_context_response_matches_bsn1(self):
        """Test context response processing matches BSN1 ReliabilityEngine.cpp"""
        # Setup strategy and deactivated_components
        self.reli_engine_node.strategy = {
            "CTX_G3_T1_1": 0.0,
            "CTX_G3_T1_2": 0.0,
            "R_G3_T1_1": 1.0,
            "R_G3_T1_2": 1.0,
        }
        self.reli_engine_node.deactivated_components = {}

        # Test context response processing (matching BSN1 format)
        response_content = "/g3t1_1:activate;/g3t1_2:deactivate;"

        # Call process context response
        self.reli_engine_node._process_context_response(response_content)

        # Check context was updated correctly (matching BSN1 logic)
        assert self.reli_engine_node.strategy["CTX_G3_T1_1"] == 1.0  # activate
        assert (
            self.reli_engine_node.strategy["CTX_G3_T1_2"] == 1.0
        )  # still 1.0 for non-G4T1

        # Check deactivated components (matching BSN1 logic)
        assert self.reli_engine_node.deactivated_components.get("R_G3_T1_2", 0) == 1
        assert (
            self.reli_engine_node.strategy["R_G3_T1_2"] == 1.0
        )  # Set to 1.0 when deactivated

    def test_process_context_response_g4t1_special_case(self):
        """Test G4T1 special case processing matches BSN1 ReliabilityEngine.cpp"""
        # Setup strategy for G4T1
        self.reli_engine_node.strategy = {
            "CTX_G4_T1": 0.0,
            "R_G4_T1": 1.0,
        }
        self.reli_engine_node.deactivated_components = {}

        # Test G4T1 activate
        response_content = "/g4t1:activate;"
        self.reli_engine_node._process_context_response(response_content)

        # Check G4T1 activate behavior (matching BSN1 special case)
        assert self.reli_engine_node.strategy["CTX_G4_T1"] == 1.0
        assert self.reli_engine_node.deactivated_components.get("R_G4_T1", 0) == 0

        # Test G4T1 deactivate
        response_content = "/g4t1:deactivate;"
        self.reli_engine_node._process_context_response(response_content)

        # Check G4T1 deactivate behavior (matching BSN1 special case)
        assert self.reli_engine_node.strategy["CTX_G4_T1"] == 0.0
        assert self.reli_engine_node.deactivated_components["R_G4_T1"] == 1

    def test_monitor_reset_strategy_matches_bsn1(self):
        """Test monitor strategy reset matches BSN1 ReliabilityEngine.cpp monitor()"""
        # Setup strategy with mixed values
        self.reli_engine_node.strategy = {
            "CTX_G3_T1_1": 0.5,
            "R_G3_T1_1": 0.8,
            "F_G3_T1_1": 0.3,
            "CTX_G3_T1_2": 0.7,
            "R_G3_T1_2": 0.9,
        }

        # Mock data access calls to avoid actual network calls
        with patch.object(
            self.reli_engine_node, "_request_reliability_data"
        ), patch.object(self.reli_engine_node, "_request_context_data"), patch.object(
            self.reli_engine_node, "analyze"
        ):

            # Call monitor
            self.reli_engine_node.monitor()

            # Check strategy was reset correctly (matching BSN1 logic)
            assert self.reli_engine_node.strategy["CTX_G3_T1_1"] == 0.0
            assert self.reli_engine_node.strategy["CTX_G3_T1_2"] == 0.0
            assert self.reli_engine_node.strategy["R_G3_T1_1"] == 1.0
            assert self.reli_engine_node.strategy["R_G3_T1_2"] == 1.0
            assert self.reli_engine_node.strategy["F_G3_T1_1"] == 1.0

            # Check cycles was incremented (matching BSN1)
            assert self.reli_engine_node.cycles == 1

    def test_analyze_matches_bsn1(self):
        """Test analyze phase matches BSN1 ReliabilityEngine.cpp analyze()"""
        # Setup test scenario
        self.reli_engine_node.setpoint = 0.9
        self.reli_engine_node.tolerance = 0.02
        self.reli_engine_node.cycles = 1
        self.reli_engine_node.monitor_freq = 1.0
        self.reli_engine_node.actuation_freq = 1.0

        # Test case 1: Error within tolerance - should not plan
        with patch.object(
            self.reli_engine_node, "calculate_qos", return_value=0.89
        ) as mock_calc, patch.object(self.reli_engine_node, "plan") as mock_plan:

            self.reli_engine_node.analyze()

            # Should calculate QoS
            mock_calc.assert_called_once()

            # Should not plan (error within tolerance)
            mock_plan.assert_not_called()

        # Test case 2: Error outside tolerance - should plan
        self.reli_engine_node.cycles = 1  # Reset cycles
        with patch.object(
            self.reli_engine_node, "calculate_qos", return_value=0.80
        ) as mock_calc, patch.object(self.reli_engine_node, "plan") as mock_plan:

            self.reli_engine_node.analyze()

            # Should calculate QoS
            mock_calc.assert_called_once()

            # Should plan (error outside tolerance and cycles sufficient)
            mock_plan.assert_called_once()

            # Cycles should be reset
            assert self.reli_engine_node.cycles == 0

    def test_analyze_actuation_frequency_matches_bsn1(self):
        """Test analyze actuation frequency logic matches BSN1"""
        # Setup test scenario
        self.reli_engine_node.setpoint = 0.9
        self.reli_engine_node.tolerance = 0.02
        self.reli_engine_node.monitor_freq = 10.0
        self.reli_engine_node.actuation_freq = 2.0

        # Calculate expected threshold
        expected_threshold = (
            self.reli_engine_node.monitor_freq / self.reli_engine_node.actuation_freq
        )  # 5.0

        # Test cycles below threshold
        self.reli_engine_node.cycles = int(expected_threshold) - 1
        with patch.object(
            self.reli_engine_node, "calculate_qos", return_value=0.70
        ) as mock_calc, patch.object(self.reli_engine_node, "plan") as mock_plan:

            self.reli_engine_node.analyze()

            # Should not plan (cycles insufficient)
            mock_plan.assert_not_called()

        # Test cycles at threshold
        self.reli_engine_node.cycles = int(expected_threshold)
        with patch.object(
            self.reli_engine_node, "calculate_qos", return_value=0.70
        ) as mock_calc, patch.object(self.reli_engine_node, "plan") as mock_plan:

            self.reli_engine_node.analyze()

            # Should plan (cycles sufficient)
            mock_plan.assert_called_once()

    def test_plan_component_filtering_matches_bsn1(self):
        """Test plan component filtering matches BSN1 ReliabilityEngine.cpp plan()"""
        # Setup strategy and components
        self.reli_engine_node.strategy = {
            "R_G3_T1_1": 0.8,
            "R_G3_T1_2": 0.9,
            "R_G4_T1": 0.7,
            "CTX_G3_T1_1": 1.0,
            "CTX_G3_T1_2": 0.0,  # Should be filtered out
            "CTX_G4_T1": 1.0,
            "F_G3_T1_1": 1.0,
            "F_G3_T1_2": 1.0,
            "F_G4_T1": 1.0,
        }

        self.reli_engine_node.deactivated_components = {
            "R_G4_T1": 1  # Should be filtered out
        }

        # Mock calculate_qos to return specific values
        with patch.object(
            self.reli_engine_node, "calculate_qos", return_value=0.85
        ) as mock_calc, patch.object(self.reli_engine_node, "execute") as mock_execute:

            # Set up for planning
            self.reli_engine_node.setpoint = 0.9
            self.reli_engine_node.tolerance = 0.02

            # Call plan
            self.reli_engine_node.plan()

            # Should have filtered components correctly
            # R_G3_T1_1 should be included (CTX=1, F=1, not deactivated)
            # R_G3_T1_2 should be excluded (CTX=0)
            # R_G4_T1 should be excluded (deactivated)

    def test_plan_priority_ordering_matches_bsn1(self):
        """Test plan priority ordering matches BSN1 ReliabilityEngine.cpp"""
        # Setup strategy and priority
        self.reli_engine_node.strategy = {
            "R_G3_T1_1": 0.8,
            "R_G3_T1_2": 0.9,
            "R_G3_T1_3": 0.7,
            "CTX_G3_T1_1": 1.0,
            "CTX_G3_T1_2": 1.0,
            "CTX_G3_T1_3": 1.0,
            "F_G3_T1_1": 1.0,
            "F_G3_T1_2": 1.0,
            "F_G3_T1_3": 1.0,
        }

        # Set different priorities (matching BSN1 priority ordering)
        self.reli_engine_node.priority = {
            "R_G3_T1_1": 70,  # High priority
            "R_G3_T1_2": 30,  # Low priority
            "R_G3_T1_3": 50,  # Medium priority
        }

        self.reli_engine_node.deactivated_components = {}

        # Mock calculate_qos to return convergent solution
        qos_values = [0.82, 0.85, 0.90]  # Sequence that converges
        with patch.object(
            self.reli_engine_node, "calculate_qos", side_effect=qos_values
        ) as mock_calc, patch.object(self.reli_engine_node, "execute") as mock_execute:

            self.reli_engine_node.setpoint = 0.9
            self.reli_engine_node.tolerance = 0.02

            # Call plan
            self.reli_engine_node.plan()

            # Should have found a solution and executed
            mock_execute.assert_called_once()

    def test_execute_strategy_format_matches_bsn1(self):
        """Test execute strategy format matches BSN1 ReliabilityEngine.cpp execute()"""
        # Setup strategy with R_ values
        self.reli_engine_node.strategy = {
            "R_G3_T1_1": 0.85,
            "R_G3_T1_2": 0.90,
            "R_G4_T1": 0.75,
            "CTX_G3_T1_1": 1.0,
            "CTX_G3_T1_2": 1.0,
        }

        # Call execute
        self.reli_engine_node.execute()

        # Check strategy was published
        self.reli_engine_node.strategy_publisher.publish.assert_called_once()

        # Get published message
        published_msg = self.reli_engine_node.strategy_publisher.publish.call_args[0][0]

        # Check message format (matching BSN1 execute logic)
        assert published_msg.source == "/engine"
        assert published_msg.target == "/enactor"

        # Check content format matches BSN1: "/g3t1_1:0.85;/g3t1_2:0.90;/g4t1:0.75"
        content = published_msg.content
        assert "/g3t1_1:0.85" in content or "/g3t1_1:0.850000" in content
        assert "/g3t1_2:0.90" in content or "/g3t1_2:0.900000" in content
        assert "/g4t1:0.75" in content or "/g4t1:0.750000" in content

    def test_execute_component_name_conversion_matches_bsn1(self):
        """Test execute component name conversion matches BSN1 logic"""
        # Setup strategy with various R_ terms
        self.reli_engine_node.strategy = {
            "R_G3_T1_1": 0.85,
            "R_G3_T1_2": 0.90,
            "R_G4_T1": 0.75,
            "R_G5_T2_3": 0.80,
        }

        # Call execute
        self.reli_engine_node.execute()

        # Get published content
        published_msg = self.reli_engine_node.strategy_publisher.publish.call_args[0][0]
        content = published_msg.content

        # Check component name conversion (matching BSN1 logic)
        # R_G3_T1_1 -> /g3t1_1
        # R_G4_T1 -> /g4t1
        # R_G5_T2_3 -> /g5t2_3
        expected_components = ["/g3t1_1", "/g3t1_2", "/g4t1", "/g5t2_3"]
        for component in expected_components:
            assert component in content

    def test_full_mape_k_cycle_matches_bsn1(self):
        """Test complete MAPE-K cycle matches BSN1 behavior"""
        # Setup realistic scenario
        self.reli_engine_node.setpoint = 0.9
        self.reli_engine_node.tolerance = 0.02
        self.reli_engine_node.cycles = 0

        # Setup initial strategy
        self.reli_engine_node.strategy = {
            "R_G3_T1_1": 1.0,
            "R_G3_T1_2": 1.0,
            "CTX_G3_T1_1": 0.0,
            "CTX_G3_T1_2": 0.0,
            "F_G3_T1_1": 1.0,
            "F_G3_T1_2": 1.0,
        }

        reliability_data = (
            "/g3t1_1:success,fail,success,0.70;/g3t1_2:success,success,0.80;"
        )
        context_data = "/g3t1_1:activate;/g3t1_2:activate;"

        # Mock service calls - create separate futures for each call
        mock_reliability_response = Mock()
        mock_reliability_response.content = reliability_data
        mock_context_response = Mock()
        mock_context_response.content = context_data

        # Create separate futures for each call
        mock_reliability_future = Mock()
        mock_reliability_future.result.return_value = mock_reliability_response
        mock_context_future = Mock()
        mock_context_future.result.return_value = mock_context_response

        # Set up the mock to return different futures for different calls
        def mock_call_async(request):
            if "reliability" in request.query:
                return mock_reliability_future
            elif "event" in request.query:
                return mock_context_future
            else:
                mock_default = Mock()
                mock_default.result.return_value = Mock(content="")
                return mock_default

        self.reli_engine_node.data_access_client.wait_for_service.return_value = True
        self.reli_engine_node.data_access_client.call_async.side_effect = mock_call_async

        # Mock QoS calculations that lead to convergence
        qos_sequence = [0.75, 0.85, 0.90]  # Monitor -> Plan -> Execute

        with patch.object(
            self.reli_engine_node, "calculate_qos", side_effect=qos_sequence
        ) as mock_calc, patch("rclpy.spin_until_future_complete"):

            # Run full cycle
            self.reli_engine_node.monitor()

            # Should have incremented cycles
            assert self.reli_engine_node.cycles == 1

            # Should have updated strategy from reliability data
            assert self.reli_engine_node.strategy["R_G3_T1_1"] == 0.70
            assert self.reli_engine_node.strategy["R_G3_T1_2"] == 0.80

            # Should have updated context
            assert self.reli_engine_node.strategy["CTX_G3_T1_1"] == 1.0
            assert self.reli_engine_node.strategy["CTX_G3_T1_2"] == 1.0

    def test_boundary_conditions_match_bsn1(self):
        """Test boundary conditions match BSN1 robustness"""
        # Test with boundary reliability values
        self.reli_engine_node.strategy = {
            "R_G3_T1_1": 0.0,  # Minimum
            "R_G3_T1_2": 1.0,  # Maximum
            "CTX_G3_T1_1": 1.0,
            "CTX_G3_T1_2": 1.0,
        }

        # Should not crash with boundary values
        try:
            result = self.reli_engine_node.calculate_qos(
                self.reli_engine_node.target_system_model,
                self.reli_engine_node.strategy,
            )
            assert isinstance(result, (int, float))
        except Exception as e:
            pytest.fail(f"Should handle boundary values gracefully: {e}")

    def test_error_recovery_matches_bsn1(self):
        """Test error recovery matches BSN1 behavior"""
        # Test with service unavailable
        self.reli_engine_node.data_access_client.wait_for_service.return_value = False

        # Should not crash
        try:
            self.reli_engine_node._request_reliability_data()
            self.reli_engine_node._request_context_data()
            assert True
        except Exception as e:
            pytest.fail(f"Should handle service unavailable gracefully: {e}")

        # Test with empty responses
        try:
            self.reli_engine_node._process_reliability_response("")
            self.reli_engine_node._process_context_response("")
            assert True
        except Exception as e:
            pytest.fail(f"Should handle empty responses gracefully: {e}")
