#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test suite for BSN2 DataAccess - Preserving BSN1 functionality
"""

import pytest
import rclpy
from rclpy.parameter import Parameter
from unittest.mock import Mock, patch, MagicMock, mock_open
import os
import json
import time
import tempfile
import shutil
from pathlib import Path

from adaptation.data_access.data_access import DataAccess
from bsn_interfaces.msg import Persist, TargetSystemData
from bsn_interfaces.srv import DataAccessRequest
from adaptation.adaptation.model.formula import Formula


@pytest.fixture(scope="class")
def data_access_node(request):
    """Setup DataAccess node for testing - matching BSN1 parameters"""
    rclpy.init()
    
    # Create temporary directory for testing
    temp_dir = tempfile.mkdtemp()
    models_dir = os.path.join(temp_dir, "models")
    os.makedirs(models_dir, exist_ok=True)
    
    # Create test parameters matching BSN1 DataAccess.cpp
    params = [
        Parameter(name='frequency', value=1.0),
        Parameter(name='buffer_size', value=1000),
        Parameter(name='time_window', value=10.1),
        Parameter(name='log_path', value=os.path.join(temp_dir, "logs")),
        Parameter(name='models_path', value=models_dir),
        Parameter(name='flush_interval', value=30),
    ]
    
    # Create test formula files
    with open(os.path.join(models_dir, "reliability.formula"), "w") as f:
        f.write("R_G3_T1_1 * CTX_G3_T1_1 * F_G3_T1_1 + R_G3_T1_2 * CTX_G3_T1_2 * F_G3_T1_2")
    
    with open(os.path.join(models_dir, "cost.formula"), "w") as f:
        f.write("W_G3_T1_1 * CTX_G3_T1_1 + W_G3_T1_2 * CTX_G3_T1_2")
    
    # Create test goal model
    goal_model = {
        "actors": [{
            "text": "BSN",
            "nodes": [
                {"id": "G1", "text": "Monitor Patient", "type": "Goal"},
                {"id": "T1.1", "text": "Collect Vital Signs", "type": "Task"},
                {"id": "T1.2", "text": "Process Data", "type": "Task"}
            ]
        }]
    }
    
    with open(os.path.join(models_dir, "goalModel.json"), "w") as f:
        json.dump(goal_model, f)
    
    # Create DataAccess node
    node = DataAccess()
    node.set_parameters(params)
    
    # Store temp directory for cleanup
    request.cls.temp_dir = temp_dir
    request.cls.data_access_node = node
    
    yield node
    
    # Cleanup
    node.destroy_node()
    shutil.rmtree(temp_dir, ignore_errors=True)
    rclpy.shutdown()


@pytest.mark.usefixtures("data_access_node")
class TestDataAccess:
    """Test DataAccess functionality matching BSN1 DataAccess.cpp"""
    
    data_access_node: DataAccess
    temp_dir: str
    
    def test_initialization_matches_bsn1(self):
        """Test DataAccess initialization matches BSN1 DataAccess.cpp constructor"""
        # Check core attributes match DataAccess.hpp
        assert hasattr(self.data_access_node, 'logical_clock')
        assert hasattr(self.data_access_node, 'frequency')
        assert hasattr(self.data_access_node, 'buffer_size')
        assert hasattr(self.data_access_node, 'time_window')
        assert hasattr(self.data_access_node, 'count_to_calc_and_reset')
        assert hasattr(self.data_access_node, 'count_to_fetch')
        assert hasattr(self.data_access_node, 'arrived_status')
        
        # Check parameters match BSN1 defaults
        assert self.data_access_node.frequency == 1.0
        assert self.data_access_node.buffer_size == 1000
        assert self.data_access_node.time_window == 10.1
        
        # Check data structures exist
        assert isinstance(self.data_access_node.status, dict)
        assert isinstance(self.data_access_node.events, dict)
        assert isinstance(self.data_access_node.contexts, dict)
        assert isinstance(self.data_access_node.components_reliabilities, dict)
        assert isinstance(self.data_access_node.components_batteries, dict)
        assert isinstance(self.data_access_node.components_costs_engine, dict)
        assert isinstance(self.data_access_node.components_costs_enactor, dict)
        
        # Check message vectors exist
        assert isinstance(self.data_access_node.status_messages, list)
        assert isinstance(self.data_access_node.event_messages, list)
        assert isinstance(self.data_access_node.energy_messages, list)
        assert isinstance(self.data_access_node.uncertainty_messages, list)
        assert isinstance(self.data_access_node.adaptation_messages, list)
    
    def test_component_initialization_matches_bsn1(self):
        """Test component data initialization matches BSN1 defaults"""
        # Check components are initialized with default values
        expected_components = ["g3t1_1", "g3t1_2", "g3t1_3", "g3t1_4", "g3t1_5", "g3t1_6"]
        
        for component in expected_components:
            # Check battery initialized to 100.0
            assert self.data_access_node.components_batteries[component] == 100.0
            
            # Check costs initialized to 0.0
            assert self.data_access_node.components_costs_engine[component] == 0.0
            assert self.data_access_node.components_costs_enactor[component] == 0.0
            
            # Check reliability initialized to 1.0
            assert self.data_access_node.components_reliabilities[component] == 1.0
    
    def test_formula_loading_matches_bsn1(self):
        """Test formula loading matches BSN1 formula loading"""
        # Check reliability formula was loaded
        assert self.data_access_node.reliability_formula_text != ""
        assert "R_G3_T1_1" in self.data_access_node.reliability_formula_text
        assert "CTX_G3_T1_1" in self.data_access_node.reliability_formula_text
        assert "F_G3_T1_1" in self.data_access_node.reliability_formula_text
        
        # Check cost formula was loaded
        assert self.data_access_node.cost_formula_text != ""
        assert "W_G3_T1_1" in self.data_access_node.cost_formula_text
        
        # Check Formula objects were created
        assert isinstance(self.data_access_node.reliability_formula, Formula)
        assert isinstance(self.data_access_node.cost_formula, Formula)
    
    def test_receive_persist_message_status_matches_bsn1(self):
        """Test receivePersistMessage for Status matches BSN1 DataAccess.cpp"""
        # Create status message
        status_msg = Persist()
        status_msg.type = "Status"
        status_msg.timestamp = int(time.time() * 1000000)
        status_msg.source = "/g3t1_1"
        status_msg.target = "/data_access"
        status_msg.content = "success"
        
        # Get initial values
        initial_logical_clock = self.data_access_node.logical_clock
        initial_arrived_status = self.data_access_node.arrived_status
        
        # Process message
        self.data_access_node.receive_persist_message(status_msg)
        
        # Check logical clock incremented
        assert self.data_access_node.logical_clock == initial_logical_clock + 1
        
        # Check arrived_status incremented
        assert self.data_access_node.arrived_status == initial_arrived_status + 1
        
        # Check status was stored in deque
        assert "/g3t1_1" in self.data_access_node.status
        assert len(self.data_access_node.status["/g3t1_1"]) == 1
        
        # Check status message was persisted
        assert len(self.data_access_node.status_messages) == 1
        assert self.data_access_node.status_messages[0].content == "success"
    
    def test_receive_persist_message_event_matches_bsn1(self):
        """Test receivePersistMessage for Event matches BSN1 DataAccess.cpp"""
        # Create event message
        event_msg = Persist()
        event_msg.type = "Event"
        event_msg.timestamp = int(time.time() * 1000000)
        event_msg.source = "/g3t1_1"
        event_msg.target = "/data_access"
        event_msg.content = "activate"
        
        # Process message
        self.data_access_node.receive_persist_message(event_msg)
        
        # Check event was stored in deque
        assert "/g3t1_1" in self.data_access_node.events
        assert len(self.data_access_node.events["/g3t1_1"]) == 1
        assert self.data_access_node.events["/g3t1_1"][0] == "activate"
        
        # Check context was updated (matching BSN1 logic)
        assert self.data_access_node.contexts["g3t1_1"] == 1
        
        # Check event message was persisted
        assert len(self.data_access_node.event_messages) == 1
        
        # Test deactivate event
        event_msg.content = "deactivate"
        self.data_access_node.receive_persist_message(event_msg)
        
        # Check context was updated to 0
        assert self.data_access_node.contexts["g3t1_1"] == 0
    
    def test_receive_persist_message_energy_status_matches_bsn1(self):
        """Test receivePersistMessage for EnergyStatus matches BSN1 DataAccess.cpp"""
        # Create energy status message from component
        energy_msg = Persist()
        energy_msg.type = "EnergyStatus"
        energy_msg.timestamp = int(time.time() * 1000000)
        energy_msg.source = "/g3t1_1"
        energy_msg.target = "/data_access"
        energy_msg.content = "5.5"
        
        # Get initial costs
        initial_engine_cost = self.data_access_node.components_costs_engine["g3t1_1"]
        initial_enactor_cost = self.data_access_node.components_costs_enactor["g3t1_1"]
        
        # Process message
        self.data_access_node.receive_persist_message(energy_msg)
        
        # Check costs were updated (matching BSN1 logic)
        assert self.data_access_node.components_costs_engine["g3t1_1"] == initial_engine_cost + 5.5
        assert self.data_access_node.components_costs_enactor["g3t1_1"] == initial_enactor_cost + 5.5
        
        # Check energy message was persisted
        assert len(self.data_access_node.energy_messages) == 1
    
    def test_receive_persist_message_engine_energy_status_matches_bsn1(self):
        """Test receivePersistMessage for engine EnergyStatus matches BSN1 DataAccess.cpp"""
        # Create energy status message from engine (multiple components)
        energy_msg = Persist()
        energy_msg.type = "EnergyStatus"
        energy_msg.timestamp = int(time.time() * 1000000)
        energy_msg.source = "/engine"
        energy_msg.target = "/data_access"
        energy_msg.content = "global:15.5;g3t1_1:3.2;g3t1_2:4.8;"
        
        # Process message
        self.data_access_node.receive_persist_message(energy_msg)
        
        # Check energy messages were persisted for each component
        assert len(self.data_access_node.energy_messages) >= 2
    
    def test_process_target_system_data_matches_bsn1(self):
        """Test processTargetSystemData matches BSN1 DataAccess.cpp"""
        # Create TargetSystemData message
        target_msg = TargetSystemData()
        target_msg.trm_batt = 85.5
        target_msg.ecg_batt = 90.2
        target_msg.oxi_batt = 75.8
        target_msg.abps_batt = 88.1
        target_msg.abpd_batt = 92.3
        target_msg.glc_batt = 78.9
        
        # Process message
        self.data_access_node.process_target_system_data(target_msg)
        
        # Check battery levels were updated (matching BSN1 logic)
        assert self.data_access_node.components_batteries["g3t1_1"] == 85.5
        assert self.data_access_node.components_batteries["g3t1_2"] == 90.2
        assert self.data_access_node.components_batteries["g3t1_3"] == 75.8
        assert self.data_access_node.components_batteries["g3t1_4"] == 88.1
        assert self.data_access_node.components_batteries["g3t1_5"] == 92.3
        assert self.data_access_node.components_batteries["g3t1_6"] == 78.9
    
    def test_process_query_formula_matches_bsn1(self):
        """Test processQuery for formulas matches BSN1 DataAccess.cpp"""
        # Test reliability formula query
        request = DataAccessRequest.Request()
        request.name = "/engine"
        request.query = "reliability_formula"
        
        response = DataAccessRequest.Response()
        result = self.data_access_node.process_query(request, response)
        
        # Check response contains reliability formula
        assert "R_G3_T1_1" in response.content
        assert "CTX_G3_T1_1" in response.content
        assert "F_G3_T1_1" in response.content
        
        # Test cost formula query
        request.query = "cost_formula"
        response = DataAccessRequest.Response()
        result = self.data_access_node.process_query(request, response)
        
        # Check response contains cost formula
        assert "W_G3_T1_1" in response.content
    
    def test_process_query_reliability_matches_bsn1(self):
        """Test processQuery for reliability matches BSN1 DataAccess.cpp"""
        # Setup test data - add some status messages
        status_data = [
            ("success", "/g3t1_1"),
            ("success", "/g3t1_1"),
            ("fail", "/g3t1_1"),
            ("success", "/g3t1_2"),
            ("success", "/g3t1_2"),
        ]
        
        for content, source in status_data:
            msg = Persist()
            msg.type = "Status"
            msg.timestamp = int(time.time() * 1000000)
            msg.source = source
            msg.target = "/data_access"
            msg.content = content
            self.data_access_node.receive_persist_message(msg)
        
        # Query reliability
        request = DataAccessRequest.Request()
        request.name = "/engine"
        request.query = "all:reliability:5"
        
        response = DataAccessRequest.Response()
        result = self.data_access_node.process_query(request, response)
        
        # Check response format matches BSN1: "/g3t1_1:success,success,fail,0.67;"
        assert "/g3t1_1:" in response.content
        assert "/g3t1_2:" in response.content
        
        # Check reliability calculation
        # g3t1_1: 2 success, 1 fail = 2/3 ≈ 0.67
        # g3t1_2: 2 success, 0 fail = 2/2 = 1.0
        assert "0.67" in response.content or "0.6667" in response.content
        assert "1.0" in response.content or "1.000000" in response.content
    
    def test_process_query_event_matches_bsn1(self):
        """Test processQuery for events matches BSN1 DataAccess.cpp"""
        # Setup test data - add some event messages
        event_data = [
            ("activate", "/g3t1_1"),
            ("deactivate", "/g3t1_1"),
            ("activate", "/g3t1_2"),
        ]
        
        for content, source in event_data:
            msg = Persist()
            msg.type = "Event"
            msg.timestamp = int(time.time() * 1000000)
            msg.source = source
            msg.target = "/data_access"
            msg.content = content
            self.data_access_node.receive_persist_message(msg)
        
        # Query events
        request = DataAccessRequest.Request()
        request.name = "/engine"
        request.query = "all:event:1"
        
        response = DataAccessRequest.Response()
        result = self.data_access_node.process_query(request, response)
        
        # Check response format matches BSN1: "/g3t1_1:deactivate;/g3t1_2:activate;"
        assert "/g3t1_1:" in response.content
        assert "/g3t1_2:" in response.content
        assert "deactivate" in response.content
        assert "activate" in response.content
    
    def test_process_query_cost_matches_bsn1(self):
        """Test processQuery for cost matches BSN1 DataAccess.cpp"""
        # Setup test data - add some energy costs
        self.data_access_node.components_costs_engine["g3t1_1"] = 15.5
        self.data_access_node.components_costs_engine["g3t1_2"] = 20.3
        self.data_access_node.components_costs_enactor["g3t1_1"] = 12.8
        self.data_access_node.components_costs_enactor["g3t1_2"] = 18.1
        
        # Query cost from engine
        request = DataAccessRequest.Request()
        request.name = "/engine"
        request.query = "all:cost:5"
        
        response = DataAccessRequest.Response()
        result = self.data_access_node.process_query(request, response)
        
        # Check response format matches BSN1: "/g3t1_1:15.5;/g3t1_2:20.3;"
        assert "/g3t1_1:" in response.content
        assert "/g3t1_2:" in response.content
        assert "15.5" in response.content
        assert "20.3" in response.content
        
        # Check costs were reset after query (matching BSN1 behavior)
        assert self.data_access_node.components_costs_engine["g3t1_1"] == 0
        assert self.data_access_node.components_costs_engine["g3t1_2"] == 0
        
        # Query cost from enactor
        request.name = "/enactor"
        response = DataAccessRequest.Response()
        result = self.data_access_node.process_query(request, response)
        
        # Check enactor costs were returned and reset
        assert "12.8" in response.content
        assert "18.1" in response.content
        assert self.data_access_node.components_costs_enactor["g3t1_1"] == 0
        assert self.data_access_node.components_costs_enactor["g3t1_2"] == 0
    
    def test_calculate_component_reliability_matches_bsn1(self):
        """Test calculateComponentReliability matches BSN1 DataAccess.cpp"""
        # Setup test data
        component = "/g3t1_1"
        current_time = self.data_access_node.now_seconds()
        
        # Add status data to deque
        self.data_access_node.status[component].append((current_time, "success"))
        self.data_access_node.status[component].append((current_time, "success"))
        self.data_access_node.status[component].append((current_time, "fail"))
        self.data_access_node.status[component].append((current_time, "success"))
        
        # Calculate reliability
        result = self.data_access_node._calculate_component_reliability(component)
        
        # Check format matches BSN1: "/g3t1_1:0.75;"
        assert result.startswith("/g3t1_1:")
        assert result.endswith(";")
        
        # Check reliability calculation: 3 success, 1 fail = 3/4 = 0.75
        assert "0.75" in result
        
        # Check component reliability was updated
        assert self.data_access_node.components_reliabilities["g3t1_1"] == 0.75
    
    def test_calculate_component_cost_matches_bsn1(self):
        """Test calculateComponentCost matches BSN1 DataAccess.cpp"""
        # Setup test data
        component = "/g3t1_1"
        self.data_access_node.components_costs_engine["g3t1_1"] = 25.8
        self.data_access_node.components_costs_enactor["g3t1_1"] = 30.2
        
        # Test engine cost calculation
        result = self.data_access_node._calculate_component_cost(component, "/engine")
        
        # Check format matches BSN1: "/g3t1_1:25.8;"
        assert result.startswith("/g3t1_1:")
        assert result.endswith(";")
        assert "25.8" in result
        
        # Check engine cost was reset
        assert self.data_access_node.components_costs_engine["g3t1_1"] == 0
        
        # Test enactor cost calculation
        result = self.data_access_node._calculate_component_cost(component, "/enactor")
        
        # Check enactor cost
        assert "30.2" in result
        
        # Check enactor cost was reset
        assert self.data_access_node.components_costs_enactor["g3t1_1"] == 0
    
    def test_apply_time_window_matches_bsn1(self):
        """Test applyTimeWindow matches BSN1 DataAccess.cpp"""
        component = "/g3t1_1"
        current_time = self.data_access_node.now_seconds()
        
        # Add old and new data
        old_time = current_time - 15.0  # Older than time_window (10.1)
        new_time = current_time - 5.0   # Newer than time_window
        
        self.data_access_node.status[component].append((old_time, "success"))
        self.data_access_node.status[component].append((new_time, "fail"))
        self.data_access_node.status[component].append((current_time, "success"))
        
        # Apply time window
        self.data_access_node._apply_time_window()
        
        # Check old data was removed
        assert len(self.data_access_node.status[component]) == 2
        
        # Check remaining data is recent
        remaining_times = [t for t, _ in self.data_access_node.status[component]]
        assert all(current_time - t < self.data_access_node.time_window for t in remaining_times)
    
    def test_timer_callback_matches_bsn1_body(self):
        """Test timer_callback matches BSN1 DataAccess.cpp body()"""
        # Get initial values
        initial_calc_reset = self.data_access_node.count_to_calc_and_reset
        initial_fetch = self.data_access_node.count_to_fetch
        
        # Call timer callback
        self.data_access_node.timer_callback()
        
        # Check counters were incremented
        assert self.data_access_node.count_to_calc_and_reset == initial_calc_reset + 1
        assert self.data_access_node.count_to_fetch == initial_fetch + 1
        
        # Test periodic reliability calculation
        self.data_access_node.count_to_calc_and_reset = int(self.data_access_node.frequency)
        
        with patch.object(self.data_access_node, '_apply_time_window') as mock_apply, \
             patch.object(self.data_access_node, '_calculate_component_reliability') as mock_calc:
            
            self.data_access_node.timer_callback()
            
            # Check time window was applied
            mock_apply.assert_called_once()
            
            # Check counter was reset
            assert self.data_access_node.count_to_calc_and_reset == 0
        
        # Test periodic formula reload
        self.data_access_node.count_to_fetch = int(self.data_access_node.frequency * 10)
        
        with patch.object(self.data_access_node, '_load_models') as mock_load:
            self.data_access_node.timer_callback()
            
            # Check models were reloaded
            mock_load.assert_called_once()
            
            # Check counter was reset
            assert self.data_access_node.count_to_fetch == 0
    
    def test_flush_logs_matches_bsn1(self):
        """Test flush logs matches BSN1 DataAccess.cpp flush()"""
        # Add some test messages
        self.data_access_node.status_messages.append(
            self.data_access_node.status_messages.__class__.__bases__[0](
                "Status", int(time.time() * 1000000), 1, "/g3t1_1", "/data_access", "success"
            )
        )
        
        self.data_access_node.event_messages.append(
            self.data_access_node.event_messages.__class__.__bases__[0](
                "Event", int(time.time() * 1000000), 2, "/g3t1_1", "/data_access", "activate"
            )
        )
        
        # Mock file operations
        with patch("builtins.open", mock_open()) as mock_file:
            self.data_access_node._flush_logs()
            
            # Check files were opened for writing
            assert mock_file.called
            
            # Check message vectors were cleared
            assert len(self.data_access_node.status_messages) == 0
            assert len(self.data_access_node.event_messages) == 0
    
    def test_buffer_size_management_matches_bsn1(self):
        """Test buffer size management matches BSN1 deque behavior"""
        component = "/g3t1_1"
        
        # Test status buffer size limit
        for i in range(self.data_access_node.buffer_size + 10):
            msg = Persist()
            msg.type = "Status"
            msg.timestamp = int(time.time() * 1000000)
            msg.source = component
            msg.target = "/data_access"
            msg.content = f"status_{i}"
            self.data_access_node.receive_persist_message(msg)
        
        # Check buffer size is respected
        assert len(self.data_access_node.status[component]) <= self.data_access_node.buffer_size
        
        # Test event buffer size limit
        for i in range(self.data_access_node.buffer_size + 10):
            msg = Persist()
            msg.type = "Event"
            msg.timestamp = int(time.time() * 1000000)
            msg.source = component
            msg.target = "/data_access"
            msg.content = "activate" if i % 2 == 0 else "deactivate"
            self.data_access_node.receive_persist_message(msg)
        
        # Check buffer size is respected
        assert len(self.data_access_node.events[component]) <= self.data_access_node.buffer_size
    
    def test_logical_clock_matches_bsn1(self):
        """Test logical clock behavior matches BSN1 DataAccess.cpp"""
        initial_clock = self.data_access_node.logical_clock
        
        # Send multiple messages
        for i in range(5):
            msg = Persist()
            msg.type = "Status"
            msg.timestamp = int(time.time() * 1000000)
            msg.source = f"/g3t1_{i}"
            msg.target = "/data_access"
            msg.content = "success"
            self.data_access_node.receive_persist_message(msg)
        
        # Check logical clock incremented for each message
        assert self.data_access_node.logical_clock == initial_clock + 5
    
    def test_error_handling_matches_bsn1_robustness(self):
        """Test error handling matches BSN1 robustness"""
        # Test malformed persist message
        msg = Persist()
        msg.type = "UnknownType"
        msg.timestamp = int(time.time() * 1000000)
        msg.source = "/g3t1_1"
        msg.target = "/data_access"
        msg.content = "test"
        
        # Should not crash
        try:
            self.data_access_node.receive_persist_message(msg)
            assert True
        except Exception as e:
            pytest.fail(f"Should handle unknown message type gracefully: {e}")
        
        # Test invalid query
        request = DataAccessRequest.Request()
        request.name = "/unknown"
        request.query = "invalid:query"
        
        response = DataAccessRequest.Response()
        result = self.data_access_node.process_query(request, response)
        
        # Should return empty response
        assert response.content == ""
    
    def test_ros_interfaces_match_bsn1(self):
        """Test ROS interfaces match BSN1 DataAccess.hpp"""
        # Check subscribers exist
        assert hasattr(self.data_access_node, 'persist_sub')
        assert hasattr(self.data_access_node, 'target_system_sub')
        
        # Check service exists
        assert hasattr(self.data_access_node, 'data_service')
        
        # Check node name
        assert self.data_access_node.get_name() == 'data_access'
    
    def test_data_structures_match_bsn1_types(self):
        """Test data structures match BSN1 DataAccess.hpp types"""
        # Check logical_clock is integer (matching C++ int64_t)
        assert isinstance(self.data_access_node.logical_clock, int)
        
        # Check frequency is float (matching C++ double)
        assert isinstance(self.data_access_node.frequency, float)
        
        # Check buffer_size is integer (matching C++ int)
        assert isinstance(self.data_access_node.buffer_size, int)
        
        # Check components_reliabilities values are float (matching C++ double)
        for value in self.data_access_node.components_reliabilities.values():
            assert isinstance(value, float)
        
        # Check components_batteries values are float (matching C++ double)
        for value in self.data_access_node.components_batteries.values():
            assert isinstance(value, float)
        
        # Check contexts values are integer (matching C++ uint32_t)
        for value in self.data_access_node.contexts.values():
            assert isinstance(value, int)
    
    def test_complete_workflow_matches_bsn1(self):
        """Test complete workflow matches BSN1 DataAccess behavior"""
        # Simulate complete data access workflow
        component = "/g3t1_1"
        
        # 1. Send status messages
        for i in range(3):
            msg = Persist()
            msg.type = "Status"
            msg.timestamp = int(time.time() * 1000000)
            msg.source = component
            msg.target = "/data_access"
            msg.content = "success" if i < 2 else "fail"
            self.data_access_node.receive_persist_message(msg)
        
        # 2. Send event message
        event_msg = Persist()
        event_msg.type = "Event"
        event_msg.timestamp = int(time.time() * 1000000)
        event_msg.source = component
        event_msg.target = "/data_access"
        event_msg.content = "activate"
        self.data_access_node.receive_persist_message(event_msg)
        
        # 3. Send energy message
        energy_msg = Persist()
        energy_msg.type = "EnergyStatus"
        energy_msg.timestamp = int(time.time() * 1000000)
        energy_msg.source = component
        energy_msg.target = "/data_access"
        energy_msg.content = "10.5"
        self.data_access_node.receive_persist_message(energy_msg)
        
        # 4. Update battery via TargetSystemData
        target_msg = TargetSystemData()
        target_msg.trm_batt = 85.0
        target_msg.ecg_batt = 90.0
        target_msg.oxi_batt = 80.0
        target_msg.abps_batt = 88.0
        target_msg.abpd_batt = 92.0
        target_msg.glc_batt = 75.0
        self.data_access_node.process_target_system_data(target_msg)
        
        # 5. Query reliability
        request = DataAccessRequest.Request()
        request.name = "/engine"
        request.query = "all:reliability:5"
        
        response = DataAccessRequest.Response()
        result = self.data_access_node.process_query(request, response)
        
        # Check complete workflow worked
        assert component in response.content
        assert "0.67" in response.content or "0.6667" in response.content  # 2/3 success
        assert self.data_access_node.contexts["g3t1_1"] == 1  # activated
        assert self.data_access_node.components_costs_engine["g3t1_1"] == 10.5
        assert self.data_access_node.components_batteries["g3t1_1"] == 85.0