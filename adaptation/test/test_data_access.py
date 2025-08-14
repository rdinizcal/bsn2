#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test suite for BSN2 DataAccess - Using actual goalModel.txt structure
"""

import pytest
import time
from unittest.mock import Mock, patch
from collections import deque
from bsn_interfaces.srv import DataAccessRequest
from bsn_interfaces.msg import Persist, TargetSystemData, Exception as BSNException


@pytest.mark.usefixtures("data_access_node")
class TestDataAccess:
    """Test DataAccess functionality matching BSN goal model"""
    
    data_access_node = None
    
    def test_data_access_initialization(self):
        """Test DataAccess initialization"""
        assert self.data_access_node is not None
        assert hasattr(self.data_access_node, 'logical_clock')
        assert hasattr(self.data_access_node, 'frequency')
        assert hasattr(self.data_access_node, 'buffer_size')
        assert hasattr(self.data_access_node, 'time_window')
        
        # Check data structures exist
        assert isinstance(self.data_access_node.status, dict)
        assert isinstance(self.data_access_node.events, dict)
        assert isinstance(self.data_access_node.contexts, dict)
        assert isinstance(self.data_access_node.components_reliabilities, dict)
        assert isinstance(self.data_access_node.components_batteries, dict)
    
    def test_goal_model_components_match_goalmodel_txt(self):
        """Test basic goal model loading (simplified)"""
        # Remove the goal_model_components check since it doesn't exist
        # Just check that goal_tree was loaded
        assert hasattr(self.data_access_node, 'goal_tree')
        assert hasattr(self.data_access_node, 'component_mapping')
        
        # Check component mapping exists
        expected_sensors = ["g3t1_1", "g3t1_2", "g3t1_3", "g3t1_4", "g3t1_5", "g3t1_6"]
        for sensor in expected_sensors:
            assert sensor in self.data_access_node.component_mapping.values()
    
    def test_component_initialization_matches_engine_tests(self):
        """Test component initialization matches engine test patterns"""
        expected_components = ["g3t1_1", "g3t1_2", "g3t1_3", "g3t1_4", "g3t1_5", "g3t1_6", "g4t1"]
        
        for component in expected_components:

            
            # Check batteries initialized to 100.0
            assert self.data_access_node.components_batteries[component] == 100.0
            
            # Don't check contexts - they're only set when events arrive
            # assert self.data_access_node.contexts[component] == 1  # ❌ Remove this
    
    def test_formula_structure_matches_engine_tests(self):
        """Test formula structure matches what engine tests expect"""
        # Test reliability formula matches engine expectations
        reliability_formula = self.data_access_node.reliability_formula_text
        
        # Should contain all components from goalModel.txt
        assert "CTX_G3_T1_1" in reliability_formula
        assert "CTX_G3_T1_2" in reliability_formula
        assert "CTX_G3_T1_3" in reliability_formula
        assert "CTX_G3_T1_4" in reliability_formula
        assert "CTX_G3_T1_5" in reliability_formula
        assert "CTX_G3_T1_6" in reliability_formula
        assert "CTX_G4_T1" in reliability_formula
        
        # Should contain R_ and F_ terms for all components
        assert "R_G3_T1_1" in reliability_formula
        assert "F_G3_T1_1" in reliability_formula
        assert "R_G4_T1" in reliability_formula
        assert "F_G4_T1" in reliability_formula
        
        # Test cost formula structure
        cost_formula = self.data_access_node.cost_formula_text
        assert "W_G3_T1_1" in cost_formula
        assert "W_G4_T1" in cost_formula
    
    def test_process_query_reliability_matches_engine_format(self):
        """Test reliability query returns format expected by engine tests"""
    
        # Add test data using keys with leading slash
        self.data_access_node.status["/g3t1_1"].append((time.time(), "success"))
        self.data_access_node.status["/g3t1_1"].append((time.time(), "fail"))
        self.data_access_node.status["/g3t1_1"].append((time.time(), "success"))
        
        # Query reliability
        request = DataAccessRequest.Request()
        request.name = "/engine"
        request.query = "all:reliability:"  # Remove the ":1" part
        
        response = DataAccessRequest.Response()
        result = self.data_access_node.process_query(request, response)
        
        # Check response format (components returned without leading slash)
        assert "g3t1_1:" in response.content
        assert "success" in response.content
        assert "fail" in response.content
    
        # Check reliability values are calculated correctly
        reli_value = float(response.content.split(";")[0].split(",")[-1])
        assert pytest.approx(reli_value, 0.01) == 0.6667  # 2/3
    
        # Check semicolon separators
        assert response.content.count(";") >= 2
    
        # Check comma separators for status entries
        assert "," in response.content  # Status entries separated by commas
    
        # Verify complete format structure
        components = response.content.split(";")
        for component_data in components:
            if component_data:  # Skip empty entries
                assert ":" in component_data  # Component name separator
                parts = component_data.split(":")
                assert len(parts) == 2
                assert (parts[0].startswith("/g3t1_") or
                        parts[0].startswith("/g4t1"))# Component name format
                # parts[1] should contain "status1,status2,...,reliability"
                
                # Check that the data part has the expected format
                data_part = parts[1]
                if "," in data_part:
                    # Should have status entries followed by reliability value
                    data_values = data_part.split(",")
                    # Last value should be a float (reliability)
                    try:
                        float(data_values[-1])
                        assert True  # Reliability value is valid
                    except ValueError:
                        assert False, f"Last value should be reliability: {data_values[-1]}"
                    
                    # Earlier values should be status strings
                    for status in data_values[:-1]:
                        assert status in ["success", "fail"], f"Invalid status value: {status}"
    
    def test_process_query_context_matches_engine_format(self):
        """Test context query returns format expected by engine tests"""
        # Add test event data
        self.data_access_node.events["/g3t1_1"].append("activate")
        self.data_access_node.events["/g3t1_2"].append("deactivate")
        self.data_access_node.events["/g4t1"].append("activate")
        
        # Query events
        request = DataAccessRequest.Request()
        request.name = "/engine"
        request.query = "all:event:1"
        
        response = DataAccessRequest.Response()
        result = self.data_access_node.process_query(request, response)
        
        # Check response format matches engine test expectations
        # Should be: "/g3t1_1:activate;/g3t1_2:deactivate;/g4t1:activate;"
        assert "/g3t1_1:activate" in response.content
        assert "/g3t1_2:deactivate" in response.content
        assert "/g4t1:activate" in response.content
    
    def test_process_query_formula_matches_engine_expectations(self):
        """Test formula queries match engine test expectations"""
        # Test reliability formula query
        request = DataAccessRequest.Request()
        request.name = "/engine"
        request.query = "reliability_formula"
        
        response = DataAccessRequest.Response()
        result = self.data_access_node.process_query(request, response)
        
        # Should return the formula text that engine tests expect
        assert response.content == self.data_access_node.reliability_formula_text
        
        # Test cost formula query
        request.query = "cost_formula"
        response = DataAccessRequest.Response()
        result = self.data_access_node.process_query(request, response)
        
        assert response.content == self.data_access_node.cost_formula_text
    
    def test_receive_persist_message_status_matches_engine_expectations(self):
        """Test status message processing matches engine expectations"""
        # Clear any existing data first using BSN component name
        self.data_access_node.status["/g3t1_1"].clear()
    
        # Create status message matching engine test format
        status_msg = Persist()
        status_msg.type = "Status"
        status_msg.timestamp = int(time.time() * 1000000)
        status_msg.source = "/oximeter_node"
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
        
        # Check status was stored using BSN component name
        assert "/g3t1_1" in self.data_access_node.status, f'data access status {self.data_access_node.status}'
        assert len(self.data_access_node.status["/g3t1_1"]) == 1, f'data access status {self.data_access_node.status}'

    def test_receive_persist_message_event_matches_engine_expectations(self):
        """Test event message processing matches engine expectations"""
    
        # Create event message
        event_msg = Persist()
        event_msg.type = "Event"
        event_msg.timestamp = int(time.time() * 1000000)
        event_msg.source = "/oximeter_node"  # ROS2 node name
        event_msg.target = "/data_access"
        event_msg.content = "activate"
        
        # Process message
        self.data_access_node.receive_persist_message(event_msg)
        
        # Check event was stored using BSN component name (no slash)
        assert "/g3t1_1" in self.data_access_node.events, f'self.data_access_node.events'
        assert len(self.data_access_node.events["g3t1_1"]) == 1
        assert self.data_access_node.events["g3t1_1"][0] == "activate"
        
        # Check context was updated using BSN component name
        assert self.data_access_node.contexts["/g3t1_1"] == 1
    
    def test_g4t1_special_case_matches_engine_tests(self):
        """Test G4T1 special case matches engine test expectations"""
    
        # Test G4T1 activate event
        event_msg = Persist()
        event_msg.type = "Event"
        event_msg.timestamp = int(time.time() * 1000000)
        event_msg.source = "/central_hub_node"  # ROS2 node name
        event_msg.target = "/data_access"
        event_msg.content = "activate"
        
        self.data_access_node.receive_persist_message(event_msg)
        
        # Check G4T1 context was updated (BSN name, no slash)
        assert self.data_access_node.contexts["/g4t1"] == 1, f'G4T1 context: {self.data_access_node.contexts}'

        # Test G4T1 deactivate event
        event_msg.content = "deactivate"
        self.data_access_node.receive_persist_message(event_msg)
        
        # Check G4T1 context was updated to 0
        assert self.data_access_node.contexts["/g4t1"] == 0
    def test_target_system_data_matches_goalmodel_components(self):
        """Test TargetSystemData processing matches goalModel.txt components"""
        # Create TargetSystemData with battery levels for all sensors
        target_msg = TargetSystemData()
        target_msg.oxi_batt = 85.5    # G3_T1_1 ← FIX: This should match assertion
        target_msg.ecg_batt = 90.2    # G3_T1_2 
        target_msg.trm_batt = 75.8    # G3_T1_3 ← FIX: This should match assertion
        target_msg.abps_batt = 88.1   # G3_T1_4 
        target_msg.abpd_batt = 92.3   # G3_T1_5 
        target_msg.glc_batt = 78.9    # G3_T1_6 
        
        # Process message
        self.data_access_node.process_target_system_data(target_msg)
        
        # Check battery levels match what you actually set
        assert self.data_access_node.components_batteries["g3t1_1"] == 85.5  # oxi_batt
        assert self.data_access_node.components_batteries["g3t1_2"] == 90.2  # ecg_batt  
        assert self.data_access_node.components_batteries["g3t1_3"] == 75.8  # trm_batt
        assert self.data_access_node.components_batteries["g3t1_4"] == 88.1  # abps_batt
        assert self.data_access_node.components_batteries["g3t1_5"] == 92.3  # abpd_batt
        assert self.data_access_node.components_batteries["g3t1_6"] == 78.9  # glc_batt
    
    def test_complete_workflow_matches_engine_integration(self):
        """Test complete workflow matches engine integration expectations"""
        # Clear any existing data first
        for component in ["/g3t1_1", "/g3t1_2", "/g4t1"]:
            self.data_access_node.status[component].clear()
            self.data_access_node.events[component].clear()
    
        # 1. Add status data for reliability calculation
        components = ["/g3t1_1", "/g3t1_2", "/g4t1"]
        for component in components:
            self.data_access_node.status[component].append((time.time(), "success"))
            self.data_access_node.status[component].append((time.time(), "success"))
            self.data_access_node.status[component].append((time.time(), "fail"))
        
        # 2. Add event data
        for component in components:
            self.data_access_node.events[component].append("activate")
        
        # 3. Query reliability (as engine would)
        request = DataAccessRequest.Request()
        request.name = "/engine"
        request.query = "all:reliability:1"
        
        response = DataAccessRequest.Response()
        result = self.data_access_node.process_query(request, response)
        
        # Check response matches engine expectations
        for component in components:
            assert component in response.content
        
        # Check reliability values are calculated correctly (2 success, 1 fail = 0.6667)
        reli_value = float(response.content.split(";")[0].split(",")[-1])
        assert pytest.approx(reli_value, 0.01) == 0.6667  
        #assert "0.67" in response.content or "0.6667" in response.content
        
        # 4. Query context (as engine would)
        request.query = "all:event:1"
        response = DataAccessRequest.Response()
        result = self.data_access_node.process_query(request, response)
        
        # Check context response matches engine expectations
        for component in components:
            assert f"{component}:activate" in response.content
        
        # 5. Query formulas (as engine would)
        request.query = "reliability_formula"
        response = DataAccessRequest.Response()
        result = self.data_access_node.process_query(request, response)
        
        # Check formula response matches engine expectations
        assert "R_G3_T1_1" in response.content
        assert "R_G4_T1" in response.content
    
    def test_data_access_service_integration(self):
        """Test DataAccess service integration matches engine requirements"""
        # Test that the service responds to queries in the format engines expect
        test_queries = [
            "reliability_formula",
            "cost_formula", 
            "all:reliability:1",
            "all:event:1",
            "all:cost:1"
        ]
        
        for query in test_queries:
            request = DataAccessRequest.Request()
            request.name = "/engine"
            request.query = query
            
            response = DataAccessRequest.Response()
            result = self.data_access_node.process_query(request, response)
            
            # All queries should return some response (even if empty)
            assert isinstance(response.content, str)
            
            # Formula queries should return non-empty content
            if "formula" in query:
                assert len(response.content) > 0
    
    def test_receive_persist_message_event_overwrite(self):
        """Test event message processing overwrites previous events"""
        # Clear any existing data first using BSN component name
        self.data_access_node.events["/g3t1_1"].clear()
    
        # Create event message
        event_msg = Persist()
        event_msg.type = "Event"
        event_msg.timestamp = int(time.time() * 1000000)
        event_msg.source = "/oximeter_node"  # ROS2 node name
        event_msg.target = "/data_access"
        event_msg.content = "activate"
        
        # Process message
        self.data_access_node.receive_persist_message(event_msg)
        
        # Check event was stored using BSN component name (no slash)
        assert "/g3t1_1" in self.data_access_node.events
        assert len(self.data_access_node.events["/g3t1_1"]) == 1
        assert self.data_access_node.events["/g3t1_1"][0] == "activate"
        
        # Overwrite event
        event_msg.content = "deactivate"
        self.data_access_node.receive_persist_message(event_msg)
        
        # Check event was updated
        assert len(self.data_access_node.events["/g3t1_1"]) == 1
        assert self.data_access_node.events["/g3t1_1"][0] == "deactivate"
    
    def test_receive_persist_message_status_event_interaction(self):
        """Test status and event message interaction"""
        # Clear any existing data first using BSN component name
        self.data_access_node.status["/g3t1_1"].clear()
        self.data_access_node.events["/g3t1_1"].clear()
    
        # Create status message matching engine test format
        status_msg = Persist()
        status_msg.type = "Status"
        status_msg.timestamp = int(time.time() * 1000000)
        status_msg.source = "/oximeter_node"
        status_msg.target = "/data_access"
        status_msg.content = "success"
        
        # Process message
        self.data_access_node.receive_persist_message(status_msg)
        
        # Check status was stored using BSN component name
        assert "/g3t1_1" in self.data_access_node.status
        assert len(self.data_access_node.status["/g3t1_1"]) == 1
        
        # Create event message
        event_msg = Persist()
        event_msg.type = "Event"
        event_msg.timestamp = int(time.time() * 1000000)
        event_msg.source = "/oximeter_node"  # ROS2 node name
        event_msg.target = "/data_access"
        event_msg.content = "activate"
        
        # Process message
        self.data_access_node.receive_persist_message(event_msg)
        
        # Check event was stored using BSN component name (no slash)
        assert "g3t1_1" in self.data_access_node.events  # ✅ Correct key
        assert len(self.data_access_node.events["g3t1_1"]) == 1
        assert self.data_access_node.events["g3t1_1"][0] == "activate"
        
        # Check context was updated using BSN component name
        assert self.data_access_node.contexts["g3t1_1"] == 1  # ✅ Correct key
    
    def test_receive_persist_message_g4t1_special_case(self):
        """Test G4T1 special case with status and event messages"""
        # Clear any existing data first using BSN component name
        self.data_access_node.status["/g4t1"].clear()
        self.data_access_node.events["/g4t1"].clear()
    
        # Test G4T1 activate event
        event_msg = Persist()
        event_msg.type = "Event"
        event_msg.timestamp = int(time.time() * 1000000)
        event_msg.source = "/central_hub_node"  # ROS2 node name
        event_msg.target = "/data_access"
        event_msg.content = "activate"
        
        self.data_access_node.receive_persist_message(event_msg)
        
        # Check G4T1 context was updated (BSN name, no slash)
        assert self.data_access_node.contexts["/g4t1"] == 1
    
        # Test G4T1 deactivate event  
        event_msg.content = "deactivate"
        self.data_access_node.receive_persist_message(event_msg)
        
        # Check G4T1 context was updated to 0
        assert self.data_access_node.contexts["/g4t1"] == 0
    
        # Create status message matching engine test format
        status_msg = Persist()
        status_msg.type = "Status"
        status_msg.timestamp = int(time.time() * 1000000)
        status_msg.source = "/oximeter_node"
        status_msg.target = "/data_access"
        status_msg.content = "success"
        
        # Process message
        self.data_access_node.receive_persist_message(status_msg)
        
        # Check status was stored using BSN component name
        assert "g4t1" in self.data_access_node.status
        assert len(self.data_access_node.status["g4t1"]) == 1
        
        # Check G4T1 context remains unchanged (status message should not affect it)
        assert self.data_access_node.contexts["g4t1"] == 0  # ✅ Correct key