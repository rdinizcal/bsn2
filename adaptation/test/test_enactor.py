"""
Test suite for BSN2 Enactor/Controller - Following data_access pattern
"""

import pytest
import time
from unittest.mock import Mock, patch
from collections import deque

from adaptation.enactor.enactor import Enactor
from adaptation.enactor.controller import Controller
from bsn_interfaces.msg import Event, Strategy, AdaptationCommand, Exception as BSNException
from bsn_interfaces.srv import DataAccessRequest, EngineRequest



@pytest.fixture(scope="class")
def enactor_node(request):
    """Create enactor node with mocked services"""
    
    # Mock all service waits to prevent hanging
    with patch('rclpy.client.Client.wait_for_service') as mock_wait:
        with patch('rclpy.client.Client.service_is_ready') as mock_ready:
            mock_wait.return_value = True
            mock_ready.return_value = True
            
            # Create the controller
            controller = Controller()
            
            # Mock the service clients to prevent actual service calls
            mock_response = Mock()
            mock_response.success = True
            mock_response.result = "success"
            mock_response.data = "test_data"
            
            mock_future = Mock()
            mock_future.result.return_value = mock_response
            
            # Mock service client calls
            controller.data_access_client.call_async = Mock(return_value=mock_future)
            controller.engine_client.call_async = Mock(return_value=mock_future)
            
            # Mock publishers
            controller.adapt = Mock()
            controller.except_pub = Mock()
            
            # Set the controller in the test class
            request.cls.controller = controller
            
            yield controller
            
            # Cleanup
            try:
                controller.destroy_node()
            except:
                pass

@pytest.mark.usefixtures("enactor_node")
class TestEnactor:
    """Test Enactor/Controller functionality following data_access pattern"""
    
    controller: Controller  # This will be set by the enactor_node fixture
    
    def test_enactor_initialization(self):
        """Test Enactor initialization following data_access pattern"""
        assert self.controller is not None
        assert hasattr(self.controller, 'invocations')
        assert hasattr(self.controller, 'exception_buffer')
        assert hasattr(self.controller, 'freq')
        assert hasattr(self.controller, 'r_curr')
        assert hasattr(self.controller, 'c_curr')
        assert hasattr(self.controller, 'r_ref')
        assert hasattr(self.controller, 'c_ref')
        assert hasattr(self.controller, 'replicate_task')
        assert hasattr(self.controller, 'cycles')
        assert hasattr(self.controller, 'stability_margin')
        assert hasattr(self.controller, 'adaptation_parameter')
        
        # Check data structures are proper types
        assert isinstance(self.controller.invocations, dict)
        assert isinstance(self.controller.exception_buffer, dict)
        assert isinstance(self.controller.freq, dict)
        assert isinstance(self.controller.r_curr, dict)
        assert isinstance(self.controller.c_curr, dict)
        assert isinstance(self.controller.r_ref, dict)
        assert isinstance(self.controller.c_ref, dict)
        assert isinstance(self.controller.replicate_task, dict)
    
    def test_inheritance_structure(self):
        """Test inheritance structure"""
        assert isinstance(self.controller, Enactor)
        assert issubclass(Controller, Enactor)
        
        # Check Node inheritance
        assert hasattr(self.controller, 'get_name')
        assert hasattr(self.controller, 'create_publisher')
        assert hasattr(self.controller, 'create_subscription')
        assert hasattr(self.controller, 'create_service')
        assert hasattr(self.controller, 'create_client')
    
    def test_default_parameters(self):
        """Test default parameters"""
        assert self.controller.cycles == 0
        assert self.controller.stability_margin == 0.02
        assert self.controller.adaptation_parameter == "reliability"
        assert self.controller.frequency == 1.0
        assert self.controller.kp == 0.5
    
    def test_publishers_and_clients_exist(self):
        """Test publishers and service clients exist"""
        # Check publishers exist (mocked)
        assert hasattr(self.controller, 'adapt')
        assert hasattr(self.controller, 'except_pub')
        assert self.controller.adapt is not None
        assert self.controller.except_pub is not None
        
        # Check service clients exist (mocked)
        assert hasattr(self.controller, 'data_access_client')
        assert hasattr(self.controller, 'engine_client')
        assert self.controller.data_access_client is not None
        assert self.controller.engine_client is not None
    
    def test_abstract_methods_enforced(self):
        """Test abstract methods are enforced"""
        enactor = Enactor()
        
        with pytest.raises(NotImplementedError):
            enactor.receive_event(Mock())
        
        with pytest.raises(NotImplementedError):
            enactor.apply_reli_strategy("test_component")
        
        with pytest.raises(NotImplementedError):
            enactor.apply_cost_strategy("test_component")
    
    def test_receive_event_activate_component(self):
        """Test component activation matches data_access pattern"""
        event = Event()
        event.source = "/g3t1_1"
        event.content = "activate"
        event.freq = 2.5
        
        self.controller.receive_event(event)
        
        component = "/g3t1_1"
        assert component in self.controller.invocations
        assert component in self.controller.exception_buffer
        assert component in self.controller.freq
        assert component in self.controller.r_curr
        assert component in self.controller.r_ref
        assert component in self.controller.replicate_task
        assert component in self.controller.kp_individual
        
        # Check initial values
        assert isinstance(self.controller.invocations[component], deque)
        assert self.controller.invocations[component].maxlen == 100
        assert self.controller.exception_buffer[component] == 0
        assert self.controller.freq[component] == 2.5
        assert self.controller.r_curr[component] == 1.0
        assert self.controller.r_ref[component] == 1.0
        assert self.controller.replicate_task[component] == 1
        assert self.controller.kp_individual[component] == self.controller.kp
    
    def test_receive_event_deactivate_component(self):
        """Test component deactivation matches data_access pattern"""
        component = "/g3t1_1"
        
        # First activate
        event = Event()
        event.source = component
        event.content = "activate"
        event.freq = 2.0
        self.controller.receive_event(event)
        
        assert component in self.controller.invocations
        
        # Then deactivate
        event.content = "deactivate"
        self.controller.receive_event(event)
        
        # Check complete removal
        assert component not in self.controller.invocations
        assert component not in self.controller.exception_buffer
        assert component not in self.controller.freq
        assert component not in self.controller.r_curr
        assert component not in self.controller.c_curr
        assert component not in self.controller.r_ref
        assert component not in self.controller.c_ref
        assert component not in self.controller.replicate_task
        assert component not in self.controller.kp_individual
    
    def test_receive_strategy_reliability(self):
        """Test strategy reception with reliability parameter"""
        strategy = Strategy()
        strategy.source = "/engine"
        strategy.target = "/enactor"
        strategy.content = "/g3t1_1:0.85;/g3t1_2:0.92"
        
        self.controller.receive_strategy(strategy)
        
        assert "/g3t1_1" in self.controller.r_ref
        assert "/g3t1_2" in self.controller.r_ref
        assert self.controller.r_ref["/g3t1_1"] == 0.85, f'self.controller.r_ref'
        assert self.controller.r_ref["/g3t1_2"] == 0.92
    
    def test_receive_strategy_cost(self):
        """Test strategy reception with cost parameter"""
        self.controller.adaptation_parameter = "cost"
        
        strategy = Strategy()
        strategy.source = "/engine"
        strategy.target = "/enactor"
        strategy.content = "/g3t1_1:15.5;/g3t1_2:22.3"
        
        self.controller.receive_strategy(strategy)
        
        assert "/g3t1_1" in self.controller.c_ref
        assert "/g3t1_2" in self.controller.c_ref
        assert self.controller.c_ref["/g3t1_1"] == 15.5
        assert self.controller.c_ref["/g3t1_2"] == 22.3
    
    def test_receive_status_data_access_query(self):
        """Test status query to DataAccess service"""
        self.controller.adaptation_parameter = "reliability"
        self.controller.receive_status()
        
        # Check service was called
        self.controller.data_access_client.call_async.assert_called_once()
        
        # Verify request format
        call_args = self.controller.data_access_client.call_async.call_args
        request = call_args[0][0]
        assert request.name == "/enactor"
        assert request.query == "all:reliability:"
    
    def test_process_status_response(self):
        """Test status response processing"""
        # Setup test components
        self.controller.r_curr = {"/g3t1_1": 0.0, "/g3t1_2": 0.0}
        self.controller.r_ref = {"/g3t1_1": 0.9, "/g3t1_2": 0.9}
        
        with patch.object(self.controller, 'apply_reli_strategy') as mock_apply:
            response_content = "/g3t1_1:success,fail,success,0.6667;/g3t1_2:success,success,1.0000;"
            
            self.controller._process_status_response(response_content)
            
            # Check current values were updated
            assert self.controller.r_curr["/g3t1_1"] == 0.6667
            assert self.controller.r_curr["/g3t1_2"] == 1.0000
            
            # Check apply_reli_strategy was called
            assert mock_apply.call_count == 2
    
    def test_apply_reli_strategy_basic(self):
        """Test basic reliability strategy application"""
        component = "/g3t1_1"
        
        # Setup component data
        self.controller.r_curr[component] = 0.75
        self.controller.r_ref[component] = 0.90
        self.controller.freq[component] = 2.0
        self.controller.exception_buffer[component] = 0
        self.controller.kp_individual[component] = 0.5
        self.controller.invocations[component] = deque(maxlen=100)
        
        with patch.object(self.controller, '_send_adaptation_command') as mock_send:
            self.controller.apply_reli_strategy(component)
            
            # Check frequency was adjusted
            expected_freq_change = (0.5 / 100) * 0.15  # (kp/100) * error
            expected_new_freq = 2.0 + expected_freq_change
            assert abs(self.controller.freq[component] - expected_new_freq) < 0.001
            
            # Check adaptation command was sent
            mock_send.assert_called_once()
            
            # Check exception buffer incremented
            assert self.controller.exception_buffer[component] == 1
            
            # Check invocations cleared
            assert len(self.controller.invocations[component]) == 0
    
    def test_apply_cost_strategy_basic(self):
        """Test basic cost strategy application"""
        component = "/g3t1_1"
        
        # Setup component data
        self.controller.c_curr[component] = 25.0
        self.controller.c_ref[component] = 15.0
        self.controller.replicate_task[component] = 2
        self.controller.exception_buffer[component] = 0
        self.controller.kp_individual[component] = 0.5
        self.controller.invocations[component] = deque(maxlen=100)
        
        with patch.object(self.controller, '_send_adaptation_command') as mock_send:
            self.controller.apply_cost_strategy(component)
            
            # Check adaptation command was sent
            mock_send.assert_called_once()
            
            # Check exception buffer incremented
            assert self.controller.exception_buffer[component] == 1
            
            # Check invocations cleared
            assert len(self.controller.invocations[component]) == 0
    
    def test_stability_margin_behavior(self):
        """Test stability margin prevents unnecessary adaptations"""
        component = "/g3t1_1"
        
        # Setup component within stability margin
        self.controller.r_curr[component] = 0.895
        self.controller.r_ref[component] = 0.900
        self.controller.freq[component] = 2.0
        self.controller.exception_buffer[component] = 0
        self.controller.kp_individual[component] = 0.5
        self.controller.invocations[component] = deque(maxlen=100)
        
        with patch.object(self.controller, '_send_adaptation_command') as mock_send:
            self.controller.apply_reli_strategy(component)
            
            # Within stability margin - no adaptation
            mock_send.assert_not_called()
    
    def test_exception_management_threshold(self):
        """Test exception management at threshold"""
        component = "/g3t1_1"
        
        # Setup component at exception threshold
        self.controller.r_curr[component] = 0.5
        self.controller.r_ref[component] = 0.9
        self.controller.freq[component] = 2.0
        self.controller.exception_buffer[component] = 4  # At threshold
        self.controller.kp_individual[component] = 0.5
        self.controller.invocations[component] = deque(maxlen=100)
        
        with patch.object(self.controller, '_publish_exception') as mock_pub:
            self.controller.apply_reli_strategy(component)
            
            # Check exception was published
            mock_pub.assert_called_once_with(component, "1")
            
            # Check exception buffer was reset
            assert self.controller.exception_buffer[component] == 0
    
    def test_body_cycle_management(self):
        """Test body cycle management"""
        with patch.object(self.controller, 'receive_status') as mock_receive:
            # Initial cycles
            self.controller.cycles = 30
            self.controller.body()
            
            mock_receive.assert_not_called()
            assert self.controller.cycles == 31
            
            # After stabilization
            self.controller.cycles = 61
            self.controller.body()
            
            mock_receive.assert_called_once()
            assert self.controller.cycles == 62
    
    def test_send_adaptation_command_format(self):
        """Test adaptation command message format"""
        component = "/g3t1_1"
        action = "freq=2.50"
        
        self.controller._send_adaptation_command(component, action)
        
        # Check message was published
        #self.controller.adapt.publish.assert_called_once()
        
        # Verify message format
        published_msg = self.controller.adapt.publish.call_args[0][0]
        assert published_msg.source == self.controller.get_name()
        assert published_msg.component == component
        assert published_msg.action == action
    
    def test_publish_exception_format(self):
        """Test exception message format"""
        component = "/g3t1_1"
        value = "1"
        
        self.controller._publish_exception(component, value)
        
        # Check message was published
        self.controller.except_pub.publish.assert_called_once()
        
        # Verify message format
        published_msg = self.controller.except_pub.publish.call_args[0][0]
        assert published_msg.source == self.controller.get_name()
        assert published_msg.component == "/engine"
        assert published_msg.content == f"{component}={value}"
    
    def test_receive_adaptation_parameter_query(self):
        """Test adaptation parameter query to engine"""
        self.controller.receive_adaptation_parameter()
        
        # Check service was called
        self.controller.engine_client.call_async.assert_called_once()
        
        # Verify request format
        call_args = self.controller.engine_client.call_async.call_args
        request = call_args[0][0]
        assert request.source == "/enactor"
    
    def test_tear_down_cleanup(self):
        """Test tearDown cleanup functionality"""
        # Setup test data
        self.controller.invocations["/g3t1_1"] = deque([1, 0, 1])
        self.controller.exception_buffer["/g3t1_1"] = 2
        self.controller.freq["/g3t1_1"] = 2.5
        
        self.controller.tear_down()
        
        # Check all data structures were cleared
        assert len(self.controller.invocations) == 0
        assert len(self.controller.exception_buffer) == 0
        assert len(self.controller.freq) == 0
        assert len(self.controller.r_curr) == 0
        assert len(self.controller.c_curr) == 0
        assert len(self.controller.r_ref) == 0
        assert len(self.controller.c_ref) == 0
        assert len(self.controller.replicate_task) == 0
    
    def test_invocations_deque_behavior(self):
        """Test invocations deque behavior"""
        component = "/g3t1_1"
        
        # Activate component
        event = Event()
        event.source = component
        event.content = "activate"
        event.freq = 1.0
        self.controller.receive_event(event)
        
        # Check deque properties
        assert isinstance(self.controller.invocations[component], deque)
        assert self.controller.invocations[component].maxlen == 100
        
        # Test operations
        invocations = self.controller.invocations[component]
        for i in range(5):
            invocations.append(1 if i % 2 == 0 else 0)
        
        assert list(invocations) == [1, 0, 1, 0, 1]
        
        invocations.clear()
        assert len(invocations) == 0
    
    def test_complete_workflow_integration(self):
        """Test complete workflow integration"""
        component = "/g3t1_1"
        
        # 1. Component activation
        event = Event()
        event.source = component
        event.content = "activate"
        event.freq = 2.0
        self.controller.receive_event(event)
        
        # 2. Strategy reception
        strategy = Strategy()
        strategy.source = "/engine"
        strategy.target = "/enactor"
        strategy.content = f"{component}:0.85"
        self.controller.receive_strategy(strategy)
        
        # 3. Status update
        self.controller.r_curr[component] = 0.75
        
        # 4. Strategy application
        with patch.object(self.controller, '_send_adaptation_command') as mock_send:
            self.controller.apply_reli_strategy(component)
            
            # Verify workflow
            assert self.controller.r_ref[component] == 0.85, f'{self.controller.r_ref}'
            assert self.controller.r_curr[component] == 0.75
            mock_send.assert_called_once()
        
        # 5. Component deactivation
        event.content = "deactivate"
        self.controller.receive_event(event)
        
        # Verify cleanup
        assert component not in self.controller.invocations
        assert component not in self.controller.freq
        assert component not in self.controller.r_curr
        assert component not in self.controller.r_ref
    
    def test_error_handling_graceful(self):
        """Test graceful error handling"""
        # Malformed strategy
        strategy = Strategy()
        strategy.content = "invalid:format"
        
        try:
            self.controller.receive_strategy(strategy)
            assert True  # Should not crash
        except Exception as e:
            pytest.fail(f"Should handle gracefully: {e}")
        
        # Missing component
        try:
            self.controller.apply_reli_strategy("/nonexistent")
            assert True  # Should not crash
        except Exception as e:
            pytest.fail(f"Should handle gracefully: {e}")
        
        # Empty response
        try:
            self.controller._process_status_response("")
            assert True  # Should not crash
        except Exception as e:
            pytest.fail(f"Should handle gracefully: {e}")
    
    def test_g4t1_special_case_handling(self):
        """Test G4T1 special case handling"""
        component = "/g4t1"
        
        # Activate G4T1 component
        event = Event()
        event.source = component
        event.content = "activate"
        event.freq = 1.0
        self.controller.receive_event(event)
        
        # Check G4T1 was activated
        assert component in self.controller.invocations
        assert component in self.controller.freq
        assert self.controller.freq[component] == 1.0
        
        # Test frequency adaptation for G4T1
        self.controller.r_curr[component] = 0.5
        self.controller.r_ref[component] = 0.9
        self.controller.freq[component] = 50.0
        self.controller.exception_buffer[component] = 0
        self.controller.kp_individual[component] = 10.0
        self.controller.invocations[component] = deque(maxlen=100)
        
        with patch.object(self.controller, '_send_adaptation_command') as mock_send:
            self.controller.apply_reli_strategy(component)
            
            # G4T1 should have different frequency behavior
            assert self.controller.freq[component] > 40.0
        
