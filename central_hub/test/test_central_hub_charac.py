import pytest
import time
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from bsn_interfaces.msg import SensorData, TargetSystemData
from std_msgs.msg import Header


@pytest.fixture(scope="class")
def central_hub_node(request):
    from central_hub.central_hub import CentralHub
    
    rclpy.init()
    
    # Create node and assign to class
    node = CentralHub()
    
    # Create publishers for each se  nsor type to simulate incoming data
    publishers = {}
    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )
    
    for sensor_type in ["thermometer", "ecg", "oximeter", "abps", "abpd", "glucosemeter"]:
        publishers[sensor_type] = node.create_publisher(
            SensorData, 
            f"sensor_data/{sensor_type}", 
            qos_profile
        )
    
    # Create subscription to capture published TargetSystemData
    request.cls.received_messages = []
    
    def target_system_callback(msg):
        request.cls.received_messages.append(msg)
    
    node.create_subscription(
        TargetSystemData,
        "target_system_data",
        target_system_callback,
        qos_profile
    )
    
    # Store node and publishers in class for tests to use
    request.cls.central_hub = node
    request.cls.publishers = publishers
    
    yield node
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


@pytest.mark.usefixtures("central_hub_node")
class TestCentralHubBehavior:
    central_hub = None  # Will be set by fixture
    publishers = {}     # Will be set by fixture
    received_messages = []  # Will store published messages
    
    def publish_sensor_data(self, sensor_type, value, risk_level="normal"):
        """Helper to publish sensor data"""
        msg = SensorData()
        header = Header()
        header.stamp = self.central_hub.get_clock().now().to_msg()
        header.frame_id = sensor_type
        
        msg.header = header
        msg.sensor_type = sensor_type
        msg.sensor_datapoint = float(value)
        msg.risk_level = risk_level
        msg.risk_percentage = self._risk_level_to_percentage(risk_level)
        
        self.publishers[sensor_type].publish(msg)
        
        # Process the message
        for _ in range(5):  # Spin a few times to ensure message is processed
            rclpy.spin_once(self.central_hub, timeout_sec=0.1)
            time.sleep(0.1)
    
    def _risk_level_to_percentage(self, risk_level):
        """Convert risk level to a percentage value"""
        if risk_level == "high":
            return 80.0
        elif risk_level == "moderate":
            return 50.0
        elif risk_level == "low" or risk_level == "normal":
            return 10.0
        else:
            return -1.0
    
    def test_initialization(self):
        """Test that the central hub initializes correctly"""
        # Check initial state
        assert self.central_hub is not None
        assert hasattr(self.central_hub, "latest_data")
        assert hasattr(self.central_hub, "latest_risks")
        
        # Check that all expected sensor types are initialized
        for sensor_type in ["thermometer", "ecg", "oximeter", "abps", "abpd", "glucosemeter"]:
            assert sensor_type in self.central_hub.latest_data
            assert self.central_hub.latest_data[sensor_type] == -1.0
            assert sensor_type in self.central_hub.latest_risks
            assert self.central_hub.latest_risks[sensor_type] == "unknown"
    
    def test_receive_datapoint(self):
        """Test receiving a datapoint from a sensor"""
        # Publish a test message
        test_value = 37.0
        test_risk = "normal"
        self.publish_sensor_data("thermometer", test_value, test_risk)
        
        # Check that the data was received and stored
        assert self.central_hub.latest_data["thermometer"] == test_value
        assert self.central_hub.latest_risks["thermometer"] == test_risk
    
    def test_detect_normal_conditions(self):
        """Test detection under normal conditions"""
        # Reset received messages
        self.received_messages.clear()
        
        # Publish normal values for all sensors
        self.publish_sensor_data("thermometer", 37.0, "normal")
        self.publish_sensor_data("ecg", 90.0, "normal")
        self.publish_sensor_data("oximeter", 98.0, "normal")
        self.publish_sensor_data("abps", 110.0, "normal")
        self.publish_sensor_data("abpd", 75.0, "normal")
        self.publish_sensor_data("glucosemeter", 80.0, "normal")
        
        # Trigger the detect method
        self.central_hub.detect()
        
        # Give some time for the message to be processed
        time.sleep(0.5)
        
        # Check that a message was published
        assert len(self.received_messages) > 0
        
        # Get the latest message
        msg = self.received_messages[-1]
        
        # Verify message content
        assert msg.trm_data == 37.0
        assert msg.ecg_data == 90.0
        assert msg.oxi_data == 98.0
        assert msg.abps_data == 110.0
        assert msg.abpd_data == 75.0
        assert msg.glc_data == 80.0
        
        # All risks should be 0.0 (normal)
        assert msg.trm_risk == 0.0
        assert msg.ecg_risk == 0.0
        assert msg.oxi_risk == 0.0
        assert msg.abps_risk == 0.0
        assert msg.abpd_risk == 0.0
        assert msg.glc_risk == 0.0
    
    def test_detect_abnormal_conditions(self):
        """Test detection under abnormal conditions"""
        # Reset received messages
        self.received_messages.clear()
        
        # Publish abnormal values for some sensors
        self.publish_sensor_data("thermometer", 39.5, "moderate")
        self.publish_sensor_data("ecg", 130.0, "high")
        
        # Trigger the detect method
        self.central_hub.detect()
        
        # Give some time for the message to be processed
        time.sleep(0.5)
        
        # Check that a message was published
        assert len(self.received_messages) > 0
        
        # Get the latest message
        msg = self.received_messages[-1]
        
        # Verify message content for abnormal sensors
        assert msg.trm_data == 39.5
        assert msg.ecg_data == 130.0
        
        # Check risk values
        assert msg.trm_risk == 0.5  # moderate
        assert msg.ecg_risk == 1.0  # high
    
    def test_risk_to_numeric(self):
        """Test the risk_to_numeric method"""
        # Set up test cases
        self.central_hub.latest_risks["thermometer"] = "high"
        self.central_hub.latest_risks["ecg"] = "moderate"
        self.central_hub.latest_risks["oximeter"] = "normal"
        self.central_hub.latest_risks["abps"] = "unknown"
        
        # Check conversions
        assert self.central_hub.risk_to_numeric("thermometer") == 1.0
        assert self.central_hub.risk_to_numeric("ecg") == 0.5
        assert self.central_hub.risk_to_numeric("oximeter") == 0.0
        assert self.central_hub.risk_to_numeric("abps") == 0.0
        assert self.central_hub.risk_to_numeric("nonexistent") == 0.0
    
    def test_format_log_message(self):
        """Test the format_log_message method"""
        # Set up test data
        self.central_hub.latest_data["thermometer"] = 38.5
        self.central_hub.latest_data["ecg"] = 95.0
        self.central_hub.latest_data["oximeter"] = -1.0  # Waiting for data
        
        self.central_hub.latest_risks["thermometer"] = "moderate"
        self.central_hub.latest_risks["ecg"] = "normal"
        
        # Get the formatted log message
        log_message = self.central_hub.format_log_message()
        
        # Check that the log message contains expected information
        assert "thermometer" in log_message
        assert "38.50" in log_message
        assert "°C" in log_message
        assert "moderate" in log_message
        
        assert "ecg" in log_message
        assert "95.00" in log_message
        assert "bpm" in log_message
        assert "normal" in log_message
        
        assert "oximeter" in log_message
        assert "waiting data" in log_message
    
    def test_multiple_readings_overwrite(self):
        """Test that multiple readings for the same sensor overwrite previous values"""
        # Publish initial data
        self.publish_sensor_data("thermometer", 37.0, "normal")
        
        # Check initial value
        assert self.central_hub.latest_data["thermometer"] == 37.0
        
        # Publish new data
        self.publish_sensor_data("thermometer", 39.0, "moderate")
        
        # Check that value was updated
        assert self.central_hub.latest_data["thermometer"] == 39.0
        assert self.central_hub.latest_risks["thermometer"] == "moderate"
    
    def test_full_system_integration(self):
        """Test full system integration with multiple sensors and changing conditions"""
        # Reset received messages
        self.received_messages.clear()
        
        # Simulate normal conditions for all sensors
        for sensor_type in ["thermometer", "ecg", "oximeter", "abps", "abpd", "glucosemeter"]:
            if sensor_type == "thermometer":
                self.publish_sensor_data(sensor_type, 37.0, "normal")
            elif sensor_type == "ecg":
                self.publish_sensor_data(sensor_type, 90.0, "normal")
            elif sensor_type == "oximeter":
                self.publish_sensor_data(sensor_type, 98.0, "normal")
            elif sensor_type == "abps":
                self.publish_sensor_data(sensor_type, 110.0, "normal")
            elif sensor_type == "abpd":
                self.publish_sensor_data(sensor_type, 75.0, "normal")
            elif sensor_type == "glucosemeter":
                self.publish_sensor_data(sensor_type, 80.0, "normal")
        
        # Trigger the detect method
        self.central_hub.detect()
        time.sleep(0.5)
        
        # Now simulate a health deterioration
        self.publish_sensor_data("thermometer", 40.0, "high")
        self.publish_sensor_data("ecg", 140.0, "high")
        self.publish_sensor_data("oximeter", 85.0, "moderate")
        
        # Trigger the detect method again
        self.central_hub.detect()
        time.sleep(0.5)
        
        # We should have at least 2 messages
        assert len(self.received_messages) >= 2
        
        # Check that the last message reflects the deterioration
        last_msg = self.received_messages[-1]
        assert last_msg.trm_risk == 1.0  # high
        assert last_msg.ecg_risk == 1.0  # high
        assert last_msg.oxi_risk == 0.5  # moderate