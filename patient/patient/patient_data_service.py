import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy
import threading
import random
import time
from patient_interfaces.srv import PatientDataService


class Patient(Node):
    def __init__(self):
        super().__init__('patient_data_service')
        
        # Parameters matching C++ behavior
        self.declare_parameter('normal_temp_min', 36.5)
        self.declare_parameter('normal_temp_max', 37.5) 
        self.declare_parameter('high_risk_temp_min', 39.0)
        self.declare_parameter('high_risk_temp_max', 41.0)
        self.declare_parameter('state_change_probability', 0.6)  # 60% chance
        self.declare_parameter('check_interval', 2.0)  # Check state every 2 seconds
        
        # Get parameters
        self.normal_min = self.get_parameter('normal_temp_min').value
        self.normal_max = self.get_parameter('normal_temp_max').value
        self.high_min = self.get_parameter('high_risk_temp_min').value
        self.high_max = self.get_parameter('high_risk_temp_max').value
        self.state_change_prob = self.get_parameter('state_change_probability').value
        self.check_interval = self.get_parameter('check_interval').value
        
        # Global state to track normal vs. high-risk (matching C++)
        self.is_high_risk = False
        
        # QoS Profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # Create service (matching C++ advertiseService)
        self.service = self.create_service(
            PatientDataService,
            'getPatientData',
            self.mock_patient_data_service
        )
        
        # Timer for periodic state changes
        self.state_timer = self.create_timer(
            self.check_interval,
            self.check_state_change
        )
        
        # Random number generators (matching C++ setup)
        self.rng = random.Random()
        self.rng.seed()
        
        self.get_logger().info("Patient data service ready to provide dynamic data...")
        self.get_logger().info(f"Normal range: {self.normal_min}-{self.normal_max}°C")
        self.get_logger().info(f"High-risk range: {self.high_min}-{self.high_max}°C")
        self.get_logger().info(f"State change probability: {self.state_change_prob * 100}%")
    
    def generate_temperature(self):
        """Generate temperature based on current state (matching C++ generateTemperature)"""
        if self.is_high_risk:
            # High-risk range: 39.0-41.0°C
            temperature = self.rng.uniform(self.high_min, self.high_max)
        else:
            # Normal range: 36.5-37.5°C  
            temperature = self.rng.uniform(self.normal_min, self.normal_max)
        
        return round(temperature, 2)
    
    def check_state_change(self):
        """Periodically check if state should change (matching C++ state_change logic)"""
        # 60% chance of switching states (matching C++ bernoulli_distribution)
        if self.rng.random() < self.state_change_prob:
            self.is_high_risk = not self.is_high_risk
            status = "HIGH RISK" if self.is_high_risk else "NORMAL"
            self.get_logger().info(f"Patient state changed: Now {status}")
    
    def mock_patient_data_service(self, request, response):
        """
        Callback function for the 'getPatientData' service
        (matching C++ mockPatientDataService)
        """
        try:
            if request.vital_sign == "temperature":
                # Generate dynamic temperature (matching C++ behavior)
                temperature = self.generate_temperature()
                response.data = temperature
                
                # Log with risk level indication
                risk_status = " [HIGH RISK]" if self.is_high_risk else " [NORMAL]"
                self.get_logger().info(
                    f"Returning simulated temperature: {temperature:.2f}°C{risk_status}"
                )
                
                response.success = True
                return response
            
            elif request.vital_sign == "heart_rate":
                # Optional: Add heart rate simulation
                if self.is_high_risk:
                    heart_rate = self.rng.uniform(90, 120)  # Elevated heart rate
                else:
                    heart_rate = self.rng.uniform(60, 90)   # Normal heart rate
                
                response.data = round(heart_rate, 1)
                response.success = True
                self.get_logger().info(f"Returning simulated heart rate: {response.data} bpm")
                return response
            
            elif request.vital_sign == "blood_pressure":
                # Optional: Add blood pressure simulation
                if self.is_high_risk:
                    bp_systolic = self.rng.uniform(140, 180)  # Elevated BP
                else:
                    bp_systolic = self.rng.uniform(110, 140)  # Normal BP
                
                response.data = round(bp_systolic, 1)
                response.success = True
                self.get_logger().info(f"Returning simulated blood pressure: {response.data} mmHg")
                return response
            
            else:
                # If the vital sign is not recognized (matching C++ return false)
                self.get_logger().warn(f"Unrecognized vital sign requested: {request.vital_sign}")
                response.success = False
                response.data = 0.0
                return response
                
        except Exception as e:
            self.get_logger().error(f"Error in service callback: {e}")
            response.success = False
            response.data = 0.0
            return response
    
    def spin_patient(self):
        """Keep the patient service running (matching C++ ros::spin)"""
        try:
            while rclpy.ok():
                time.sleep(0.1)  # Small sleep to prevent high CPU usage
        except KeyboardInterrupt:
            self.get_logger().info("Patient service shutting down...")


def main(args=None):
    rclpy.init(args=args)

    patient = Patient()

    # Run spin in a thread, make thread daemon so we don't have to join it to exit
    thread = threading.Thread(target=rclpy.spin, args=(patient,), daemon=True)
    thread.start()

    try:
        patient.spin_patient()
    finally:
        patient.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
