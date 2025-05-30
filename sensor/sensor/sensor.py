from rclpy.node import Node
from collections import deque
import rclpy
import threading
from time import sleep
from bsn_interfaces.srv import PatientData
from bsn_interfaces.msg import SensorData, EnergyStatus, Event, Status
from std_msgs.msg import Header, String
from sensor.risk_evaluator import RiskEvaluator
from system_monitor.battery import Battery
from rclpy.lifecycle import LifecycleNode, State
from rclpy.lifecycle import TransitionCallbackReturn

class Sensor(LifecycleNode):

    def __init__(self, node_name: str, parameters=None):
        super().__init__(
            node_name, parameter_overrides=parameters or []
        )
        self.declare_parameter("sensor", "")
        self.declare_parameter("vital_sign", "")
        self.declare_parameter("frequency", "1.0")
        
        self.sensor = self.get_parameter("sensor").get_parameter_value().string_value
        self.vital_sign = (
            self.get_parameter("vital_sign").get_parameter_value().string_value
        )
        self._finalized = False 
        self.frequency = float(
            self.get_parameter("frequency").get_parameter_value().string_value
        )

        # Battery parameters
        self.declare_parameter("battery_id", f"{self.sensor}_battery")
        self.declare_parameter("battery_capacity", 100.0)
        self.declare_parameter("battery_level", 100.0)
        self.declare_parameter("battery_unit", 0.05)
        self.declare_parameter("instant_recharge", False)
        
        # Initialize battery
        battery_id = self.get_parameter("battery_id").value
        battery_capacity = float(self.get_parameter("battery_capacity").value)
        battery_level = float(self.get_parameter("battery_level").value)
        battery_unit = float(self.get_parameter("battery_unit").value)
        
        self.battery = Battery(
            id_name=battery_id,
            capacity=battery_capacity,
            current_level=battery_level,
            unit=battery_unit
        )
        
        self.instant_recharge = self.get_parameter("instant_recharge").value
        self.cost = 0.0  # Track cost for energy status reporting
        
        # Initialize with the inactive state for lifecycle compliance
        self.active = False  # Start as inactive until activated through lifecycle
        
        # Set up status publisher as None initially (will be created on configure)
        self.status_pub = None
        self.current_task = "idle"
        
        # Create energy status publisher
        self.energy_status_pub = self.create_publisher(
            EnergyStatus, f'collect_energy_status/{self.sensor}', 10
        )
        
        
        self.event_pub = self.create_publisher(Event, 'collect_event', 10)
        
        # Create timer for status updates and recharging
        self.timer = self.create_timer(1.0, self.check_battery_status)

        # Create the risk evaluator and configure it
        self.risk_evaluator = RiskEvaluator()
        self._configure_risk_evaluator()

        # Initialize service client
        sleep(1.0)  # Ensure parameters are set before creating client
        self.cli = self.create_client(PatientData, "/get_sensor_reading")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
            self.get_logger().debug(
                f'sensor: {self.sensor}, vital_sign: {self.vital_sign}, frequency: {self.frequency}'
            )

        self.req = PatientData.Request()
        self.publisher_ = self.create_publisher(
            SensorData, f'sensor_data/{self.sensor}', 10
        )
        
        self.window_size = 5
        self.data_window = deque(maxlen=self.window_size)
        
        # Set up heartbeat timer as None initially (will be created on activate)
        self.heartbeat_timer = None
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Callback when transitioning to Configured state."""
        self.get_logger().debug("CONFIGURING SENSOR")
        
        # Add status publisher
        self.status_pub = self.create_publisher(
            Status, 'component_status', 10
        )
        
        # Publish configured status
        self.publish_status("configured", "idle")
        
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug("ACTIVATING SENSOR...")
        
        self.active = True
        # Start heartbeat timer and publish immediate activate event
        if self.heartbeat_timer is None:
            self.heartbeat_timer = self.create_timer(2.0, self.publish_heartbeat)
        # Publish activation event immediately
        self.publish_event("activate")
        self.publish_status("activated", "idle")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating sensor...")
        self.active = False
        # Stop heartbeat timer
        if self.heartbeat_timer:
            self.heartbeat_timer.cancel()
            self.heartbeat_timer = None
        if self.timer:
            self.timer.cancel()
            self.timer = None
        self.publish_event("deactivate")
        self.publish_status("deactivated", "idle")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up...")
        self.cli = None
        self.publisher_ = None
        self.energy_status_pub = None
        self.event_pub = None
        self.data_window.clear()
        self.publish_status("unconfigured", "idle")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down sensor...")
        self.active = False
        self._finalized = True 
        if self.heartbeat_timer is not None:
            self.heartbeat_timer.cancel()
            self.heartbeat_timer = None
        return TransitionCallbackReturn.SUCCESS

    
    def turn_on(self):
        """Activate the sensor"""
        self.active = True
        self.get_logger().info(f"Sensor {self.sensor} activated")
        self.publish_event("activate")

    def turn_off(self):
        """Deactivate the sensor"""
        self.active = False
        self.get_logger().info(f"Sensor {self.sensor} deactivated")
        self.publish_event("deactivate")

    # Consolidate the heartbeat and event publishing into one method
    def publish_heartbeat(self):
        """Publish a heartbeat event indicating current activity state"""
        msg = Event()
        msg.source = self.get_name()
        msg.target = "system"
        msg.freq = float(self.frequency)
        msg.content = "activate" if self.active else "deactivate"
        self.event_pub.publish(msg)
        self.get_logger().debug(f"Heartbeat event published: {msg.content} at {msg.freq}Hz")

    # For one-time events, use this method
    def publish_event(self, event_type):
        """Publish a one-time event like activate or deactivate"""
        msg = Event()
        msg.source = self.get_name()
        msg.target = "system"
        msg.content = event_type
        msg.freq = float(self.frequency)
        self.event_pub.publish(msg)
        self.get_logger().info(f"Event published: {event_type}")

    def publish_status(self, content, task):
        """Publish component status"""
        if self.status_pub is None:
            return
            
        msg = Status()
        msg.source = self.get_name()
        msg.target = "system"
        msg.content = content
        msg.task = task
        self.status_pub.publish(msg)
        self.get_logger().debug(f"Status published: {content}, task: {task}")
        self.current_task = task

    def recharge(self):
        """Recharge the battery"""
        if not self.instant_recharge:
            if self.battery.current_level <= 100:
                self.battery.generate(1)  # Generate 1 unit per cycle
        else:
            self.battery.generate(100)  # Instantly recharge to full
            
    def send_energy_status(self, cost):
        """Send energy status to monitoring system"""
        msg = EnergyStatus()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = self.get_name()
        msg.target = "system"
        msg.content = f"energy:{self.battery.current_level:.2f}:cost:{cost:.2f}"
        
        self.energy_status_pub.publish(msg)
        self.get_logger().debug(f"Energy status: {msg.content}")
        
    def check_battery_status(self):
        """Check and manage battery status"""
        # Turn on if charged enough, turn off if too low
        if not self.active and self.battery.current_level > 10:
            pass
        elif self.active and self.battery.current_level < 2:
            self.turn_off()
            
        # Recharge if inactive
        if not self.active:
            self.recharge()

    def collect(self):
        """Collect data from patient service with battery management"""
        self.publish_status(self.active and "activated" or "deactivated", "collect")
        
        # Check if active before collecting
        if not self.active:
            self.recharge()
            return -1.0
            
        # Consume battery for collection
        self.battery.consume(1.0)  # Standard unit consumption
        self.cost += 1.0
        
        # Regular collection logic
        self.req.vital_sign = self.vital_sign
        response = self.cli.call(self.req)
        if response is None:
            self.get_logger().error("Service call failed. No response received.")
            return -1.0
        self.get_logger().info(
            f"++Collect++\n new data from {self.req.vital_sign} collected: [{response.datapoint}]"
        )
        result = response.datapoint
        
        self.publish_status(self.active and "activated" or "deactivated", "idle")
        return result
        
    def process(self, datapoint: float):
        """Process data with battery consumption"""
        if not self.active or datapoint < 0:
            return -1.0
            
        self.publish_status(self.active and "activated" or "deactivated", "process")
        
        # Consume battery for processing based on window size (more complex = more battery)
        multiplier = min(1.0, self.window_size / 10.0)  # Scale down to avoid excessive consumption
        self.battery.consume(multiplier)
        self.cost += multiplier
        
        # Add the new datapoint to the deque
        self.data_window.append(datapoint)

        # Calculate the moving average
        if len(self.data_window) == self.window_size:
            moving_avg = sum(self.data_window) / self.window_size
            self.get_logger().info(f"Moving average for {self.sensor}: {moving_avg}")
            result = moving_avg
        else:
            self.get_logger().info(
                f"Insufficient data for moving average. Current window size: {len(self.data_window)}"
            )
            result = -1.0
            
        self.publish_status(self.active and "activated" or "deactivated", "idle")
        return result
        
    def assess_risk(self, avg: float):
        """Calculate risk level based on sensor type and value"""
        if self.sensor == "thermometer":
            if avg < 32.0 or avg > 50.0:
                return "high"
            elif 36.0 <= avg <= 37.99:
                return "normal"
            else:
                return "moderate"

        elif self.sensor == "abpd":
            if avg > 90:
                return "high"
            elif 80 <= avg < 90:
                return "moderate"
            else:
                return "normal"

        elif self.sensor == "abps":
            if avg >= 140:
                return "high"
            elif 120 <= avg < 140:
                return "moderate"
            else:
                return "normal"

        elif self.sensor == "ecg":
            if avg > 115 or avg < 70:
                return "high"
            elif 85 <= avg <= 97:
                return "normal"
            else:
                return "moderate"

        elif self.sensor == "glucosemeter":
            if avg < 40 or avg >= 120:
                return "high"
            elif 55 <= avg < 96:
                return "normal"
            else:
                return "moderate"

        elif self.sensor == "oximeter":
            if avg <= 55:
                return "high"
            elif 55 < avg <= 65:
                return "moderate"
            else:
                return "normal"

        else:
            if avg < 30 or avg > 180:
                return "high"
            elif 30 <= avg <= 50 or 130 <= avg <= 180:
                return "moderate"
            else:
                return "normal"

    def transfer(self, datapoint: float):
        """Transfer data with battery consumption"""
        if not self.active or datapoint < 0:
            return
            
        self.publish_status(self.active and "activated" or "deactivated", "transfer")
        
        # Consume battery for transfer
        self.battery.consume(1.0)
        self.cost += 1.0
        
        msg = SensorData()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.sensor

        msg.header = header
        msg.sensor_type = self.sensor
        msg.sensor_datapoint = datapoint
        # Include battery level in the message (you may need to extend SensorData)
        msg.battery_level = self.battery.current_level

        if datapoint >= 0:
            # Calculate numeric risk (0-100)
            risk_value = self.risk_evaluator.evaluate_risk(self.sensor, datapoint)
            msg.risk = float(risk_value)

            # Set risk level label based on the risk percentage
            if risk_value >= 0:
                msg.risk_level = self.risk_evaluator.risk_label(risk_value)
            else:
                msg.risk_level = "unknown"
        else:
            msg.risk_level = "unknown"
            msg.risk = -1.0

        self.publisher_.publish(msg)
        self.get_logger().info(
            f"++Transfer++\n Value: {msg.sensor_datapoint}, Risk: {msg.risk:.2f}%, "
            f"Level: {msg.risk_level}, Battery: {self.battery.current_level:.1f}%"
        )
        
        # Send energy status after completing an operation cycle
        self.send_energy_status(self.cost)
        self.cost = 0.0  # Reset cost after reporting
    def spin_sensor(self):
        """Main sensor loop with battery management"""
        rate = self.create_rate(self.frequency)
        while rclpy.ok():
            if hasattr(self, '_finalized') and self._finalized:
                self.get_logger().info("Node has been shut down, exiting spin loop")
                break
            elif self.active:
                try:
                    # Perform the sensor operations
                    datapoint = self.collect()
                    if datapoint >= 0:
                        datapoint = self.process(datapoint)
                        self.transfer(datapoint)
                except Exception as e:
                    self.get_logger().error(f"Sensor operation failed: {str(e)}")
            else:
                self.recharge()
                self.get_logger().info(f"Recharging battery: {self.battery.current_level:.1f}%")

            

            rate.sleep()

    def _configure_risk_evaluator(self):
        """Load risk ranges from parameters and configure the risk evaluator"""
        # Load risk percentage ranges
        self.declare_parameter("lowrisk", "0,20")
        self.declare_parameter("midrisk", "21,65")
        self.declare_parameter("highrisk", "66,100")

        low_risk_str = self.get_parameter("lowrisk").value
        mid_risk_str = self.get_parameter("midrisk").value
        high_risk_str = self.get_parameter("highrisk").value

        low_min, low_max = map(float, low_risk_str.split(","))
        mid_min, mid_max = map(float, mid_risk_str.split(","))
        high_min, high_max = map(float, high_risk_str.split(","))

        risk_percentages = [
            (low_min, low_max),
            (mid_min, mid_max),
            (high_min, high_max),
        ]

        self.declare_parameter(f"HighRisk0", "-1,-1")
        self.declare_parameter(f"MidRisk0", "-1,-1")
        self.declare_parameter(f"LowRisk", "-1,-1")
        self.declare_parameter(f"MidRisk1", "-1,-1")
        self.declare_parameter(f"HighRisk1", "-1,-1")

        high_risk0_str = self.get_parameter(f"HighRisk0").value
        mid_risk0_str = self.get_parameter(f"MidRisk0").value
        low_risk_str = self.get_parameter(f"LowRisk").value
        mid_risk1_str = self.get_parameter(f"MidRisk1").value
        high_risk1_str = self.get_parameter(f"HighRisk1").value

        # Parse ranges
        high_risk0 = tuple(map(float, high_risk0_str.split(",")))
        mid_risk0 = tuple(map(float, mid_risk0_str.split(",")))
        low_risk = tuple(map(float, low_risk_str.split(",")))
        mid_risk1 = tuple(map(float, mid_risk1_str.split(",")))
        high_risk1 = tuple(map(float, high_risk1_str.split(",")))

        # Create ranges dictionary
        sensor_ranges = {
            "high_risk0": high_risk0,
            "mid_risk0": mid_risk0,
            "low_risk": low_risk,
            "mid_risk1": mid_risk1,
            "high_risk1": high_risk1,
        }

        # Configure the risk evaluator
        self.risk_evaluator.configure(self.sensor, sensor_ranges, risk_percentages)

        self.get_logger().info(
            f"Configured risk evaluator for {self.sensor}:\n"
            f"  Risk Percentages: Low={risk_percentages[0]}, Mid={risk_percentages[1]}, High={risk_percentages[2]}\n"
            f"  Ranges: {sensor_ranges}"
        )

def main(args=None):
    rclpy.init(args=args)

    # Get node name from CLI remapping (set in the launch file)
    sensor_node = Sensor(node_name="sensor_node")

    thread = threading.Thread(target=rclpy.spin, args=(sensor_node,), daemon=True)
    thread.start()

    try:
        sensor_node.spin_sensor()
    finally:
        sensor_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
