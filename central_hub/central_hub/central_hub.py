import rclpy
import threading
import time
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from lifecycle_msgs.msg import State as LifecycleState
from bsn_interfaces.msg import SensorData, TargetSystemData, EnergyStatus, Event, Status
from std_msgs.msg import Header, Float32
from system_monitor.battery import Battery  # Import Battery class


class CentralHub(LifecycleNode):  # Change from Node to LifecycleNode

    def __init__(self):
        super().__init__("central_hub")
        
        # Battery parameters for central hub
        self.declare_parameter("battery_id", "hub_battery")
        self.declare_parameter("frequency", 2.0)
        self.declare_parameter("battery_capacity", 100.0)  
        self.declare_parameter("battery_level", 100.0)
        self.declare_parameter("battery_unit", 0.001)      
        self.declare_parameter("instant_recharge", False)  
        
        # Initialize battery
        battery_id = self.get_parameter("battery_id").value
        battery_capacity = float(self.get_parameter("battery_capacity").value)
        battery_level = float(self.get_parameter("battery_level").value)
        battery_unit = float(self.get_parameter("battery_unit").value)
        
        self.frequency = float(self.get_parameter("frequency").value)
        self.battery = Battery(
            id_name=battery_id,
            capacity=battery_capacity,
            current_level=battery_level,
            unit=battery_unit
        )
        
        self.instant_recharge = self.get_parameter("instant_recharge").value
        self.cost = 0.0  # Track cost for energy status reporting
        
        # Initialize with inactive state for lifecycle compliance
        self.active = False  # Start as inactive until activated through lifecycle
        self._finalized = False  # Flag for shutdown handling
        
        # Set up status publisher as None initially (will be created on configure)
        self.status_pub = None
        self.current_task = "idle"
        
        # Create energy status publisher
        self.energy_status_pub = self.create_publisher(
            EnergyStatus, 'collect_energy_status/central_hub_node', 10
        )
        
        # Create event publisher
        self.event_pub = self.create_publisher(Event, 'collect_event', 10)
        
        # Create timer for status updates and recharging
        self.timer = self.create_timer(1.0, self.check_battery_status)
        
        # Set up heartbeat timer as None initially (will be created on activate)
        self.heartbeat_timer = None
        
        # Data structures for tracking sensors
        self.NOT_USED = ""
        self.latest_risk = {
            "abpd": 0.0,
            "abps": 0.0,
            "ecg": 0.0,
            "glucosemeter": 0.0,
            "oximeter": 0.0,
            "thermometer": 0.0,
        }
        
        self.sensor_battery_levels = {
            "abpd": 100.0,
            "abps": 100.0,
            "ecg": 100.0,
            "glucosemeter": 100.0,
            "oximeter": 100.0,
            "thermometer": 100.0,
        }
        
        self.latest_data = {
            "abpd": -1.0,
            "abps": -1.0,
            "ecg": -1.0,
            "glucosemeter": -1.0,
            "oximeter": -1.0,
            "thermometer": -1.0,
        }

        self.latest_risks_labels = {
            "abpd": "unknown",
            "abps": "unknown",
            "ecg": "unknown",
            "glucosemeter": "unknown",
            "oximeter": "unknown",
            "thermometer": "unknown",
        }

    # Lifecycle Node callback methods
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Callback when transitioning to Configured state."""
        self.get_logger().info("Configuring Central Hub...")
        
        # Add status publisher
        self.status_pub = self.create_publisher(
            Status, 'component_status', 10
        )
        
        # Setup subscriptions
        self.sub_abpd = self.create_subscription(
            SensorData, "sensor_data/abpd", self.receive_datapoint, 10
        )
        self.sub_abps = self.create_subscription(
            SensorData, "sensor_data/abps", self.receive_datapoint, 10
        )
        self.sub_ecg = self.create_subscription(
            SensorData, "sensor_data/ecg", self.receive_datapoint, 10
        )
        self.sub_glucosemeter = self.create_subscription(
            SensorData, "sensor_data/glucosemeter", self.receive_datapoint, 10
        )
        self.sub_oximeter = self.create_subscription(
            SensorData, "sensor_data/oximeter", self.receive_datapoint, 10
        )
        self.sub_thermometer = self.create_subscription(
            SensorData, "sensor_data/thermometer", self.receive_datapoint, 10
        )
        
        # Setup publisher
        self.target_system_publisher = self.create_publisher(
            TargetSystemData, "target_system_data", 10
        )
        
        # Publish configured status
        self.publish_status("configured", "idle")
        
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating Central Hub...")
        
        self.active = True
        # Start heartbeat timer and publish immediate activate event
        if self.heartbeat_timer is None:
            self.heartbeat_timer = self.create_timer(2.0, self.publish_heartbeat)
        # Publish activation event immediately
        self.publish_event("activate")
        self.publish_status("activated", "idle")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating Central Hub...")
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
        self.get_logger().info("Cleaning up Central Hub...")
        self.sub_abpd = None
        self.sub_abps = None
        self.sub_ecg = None
        self.sub_glucosemeter = None
        self.sub_oximeter = None
        self.sub_thermometer = None
        self.target_system_publisher = None
        self.publish_status("unconfigured", "idle")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down Central Hub...")
        self.active = False
        self._finalized = True
        if self.heartbeat_timer is not None:
            self.heartbeat_timer.cancel()
            self.heartbeat_timer = None
        return TransitionCallbackReturn.SUCCESS
        
    # Status and event methods
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
        
    def publish_heartbeat(self):
        """Publish a heartbeat event indicating current activity state"""
        msg = Event()
        msg.source = self.get_name()
        msg.target = "system"
        msg.freq = self.frequency  # 2Hz operation from main()
        msg.content = "activate" if self.active else "deactivate"
        self.event_pub.publish(msg)
        self.get_logger().debug(f"Heartbeat event published: {msg.content} at {msg.freq}Hz")

    def publish_event(self, event_type):
        """Publish a one-time event like activate or deactivate"""
        msg = Event()
        msg.source = self.get_name()
        msg.target = "system"
        msg.content = event_type
        msg.freq = self.frequency  # 2Hz operation from main()
        self.event_pub.publish(msg)
        self.get_logger().info(f"Event published: {event_type}")
    
    # Battery and energy methods
    def is_active(self):
        """Check if hub is active based on active flag"""
        return self.active
    
    def turn_on(self):
        """Activate the hub"""
        self.active = True
        self.get_logger().info("Central Hub activated")
        self.publish_event("activate")
        
    def turn_off(self):
        """Deactivate the hub"""
        self.active = False
        self.get_logger().info("Central Hub deactivated due to low battery")
        self.publish_event("deactivate")
    
    def recharge(self):
        """Recharge the battery"""
        if not self.instant_recharge:
            # Recover 5% per second (100% in 20 seconds)
            
            self.battery.generate((100.0/20.0)/self.frequency)
        else:
            self.battery.generate(100)  # Instantly recharge to full
            
    def send_energy_status(self, cost):
        """Send energy status to monitoring system"""
        msg = EnergyStatus()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = self.get_name()
        msg.target = "system"
        msg.content = f"energy:{self.battery.current_level:.2f}:cost:{cost:.4f}"
        
        self.energy_status_pub.publish(msg)
        self.get_logger().debug(f"Energy status: {msg.content}")
        
    def check_battery_status(self):
        """Check and manage battery status"""
        # Turn on if charged enough, turn off if too low
        if not self.is_active() and self.battery.current_level > 90:
            self.turn_on()
        elif self.is_active() and self.battery.current_level < 2:
            self.turn_off()
            
        # Recharge if inactive or when instant_recharge is enabled (always plugged in)
        if not self.is_active() or self.instant_recharge:
            self.recharge()
        
        # Log battery status
        self.get_logger().info(f"Hub battery level: {self.battery.current_level:.1f}%")

    # Main functionality methods
    def receive_datapoint(self, msg):
        """Receive data from sensors"""
        # Skip if not active
        if not self.active:
            return
            
        self.publish_status(self.active and "activated" or "deactivated", "receive")
            
        if msg.sensor_type == "":
            self.get_logger().debug(
                f"null value received: {msg.sensor_type} with {msg.sensor_datapoint}"
            )
        else:
            # Update the latest data and risk for the corresponding sensor type
            self.latest_data[msg.sensor_type] = msg.sensor_datapoint
            self.latest_risks_labels[msg.sensor_type] = msg.risk_level
            self.latest_risk[msg.sensor_type] = msg.risk
            
            # Capture battery level from sensor
            self.sensor_battery_levels[msg.sensor_type] = getattr(msg, 'battery_level', 100.0)

            # Use a small amount of battery to receive data (using reference value)
            self.battery.consume(0.001) 
            self.cost += 0.001

            self.get_logger().info(
                f"Received data from {msg.sensor_type}: {msg.sensor_datapoint} with risk: {msg.risk_level}"
            )
            
        self.publish_status(self.active and "activated" or "deactivated", "idle")
            
    def data_fuse(self):
        """Calculate patient status using the original BSN fusion algorithm"""
        # Skip if not active
        if not self.active:
            return 0.0
            
        self.publish_status(self.active and "activated" or "deactivated", "calculate")
        
        # Use battery consumption from reference code
        data_count = sum(1 for risk in self.latest_risk.values() if risk >= 0)
        self.battery.consume(data_count * 0.001)
        self.cost += data_count * 0.001
        
        # Original data fusion algorithm implementation
        sensor_types = [
            "thermometer", "ecg", "oximeter", "abps", "abpd", "glucosemeter",
        ]

        # Get risk values in order
        packets_received = []
        for sensor_type in sensor_types:
            packets_received.append(self.latest_risk.get(sensor_type, -1.0))

        # Special handling and calculations
        average = 0.0
        count = 0
        index = 0
        bpr_avg = 0.0
        values = []

        for risk in packets_received:
            if risk >= 0:
                # Special handling for blood pressure sensors
                if index == 3 or index == 4:  # abps or abpd
                    bpr_avg += risk
                else:
                    average += risk
                    values.append(risk)
                count += 1

            # Process blood pressure after both readings
            if index == 4 and bpr_avg >= 0.0:
                bpr_avg /= 2  # Average of systolic and diastolic
                average += bpr_avg
                values.append(bpr_avg)

            index += 1

        if count == 0:
            self.publish_status(self.active and "activated" or "deactivated", "idle")
            return 0.0  # No data

        # Calculate average
        avg = average / count

        # Calculate deviations
        deviations = []
        min_dev = float("inf")
        max_dev = -float("inf")

        for value in values:
            # Use actual deviation (not absolute) as per reference
            dev = value - avg
            deviations.append(dev)

            if dev > max_dev:
                max_dev = dev
            if dev < min_dev:
                min_dev = dev

        # Calculate weighted average
        weighted_average = 0.0
        weight_sum = 0.0

        # If all values are the same, return simple average
        if max_dev - min_dev <= 0.0:
            self.publish_status(self.active and "activated" or "deactivated", "idle")
            return avg

        # Otherwise calculate weighted average based on deviations
        for i in range(len(values)):
            # Normalize deviation to 0-1 range
            norm_dev = (deviations[i] - min_dev) / (max_dev - min_dev)
            weight_sum += norm_dev
            weighted_average += values[i] * norm_dev

        # Final weighted risk status
        if weight_sum > 0:
            risk_status = weighted_average / weight_sum
        else:
            risk_status = avg

        self.publish_status(self.active and "activated" or "deactivated", "idle")
        return risk_status

    def format_log_message(self):
        """Format sensor data into a readable table"""
        # Consume minimal battery for formatting (using reference value)
        self.battery.consume(0.001)
        self.cost += 0.001
        
        log_message = " \n"
        border = "+-----------------+-----------------+-----------------------------+"
        header = "| {0:<15} | {1:<15} | {2:<27} |".format(
            "Vital Sign", "Value", "Risk Level"
        )

        log_message += border + "\n"
        log_message += header + "\n"
        log_message += border + "\n"

        # Iterate over the latest data to process received data
        for signal, value in self.latest_data.items():
            if value == self.NOT_USED:
                continue
            if value == -1:  # Insufficient data for moving average
                log_message += f"| {signal:<15} | waiting data      |                           |\n"
                continue
            else:
                risk = self.latest_risks_labels[signal]
                unit = ""
                if signal == "thermometer":
                    unit = "°C"
                elif signal in ["abpd", "abps"]:
                    unit = "mmHg"
                elif signal == "ecg":
                    unit = "bpm"
                elif signal == "glucosemeter":
                    unit = "mg/dL"
                elif signal == "oximeter":
                    unit = "% SpO2"

                log_message += (
                    f"| {signal:<15} | {value:>7.2f} {unit:<7} | Risk: {risk:<21} |\n"
                )

        log_message += (
            "+-----------------+-----------------+-----------------------------+"
        )
        return log_message

    def emit_alert(self, patient_status):
        """Emit alerts based on patient status and sensor risk levels"""
        # Skip if not active
        if not self.active or patient_status <= 0:
            return
            
        self.publish_status(self.active and "activated" or "deactivated", "emit_emergency")
        
        # Categorize patient status
        if patient_status <= 20.0:
            risk_category = "VERY LOW RISK"
        elif 20.0 < patient_status <= 40.0:
            risk_category = "LOW RISK"
        elif 40.0 < patient_status <= 60.0:
            risk_category = "MODERATE RISK"
        elif 60.0 < patient_status <= 80.0:
            risk_category = "CRITICAL RISK"
            self.get_logger().fatal(
                f"[Emergency Detection]\n SYSTEM ALERT: {risk_category} - Patient Status: {patient_status:.1f}%\n"
            )
        elif 80.0 < patient_status <= 100.0:
            risk_category = "VERY CRITICAL RISK"
            self.get_logger().fatal(
                f"[Emergency Detection]\n SYSTEM ALERT: {risk_category} - Patient Status: {patient_status:.1f}%\n"
            )
        else:
            risk_category = "UNKNOWN RISK"
        
        # Log moderate/low risk (critical alerts already handled above)
        if 40.0 < patient_status <= 60.0:
            self.get_logger().warning(
                f"[Emergency Detection]\n SYSTEM WARNING: {risk_category} - Patient Status: {patient_status:.1f}%\n"
            )
        elif patient_status <= 40.0 and patient_status > 0:
            self.get_logger().info(
                f"[Emergency Detection]\n System Status: {risk_category} - {patient_status:.1f}%\n"
            )
            
        self.publish_status(self.active and "activated" or "deactivated", "idle")

    def detect(self):
        """Main detection function that processes and publishes health data"""
        # Skip processing if hub is not active
        if not self.is_active():
            self.recharge()
            return
            
        try:
            # Create a TargetSystemData message
            msg = TargetSystemData()

            # Add header
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "central_hub"
            msg.header = header

            patient_status = self.data_fuse()

            # Display formatted log message
            log_message = self.format_log_message()
            self.get_logger().info(log_message)

            # Emit alerts for high/moderate risks
            self.emit_alert(patient_status)

            # Populate the message fields
            msg.trm_data = self.latest_data.get("thermometer", -1.0)
            msg.ecg_data = self.latest_data.get("ecg", -1.0)
            msg.oxi_data = self.latest_data.get("oximeter", -1.0)
            msg.abps_data = self.latest_data.get("abps", -1.0)
            msg.abpd_data = self.latest_data.get("abpd", -1.0)
            msg.glc_data = self.latest_data.get("glucosemeter", -1.0)

            # Set risk values from risk_percentages
            msg.trm_risk = self.latest_risk.get("thermometer", 0.0)
            msg.ecg_risk = self.latest_risk.get("ecg", 0.0)
            msg.oxi_risk = self.latest_risk.get("oximeter", 0.0)
            msg.abps_risk = self.latest_risk.get("abps", 0.0)
            msg.abpd_risk = self.latest_risk.get("abpd", 0.0)
            msg.glc_risk = self.latest_risk.get("glucosemeter", 0.0)

            # Update battery levels from the sensor data
            msg.trm_batt = self.sensor_battery_levels.get("thermometer", 100.0)
            msg.ecg_batt = self.sensor_battery_levels.get("ecg", 100.0)
            msg.oxi_batt = self.sensor_battery_levels.get("oximeter", 100.0)
            msg.abps_batt = self.sensor_battery_levels.get("abps", 100.0)
            msg.abpd_batt = self.sensor_battery_levels.get("abpd", 100.0)
            msg.glc_batt = self.sensor_battery_levels.get("glucosemeter", 100.0)

            msg.patient_status = patient_status

            # Publish the message - consume battery for transmission (using reference value)
            self.battery.consume(0.001)
            self.cost += 0.001
            self.target_system_publisher.publish(msg)
            self.get_logger().info("Published TargetSystemData")
            
            # Report energy status after each detection cycle
            self.send_energy_status(self.cost)
            self.cost = 0.0  # Reset cost counter
            
        except Exception as e:
            self.get_logger().error(f"Error in detect(): {str(e)}")


def main(args=None):
    rclpy.init(args=args)

    emergency_detector = CentralHub()
    
    # Run spin in a thread, make thread daemon so we don't have to join it to exit
    thread = threading.Thread(
        target=rclpy.spin, args=(emergency_detector,), daemon=True
    )
    thread.start()
    rate = emergency_detector.create_rate(emergency_detector.frequency)  # 2 Hz

    try:
        # Automatically configure and activate
        emergency_detector.trigger_configure()
        time.sleep(0.5)  # Wait for configuration
        emergency_detector.trigger_activate()
        time.sleep(0.5)  # Wait for activation
        
        while rclpy.ok():
            if emergency_detector._finalized:
                break
                
            # Only run detection if hub is active
            if emergency_detector.is_active():
                emergency_detector.detect()
            else:
                # If hub is inactive, try to recharge
                emergency_detector.recharge()
                emergency_detector.get_logger().warning(
                    f"Hub in power saving mode, recharging: {emergency_detector.battery.current_level:.1f}%"
                )
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        emergency_detector.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
