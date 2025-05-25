from rclpy.node import Node
from collections import deque
import rclpy
import threading
from bsn_interfaces.srv import PatientData
from bsn_interfaces.msg import SensorData
from std_msgs.msg import Header
from diagnostic_updater import Updater, DiagnosticStatusWrapper, Heartbeat
from diagnostic_msgs.msg import DiagnosticStatus
from sensor.risk_evaluator import RiskEvaluator


class Sensor(Node):

    def __init__(self, node_name: str, parameters=None):
        super().__init__(
            node_name, parameter_overrides=parameters or []
        )  # Use the node's name as provided in the launch file
        self.declare_parameter("sensor", "")
        self.declare_parameter("vital_sign", "")
        self.declare_parameter("frequency", "1.0")

        self.sensor = self.get_parameter("sensor").get_parameter_value().string_value
        self.vital_sign = (
            self.get_parameter("vital_sign").get_parameter_value().string_value
        )
        self.get_logger().debug(
            f"vital sign in Sensor {self.get_name()}: {self.vital_sign}"
        )
        self.frequency = float(
            self.get_parameter("frequency").get_parameter_value().string_value
        )

        # Create the risk evaluator and configure it
        self.risk_evaluator = RiskEvaluator()
        self._configure_risk_evaluator()
        
        self.last_collected_datapoint = -1.0
        self.last_risk_level = "unknown"
        self.last_risk_value = -1.0
        
        self._setup_diagnostics()

        self.cli = self.create_client(PatientData, "get_sensor_reading")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        self.req = PatientData.Request()
        self.publisher_ = self.create_publisher(
            SensorData, f"sensor_data/{self.sensor}", 10
        )
        

        self.window_size = 5
        self.data_window = deque(maxlen=self.window_size)

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
    def _setup_diagnostics(self):
        """
        Sets up the diagnostic updater and registers all diagnostic tasks.
        """
        self.updater = Updater(self, period=1.0) # Publishes diagnostics every 1 second
        self.updater.setHardwareID(f"sensor_unit_{self.sensor}")

        # Add a Heartbeat task
        self.updater.add(Heartbeat())

        # Add custom diagnostic tasks
        self.updater.add(
            f"{self.sensor} Service Client Status",
            self.check_service_client_status
        )
        self.updater.add(
            f"{self.sensor} Data Window Status",
            self.check_data_window_status
        )
        self.updater.add(
            f"{self.sensor} Last Collected Data",
            self.check_last_collected_data
        )
        self.updater.add(
            f"{self.sensor} Risk Evaluation Status",
            self.check_risk_evaluation_status
        )

    # --- Diagnostic Callback Functions (keep these as separate methods) ---
    def check_service_client_status(self, stat: DiagnosticStatusWrapper):
        """Diagnostic task to report on the service client's connection status."""
        if self.cli.service_is_ready():
            stat.summary(DiagnosticStatus.OK, "Service client connected.")
        else:
            stat.summary(DiagnosticStatus.WARN, "Service client NOT connected or ready.")
        stat.add("Service Name", self.cli.srv_name)
        return stat

    def check_data_window_status(self, stat: DiagnosticStatusWrapper):
        """Diagnostic task to report on the data window's fill level."""
        current_fill = len(self.data_window)
        percentage_full = (current_fill / self.window_size) * 100

        if current_fill == self.window_size:
            stat.summary(DiagnosticStatus.OK, "Data window is full.")
        elif current_fill > 0:
            stat.summary(DiagnosticStatus.WARN, "Data window is not yet full.")
        else:
            stat.summary(DiagnosticStatus.ERROR, "Data window is empty.")

        stat.add("Current Window Size", str(current_fill))
        stat.add("Max Window Size", str(self.window_size))
        stat.add("Fill Percentage", f"{percentage_full:.1f}%")
        return stat

    def check_last_collected_data(self, stat: DiagnosticStatusWrapper):
        """Diagnostic task to report the last collected data point."""
        if self.last_collected_datapoint >= 0:
            stat.summary(DiagnosticStatus.OK, f"Last collected: {self.last_collected_datapoint:.2f}")
        else:
            stat.summary(DiagnosticStatus.WARN, "Last collection failed or no data yet.")
        stat.add("Last Data Value", f"{self.last_collected_datapoint:.2f}")
        return stat

    def check_risk_evaluation_status(self, stat: DiagnosticStatusWrapper):
        """Diagnostic task to report the last calculated risk level."""
        if self.last_risk_level == "low":
            stat.summary(DiagnosticStatus.OK, f"Last risk: {self.last_risk_level} ({self.last_risk_value:.1f}%)")
        elif self.last_risk_level == "moderate":
            stat.summary(DiagnosticStatus.WARN, f"Last risk: {self.last_risk_level} ({self.last_risk_value:.1f}%)")
        elif self.last_risk_level == "high":
            stat.summary(DiagnosticStatus.ERROR, f"Last risk: {self.last_risk_level} ({self.last_risk_value:.1f}%)")
        else:
            stat.summary(DiagnosticStatus.STALE, "Risk level unknown or not evaluated.")
        stat.add("Last Risk Level", self.last_risk_level)
        stat.add("Last Risk Value", f"{self.last_risk_value:.1f}")
        return stat    

    def collect(self):
        self.req.vital_sign = self.vital_sign
        response = self.cli.call(self.req)
        if response is None:
            self.get_logger().error("Service call failed. No response received.")
            return -1.0  # Return -1 to indicate failure
        self.get_logger().info(
            f"++Collect++\n new data from {self.req.vital_sign} collected: [{response.datapoint}]"
        )
        return response.datapoint

    def process(self, datapoint: float):
        # Add the new datapoint to the deque
        self.data_window.append(datapoint)

        # Calculate the moving average
        if len(self.data_window) == self.window_size:
            moving_avg = sum(self.data_window) / self.window_size
            self.get_logger().info(f"Moving average for {self.sensor}: {moving_avg}")
            return moving_avg
        else:
            self.get_logger().info(
                f"Insufficient data for moving average. Current window size: {len(self.data_window)}"
            )
            return -1.0  # Return -1 to indicate insufficient data

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
        msg = SensorData()

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.sensor

        msg.header = header
        msg.sensor_type = self.sensor
        msg.sensor_datapoint = datapoint

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
            f"++Transfer++\n Value: {msg.sensor_datapoint}, Risk: {msg.risk:.2f}%, Level: {msg.risk_level}"
        )

    def spin_sensor(self):
        rate = self.create_rate(self.frequency)
        while rclpy.ok():
            datapoint = self.collect()
            datapoint = self.process(datapoint)
            self.transfer(datapoint)
            self.updater.update()
            rate.sleep()


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
