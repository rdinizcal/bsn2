from rclpy.node import Node
from collections import deque
import rclpy
import threading
from bsn_interfaces.srv import PatientData
from bsn_interfaces.msg import SensorData
from std_msgs.msg import Header


class Sensor(Node):

    def __init__(self, node_name: str):
        super().__init__(
            node_name
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

        self.cli = self.create_client(PatientData, "get_sensor_reading")

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        self.req = PatientData.Request()
        self.publisher_ = self.create_publisher(
            SensorData, f"sensor_data/{self.sensor}", 10
        )

        self.window_size = 5
        self.data_window = deque(maxlen=self.window_size)

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
            msg.risk_level = self.assess_risk(datapoint)
        else:
            msg.risk_level = "unknown"
        self.publisher_.publish(msg)
        self.get_logger().info(f'++Transfer++\n Publishing: "{msg.sensor_datapoint}"')

    def spin_sensor(self):
        rate = self.create_rate(self.frequency)
        while rclpy.ok():
            datapoint = self.collect()
            datapoint = self.process(datapoint)
            self.transfer(datapoint)
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
