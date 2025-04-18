import rclpy
import threading
from collections import deque
from rclpy.node import Node
from bsn_interfaces.msg import SensorData


class CentralHub(Node):

    def __init__(self):
        super().__init__('central_hub')
        self.sub_abpd = self.create_subscription(SensorData, 'sensor_data/abpd', self.receive_datapoint, 10)
        self.sub_abps = self.create_subscription(SensorData, 'sensor_data/abps', self.receive_datapoint, 10)
        self.sub_ecg = self.create_subscription(SensorData, 'sensor_data/ecg', self.receive_datapoint, 10)
        self.sub_glucosemeter = self.create_subscription(SensorData, 'sensor_data/glucosemeter', self.receive_datapoint, 10)
        self.sub_oximeter = self.create_subscription(SensorData, 'sensor_data/oximeter', self.receive_datapoint, 10)
        self.sub_thermometer = self.create_subscription(SensorData, 'sensor_data/thermometer', self.receive_datapoint, 10)
        
        self.window_size = 5
        self.latest_temp = None
        self.data_windows = {
            'abpd': deque(maxlen=self.window_size),
            'abps': deque(maxlen=self.window_size),
            'ecg': deque(maxlen=self.window_size),
            'glucosemeter': deque(maxlen=self.window_size),
            'oximeter': deque(maxlen=self.window_size),
            'thermometer': deque(maxlen=self.window_size),
        }

    def receive_datapoint(self, msg):
        if msg.sensor_type == '':
            self.get_logger().debug(f'null value received: {msg.sensor_type} with {msg.sensor_datapoint}')
        else:
            self.data_windows[msg.sensor_type].append(msg.sensor_datapoint)
        
    def handle_risks(self, signal: str, avg: float):
        # Each field: signal (15), value (15), risk (20)
        if signal == 'thermometer':
            if avg < 32.0 or avg > 50.0:
                risk = 'high'
            elif 36.0 <= avg <= 37.99:
                risk = 'normal'
            else:
                risk = 'moderate'
            return risk, f"| {signal:<15} | {avg:>7.2f} °C      | Risk: {risk:<21} |\n"

        elif signal == 'abpd':
            if avg > 90:
                risk = 'high'
            elif 80 <= avg < 90:
                risk = 'moderate'
            else:
                risk = 'normal'
            return risk, f"| {signal:<15} | {avg:>7.2f} mmHg    | Risk: {risk:<21} |\n"

        elif signal == 'abps':
            if avg >= 140:
                risk = 'high'
            elif 120 <= avg < 140:
                risk = 'moderate'
            else:
                risk = 'normal'
            return risk, f"| {signal:<15} | {avg:>7.2f} mmHg    | Risk: {risk:<21} |\n"

        elif signal == 'ecg':
            if avg > 115 or avg < 70:
                risk = 'high'
            elif 85 <= avg <= 97:
                risk = 'normal'
            else:
                risk = 'moderate'
            return risk, f"| {signal:<15} | {avg:>7.2f} bpm     | Risk: {risk:<21} |\n"

        elif signal == 'glucosemeter':
            if avg < 40 or avg >= 120:
                risk = 'high'
            elif 55 <= avg < 96:
                risk = 'normal'
            else:
                risk = 'moderate'
            return risk, f"| {signal:<15} | {avg:>7.2f} mg/dL   | Risk: {risk:<21} |\n"

        elif signal == 'oximeter': 
            if avg <= 55:
                risk = 'high'
            elif 55 < avg <= 65:
                risk = 'moderate'
            else:
                risk = 'normal'
            return risk, f"| {signal:<15} | {avg:>7.2f}% SpO2   | Risk: {risk:<21} |\n"

        else:
            if avg < 30 or avg > 180:
                risk = 'high'
            elif 30 <= avg <= 50 or 130 <= avg <= 180:
                risk = 'moderate'
            else:
                risk = 'normal'
            return risk, f"| {signal:<15} | {avg:>7.2f}         | Risk: {risk:<21} |\n"


    
    def fuse_data(self):
        risk_levels = {}
        log_message = " \n"
        border = "+-----------------+-----------------+-----------------------------+"
        header = "| {0:<15} | {1:<15} | {2:<27} |".format("Vital Sign", "Value", "Risk Level")

        log_message += border + "\n"
        log_message += header + "\n"
        log_message += border + "\n"


        
        for signal, window in self.data_windows.items():
            if len(window) < self.window_size:
                
                log_message += f"| {signal:<15} | waiting data      |                           |\n"
                continue
            else:
                avg = sum(window) / self.window_size
                risk, log = self.handle_risks(signal, avg)
                log_message += log
                risk_levels[signal] = risk
        log_message += "+-----------------+-----------------+-----------------------------+"
        self.get_logger().info(log_message)
        return risk_levels

    
    def emit_alert(self):
        risks = self.fuse_data()
        for signal, level in risks.items():
            if level == 'high':
                self.get_logger().fatal(f'[Emergency Detection]\n ALERT: High risk in {signal} detected!\n')
            elif level == 'moderate':
                self.get_logger().warning(f'[Emergency Detection]\n Warning: Abnormal {signal} levels.\n')


def main(args=None):
    rclpy.init(args=args)

    emergency_detector = CentralHub()

    # Run spin in a thread, make thread daemon so we don't have to join it to exit
    thread = threading.Thread(target=rclpy.spin, args=(emergency_detector, ), daemon=True)
    thread.start()
    rate = emergency_detector.create_rate(2) # 1 Hz

    while rclpy.ok():
        emergency_detector.fuse_data()
        emergency_detector.emit_alert()
        rate.sleep()

    sensor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()