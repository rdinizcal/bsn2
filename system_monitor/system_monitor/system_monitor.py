import rclpy
from rclpy.node import Node
from diagnostic_updater import Updater, DiagnosticStatusWrapper, Heartbeat
import time # For simulation of work

class MyHeartbeatNode(Node):
    def __init__(self):
        super().__init__('my_heartbeat_node')
        self.get_logger().info("MyHeartbeatNode has been started!")

        # 1. Create a Diagnostic Updater
        # The Updater manages a list of diagnostic tasks and publishes them periodically.
        # The period (e.g., 1.0 second) is the rate at which diagnostics are published.
        self.updater = Updater(self, period=1.0)

        # Optional: Set a hardware ID for better organization in rqt_robot_monitor
        self.updater.setHardwareID("my_robot_platform_serial_123")

        # 2. Add a Heartbeat Diagnostic Task
        # The Heartbeat task simply reports OK if the updater is being updated regularly.
        # It's a simple way to confirm the node is running and its update loop is active.
        self.heartbeat_task = Heartbeat()
        self.updater.add(self.heartbeat_task)

        # 3. Add a custom diagnostic task (optional, for more detailed state)
        # You can define a function that will be called by the updater
        # to collect custom diagnostic information.
        self.custom_value = 0
        self.updater.add(
            'Custom Value Status', # Name of the diagnostic item
            self.check_custom_value # The callback function
        )
        self.create_timer(0.5, self.update_custom_value) # Timer to update custom value


        # Example: a timer that simulates work in your node
        self.timer = self.create_timer(0.2, self.timer_callback)

    def check_custom_value(self, stat: DiagnosticStatusWrapper):
        """
        A callback function for a custom diagnostic task.
        It updates the status based on the custom_value.
        """
        if self.custom_value < 10:
            stat.summary(DiagnosticStatusWrapper.OK, f"Custom value is good: {self.custom_value}")
        elif self.custom_value < 20:
            stat.summary(DiagnosticStatusWrapper.WARN, f"Custom value is increasing: {self.custom_value}")
        else:
            stat.summary(DiagnosticStatusWrapper.ERROR, f"Custom value is too high: {self.custom_value}")

        stat.add('Current Custom Value', str(self.custom_value))
        return stat

    def update_custom_value(self):
        self.custom_value += 1
        if self.custom_value > 25:
            self.custom_value = 0 # Reset to simulate normal operation cycle

    def timer_callback(self):
        # Simulate some work being done by the node
        # IMPORTANT: Call updater.update() frequently to keep diagnostics fresh
        self.updater.update()
        # self.get_logger().info(f"Node doing work and updating diagnostics...")

def main(args=None):
    rclpy.init(args=args)
    node = MyHeartbeatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()