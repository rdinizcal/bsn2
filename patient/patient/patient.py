import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from bsn_interfaces.srv import PatientData

import threading
import random 


class Patient(Node):

    def __init__(self):
        super().__init__('patient')
        self.srv = self.create_service(PatientData, 'get_sensor_reading', self.get_data)

        self.transition_matrix = [
                [0.84,0.15,0.01],
                [0.02,0.95,0.03],
                [0.01,0.15,0.84]
                ]
        self.states = {
                0: [35,37],
                1: [37,39],
                2: [39,41]
            }
        self.curr_state=1

        self.curr_datapoint=0

    def gen_data(self):
        # transition
        x = random.random()
        prev_state = self.curr_state
        prob_array = self.transition_matrix[self.curr_state]
        if x <= prob_array[0]:
            self.curr_state = 0
        elif x <= prob_array[0] + prob_array[1]:
            self.curr_state = 1
        else:
            self.curr_state = 2
        
        if self.curr_state != prev_state:
            self.get_logger().info('State transition: %d -> %d' % (prev_state, self.curr_state))

        # generate data based on current state
        self.curr_datapoint = random.uniform(self.states[self.curr_state][0],self.states[self.curr_state][1])
        self.get_logger().info('Generated %.2f°C' % self.curr_datapoint)

        return

    def get_data(self, request, response):
        response.datapoint = float(self.curr_datapoint)
        self.get_logger().info('Request: (vital_sign: %s)\nResponse: (datapoint: %.4f)' % (request.vital_sign,response.datapoint))
        return response

def main(args=None):
    rclpy.init(args=args)

    patient = Patient()

    # Run spin in a thread, make thread daemon so we don't have to join it to exit
    thread = threading.Thread(target=rclpy.spin, args=(patient, ), daemon=True)
    thread.start()

    rate = patient.create_rate(5) # 1 Hz

    while rclpy.ok():
        patient.gen_data()
        rate.sleep()

    sensor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()