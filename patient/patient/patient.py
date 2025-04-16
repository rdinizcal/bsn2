import threading
import random
import rclpy
from rclpy.node import Node
from bsn_interfaces.srv import PatientData



class Patient(Node):

    def __init__(self):
        super().__init__('patient')

        # Declare global frequency
        self.declare_parameter('frequency', 1.0)
        self.frequency = self.get_parameter('frequency').value
        
        # Declare vital sign list
        self.declare_parameter('vitalSigns', ['temperature', 'abps', 'abpd', 'heart_rate', 'glucose', 'oxigenation'])
        self.vital_signs = self.get_parameter('vitalSigns').value
        
        self.change_rates = {}
        self.offsets = {}
        self.states = {}
        self.risk_ranges = {}

        for vital in self.vital_signs:
            # Declare change rate and offset
            self.declare_parameter(f'{vital}_Change', 1.0)
            self.declare_parameter(f'{vital}_Offset', 0.0)

            self.change_rates[vital] = self.get_parameter(f'{vital}_Change').value
            self.offsets[vital] = self.get_parameter(f'{vital}_Offset').value
            self.states[vital] = self._gen_sensor_state_matrix(vital)
            self.risk_ranges[vital] = self._set_up_sensor_risk_ranges(vital)


        self.vital_states = {key: 1 for key in self.vital_signs}
        self.vital_datapoints = {key: 0.0 for key in self.vital_signs}
        '''
         
        self.transition_matrix = [
                [0.84,0.15,0.01],
                [0.02,0.95,0.03],
                [0.01,0.15,0.84]
                ]
        self.states = {
            'temperature': {0: [35, 37], 1: [37, 39], 2: [39, 41]},
            'abps':        {0: [90, 110], 1: [111, 130], 2: [131, 150]},
            'abpd':        {0: [60, 75],  1: [76, 85],   2: [86, 95]},
            'heart_rate':  {0: [60, 80],  1: [81, 100],  2: [101, 120]},
            'glucose':     {0: [70, 100], 1: [101, 125], 2: [126, 160]},
            'oxigenation': {0: [95, 97],  1: [92, 94],   2: [88, 91]}
        }
        self.vital_states = {key: 1 for key in self.states}
        self.vital_datapoints = {key: 0.0 for key in self.states}
        
        '''
        self.srv = self.create_service(PatientData, 'get_sensor_reading', self.get_data)
    def _gen_sensor_state_matrix(self, vital: str):
        state_dict = {}
        for i in range(5):
            self.declare_parameter(f'{vital}_State{i}', [0.0, 0.0, 0.0, 0.0, 0.0])
            state_dict[i] = self.get_parameter(f'{vital}_State{i}').value
        state_dict = {}
        return state_dict
    
    def _set_up_sensor_risk_ranges(self, vital: str):
        risks = {}
        for label in ['LowRisk', 'MidRisk0', 'MidRisk1', 'HighRisk0', 'HighRisk1']:
            param_name = f'{vital}_{label}'
            self.declare_parameter(param_name, [-1.0, -1.0])
            risks[label] = self.get_parameter(param_name).value
        return risks

    def gen_data(self):
        for vital_sign, state_ranges in self.states.items():
            x = random.random()
            prev_state = self.vital_states[vital_sign]
            probs = self.transition_matrix[prev_state]

            if x <= probs[0]:
                self.vital_states[vital_sign] = 0
            elif x <= probs[0] + probs[1]:
                self.vital_states[vital_sign] = 1
            else:
                self.vital_states[vital_sign] = 2

            curr_state = self.vital_states[vital_sign]

            if curr_state != prev_state:
                self.get_logger().info(f'State transition for {vital_sign}: {prev_state} -> {curr_state}')

            range_min, range_max = state_ranges[curr_state]
            self.vital_datapoints[vital_sign] = random.uniform(range_min, range_max)
            #self.get_logger().info(f'Generated {vital_sign}: {self.vital_datapoints[vital_sign]:.2f}')


    def get_data(self, request, response):
        vital = request.vital_sign.lower()
        if vital not in self.vital_datapoints:
            self.get_logger().warn(f"Requested unknown vital sign: {vital}")
            response.datapoint = -1.0
        else:
            response.datapoint = float(self.vital_datapoints[vital])
            self.get_logger().info(f'Request: ({vital}) -> {response.datapoint:.4f}')
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