SENSOR_NODE = {  
    'thermometer': '/thermometer_node',    # /g3t1_3
    'oximeter': '/oximeter_node',          # /g3t1_1
    'ecg': '/ecg_node',                    # /g3t1_2
    'abps': '/abps_node',                  # /g3t1_4
    'abpd': '/abpd_node',                  # /g3t1_5
    'glucometer': '/glucometer_node'       # /g3t1_6
}
FULL_SYSTEM = ['/collector', '/param_adapter',
               '/thermometer_node', '/oximeter_node', '/ecg_node', 
               '/abps_node', '/abpd_node', '/glucometer_node', 
               '/central_hub_node']
REDUCED_SYSTEM = ['/collector', '/param_adapter','/thermometer_node', '/central_hub_node', '/patient_data_service']

PERSISTENCE_NODES = [
    "/central_hub_node",
    "/node_monitor",
    #"/param_adapter",
    "/thermometer_node",
    #"/data_access",
    "/logger"
]
PERSISTANCE_TOPICS = [
        '/thermometer_data',
        '/collect_energy_status',
        '/persist',
        '/log_energy_status',
        '/TargetSystemData'
    ]
SENSOR_TOPICS = [
        '/thermometer_data',
        '/oximeter_data',
        '/ecg_data',
        '/abps_data',
        '/abpd_data',
        '/glucometer_data',
]
NON_SENSOR_TOPICS = [
        '/collect_energy_status',
        '/persist',
        '/log_energy_status',
        '/TargetSystemData'
]