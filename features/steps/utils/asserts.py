def count_matching_elements(list1, list2):
    matching_elements = set(list1) & set(list2)
    # Return the count of matching elements
    return len(matching_elements)

def node_is_active(node_names):
    if isinstance(node_names, str):
        node_names = [node_names]

    result = subprocess.run(['ros2', 'node', 'list'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    node_list = result.stdout.decode('utf-8').splitlines()
    print(f"node list: {node_list}")
    for node_name in node_names:
        assert node_name in node_list, f"{node_name} is not online. Make sure to give the system more time to start up."

def check_time_performance(sensor_data, target_system_data, key, value, evaluate):
    time_threshold=250000

    # Iterate over both lists and check for matching values and time condition
    for i, sensor_risk in enumerate(sensor_data[key][evaluate]):
        for j, target_risk in enumerate(target_system_data[value]):
            print(f'SENSOR RISK of {key}: {sensor_risk} TARGET RISK: {target_risk}')
            if sensor_risk == target_risk:
                # Parse time strings into floats
                sensor_time = float(sensor_data[key]['%time'][i]) / 1e3
                target_time = float(target_system_data['%time'][j]) / 1e3

                # Round and compare times
                #rounded_sensor_time = round(sensor_time, -5) / 1e6
                #rounded_target_time = round(target_time, -5) / 1e6
                time_diff = sensor_time - target_time
                if time_diff < time_threshold:
                    return False
                print(f'TIME DIFFERENCE in {key}: {time_diff} µs')
                
    return True