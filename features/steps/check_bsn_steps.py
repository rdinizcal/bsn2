from behave import given, when, then
import subprocess
import concurrent.futures
from utils.parsers import capture_csv_data,format_entity
def count_matching_elements(list1, list2):
    matching_elements = set(list1) & set(list2)
    # Return the count of matching elements
    return len(matching_elements)

@given('the {topic_name} topic is online')
def step_given_topic_is_online(context, topic_name):
    result = subprocess.run(['ros2', 'topic', 'list'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    topic_list = result.stdout.decode('utf-8').splitlines()
    assert format_entity(topic_name) in topic_list, f"{topic_name} is not online"

@when('I listen to topics')
def step_when_listen_to_topics(context):
    context.topic_data = {}

    # Use ThreadPoolExecutor to capture data from each topic in parallel
    with concurrent.futures.ThreadPoolExecutor() as executor:
        futures = {}
        for row in context.table:
            topic = format_entity(row['Topic Name'])
            line_limit = 10  # Optional line limit per topic

            # Submit the capture task to the executor
            futures[executor.submit(capture_csv_data, topic,line_limit)] = topic

        # Retrieve the captured data once all tasks are completed
        for future in concurrent.futures.as_completed(futures):
            topic = futures[future]
            try:
                context.topic_data[topic] = future.result()
            except Exception as e:
                print(f"Error while capturing data from {topic}: {e}")

@then('sensors will process the risks')
def step_then_check_high_risk(context):
    assert any(context.topic_data.values()), "No risk data found in sensor topics."

    for topic, data in context.topic_data.items():
        if topic == '/TargetSystemData':
            continue 
        assert 'risk' in data and data['risk'], f"No risk data detected in topic {topic}."

@then("{topic_name} will receive the risks from sensors and detect patient's status")
def step_then_check_target_system_receives_risk(context,topic_name):
    #print(f'TargetSystemData is receiving the risk data from sensors: {context.target_system_data}')
    risk_key_mapping = {
    '/thermometer_data': 'trm_risk',
    '/ecg_data': 'ecg_risk',

    }
    topic_name = format_entity(topic_name)
    print("TARGET SYSTEM DATA: ", context.topic_data[topic_name])
    target_data= context.topic_data[topic_name]
    sensor_data = {key: value for key, value in context.topic_data.items() if key != '/TargetSystemData'}
    print(f'SENSOR DATA: {sensor_data}')
    for key, value in risk_key_mapping.items():
    
        print("Target:", key, "Sensor:", value)
        print(f"Target risks: {sensor_data[key]['risk']} Sensor risks: {target_data[value]}")
        elements = count_matching_elements(sensor_data[key]['risk'], target_data[value])
        assert elements > 0, f"Topics {key} and {value} do not have matching risk data."
        assert target_data['patient_status'], f'patient_status is not being provided'
        assert len(target_data['patient_status']) >= elements, f'patient status is not being updated'
        assert all((x.replace('.', '', 1).isdigit() and 0 <= float(x) <= 100) 
                for x in target_data['patient_status']), f'patient status is not processing valid risk.'
@then('sensors will process the data')
def step_check_if_sensors_process_data(context):
    assert any(context.topic_data.values()), "No risk data found in sensor topics."

    for topic, data in context.topic_data.items():
        if topic == '/TargetSystemData':
            continue  # Skip the assertion for the TargetSystemData topic
        assert 'data' in data and data['data'], f"No data detected in topic {topic}."
        
@then('{topic_name} will receive the data from sensors')
def step_check_if_TargetSystem_process_data(context,topic_name):
    data_key_mapping = {
    '/thermometer_data': 'trm_data',
    '/ecg_data': 'ecg_data',
    
    }
    topic_name = format_entity(topic_name)
    print(f'TARGET system data: {context.topic_data[topic_name]}')
    target_data= context.topic_data[topic_name]
    sensor_data = {key: value for key, value in context.topic_data.items() if key != '/TargetSystemData'}
   
    print(f'SENSOR DATA: {sensor_data}')
    for key, value in data_key_mapping.items():
    
        print("Target:", key, "Sensor:", value)
        print(f"sensor data: {sensor_data[key]['data']} target data: {target_data[value]}")
        elements = count_matching_elements(sensor_data[key]['data'], target_data[value])
        assert elements > 0, f"Topics {key} and {value} do not have matching risk data."