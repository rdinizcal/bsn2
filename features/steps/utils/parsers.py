import subprocess
import importlib
import concurrent.futures

def format_entity(raw_string):
    # Check if any words in the string start with an uppercase letter
    words = raw_string.split()
    if any(word[0].isupper() for word in words):
        # If there are uppercase letters, format in camel case
        formatted_string = ''.join(word.capitalize() for word in words)
    else:
        # Otherwise, format in snake case
        formatted_string = '_'.join(word.lower() for word in words)
    
    # Add a forward slash at the beginning
    return f'/{formatted_string}'

def get_message_attributes(topic_name):
    """Dynamically get the attributes of a ROS2 message type."""
    # Convert 'format_data/msg/Data' to 'format_data.msg.Data'
    result = subprocess.run(['ros2', 'topic', 'type', topic_name], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
    if result.returncode != 0:
        raise Exception(f"Error getting node info: {result.stderr.decode('utf-8')}")
    
    message_type = result.stdout.decode('utf-8').strip()
    
    module_name = message_type.replace('/', '.')
    class_name = module_name.split('.')[-1]  # Extract the last part as the class name
    module_name = '.'.join(module_name.split('.')[:-1])  # Get the module part

    # Import the module and get the message class
    module = importlib.import_module(module_name)
    message_class = getattr(module, class_name)

    # Retrieve attributes
    attributes = []
    if hasattr(message_class, '__slots__'):
        attributes = list(message_class.__slots__)
    elif hasattr(message_class, '__dataclass_fields__'):
        attributes = list(message_class.__dataclass_fields__.keys())

    # Handle nested fields like 'header' and remove invalid fields
    expanded_attributes = []
    for attr in attributes:
        if attr == 'header':
            expanded_attributes.extend(['_header_stamp_sec', '_header_stamp_nanosec', '_header_frame_id'])
        elif attr != '_check_fields':  # Exclude invalid or unnecessary fields
            expanded_attributes.append(attr)

    # Ensure header-related fields are at the front of the list
    return ['_header_stamp_sec', '_header_stamp_nanosec', '_header_frame_id'] + [
        attr for attr in expanded_attributes if attr not in ['_header', '_check_fields']
    ]

def capture_csv_data(topic, line_limit=10):
    """
    Capture CSV data from a ROS2 topic and organize it into a dictionary with keys based on message keys.
    """
    # Step 1: Get the message attributes for dynamic keys
    message_keys = get_message_attributes(topic)
    output = {key[1:]: [] for key in message_keys}
    

    # Step 2: Start the subprocess to capture CSV data
    process = subprocess.Popen(
        ['ros2', 'topic', 'echo', '--csv', topic],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )

    try:
        # Read lines until reaching the line limit
        while True:
            line = process.stdout.readline()  # Read one line at a time
            if not line:  # Break if no more lines to read
                break

            values = line.strip().split(',')  # Split the CSV line into values
            
            if len(values) == len(output):  # Ensure the line has the correct number of values
                # Map values to their respective keys in the output dictionary
                for key, value in zip(output.keys(), values):
                    output[key].append(value)

            # Break if all lists have at least `line_limit` items
            if all(len(v) >= line_limit for v in output.values()):
                break
            
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        process.terminate()  # Ensure the process exits properly
        process.wait()  # Ensure the process exits properly

    return output

def capture_topic_data(context, topics, line_limit=10):
    """
    Captures data from multiple ROS 2 topics using a ThreadPoolExecutor.

    Args:
        context: Behave context object to store topic data.
        topics (list): A list of ROS 2 topic names to capture data from.
        line_limit (int): Optional line limit for the data capture.

    Returns:
        None: Updates context.topic_data with the captured data for each topic.
    """
    with concurrent.futures.ThreadPoolExecutor() as executor:
        futures = {}
        for topic in topics:
            futures[executor.submit(capture_csv_data, topic, line_limit)] = topic

        for future in concurrent.futures.as_completed(futures):
            topic = futures[future]
            try:
                context.topic_data[topic] = future.result()
            except Exception as e:
                print(f"Error while capturing data from {topic}: {e}")

def get_rosnode_info_ros2(lines):
    node_info = {
        "publishers": [],
        "subscribers": [],
        "services": {
            "servers": [],
            "clients": []
        },
        "actions": {
            "servers": [],
            "clients": []
        }
    }

    def parse_topic_lines(start_index, keyword):
        topics = []
        i = start_index
        while i < len(lines) and lines[i].strip().startswith(keyword):
            line = lines[i].strip().split(': ')
            if len(line) == 2:
                topic = line[0].strip()
                msg_type = line[1].strip()
                topics.append({"topic": topic, "type": msg_type})
            i += 1
        return topics, i

    i = 0
    while i < len(lines):
        line = lines[i].strip()

        if line.startswith('Subscribers:'):
            node_info["subscribers"], i = parse_topic_lines(i + 1, '/')

        elif line.startswith('Publishers:'):
            node_info["publishers"], i = parse_topic_lines(i + 1, '/')

        elif line.startswith('Service Servers:'):
            i += 1
            while i < len(lines) and lines[i].strip().startswith('/'):
                service = lines[i].strip().split(': ')[0]
                node_info["services"]["servers"].append(service)
                i += 1

        elif line.startswith('Service Clients:'):
            i += 1
            while i < len(lines) and lines[i].strip().startswith('/'):
                client = lines[i].strip().split(': ')[0]
                node_info["services"]["clients"].append(client)
                i += 1

        elif line.startswith('Action Servers:'):
            i += 1
            while i < len(lines) and lines[i].strip().startswith('/'):
                action_server = lines[i].strip().split(': ')[0]
                node_info["actions"]["servers"].append(action_server)
                i += 1

        elif line.startswith('Action Clients:'):
            i += 1
            while i < len(lines) and lines[i].strip().startswith('/'):
                action_client = lines[i].strip().split(': ')[0]
                node_info["actions"]["clients"].append(action_client)
                i += 1
        else:
            i += 1

    return node_info