import subprocess
import time
# import signal
# import os
# 
# 
# def before_all(context):
    # """Global setup - runs once before all scenarios"""
    # context.launched_systems = {}
    # 
# 
# def before_scenario(context, scenario):
    # """Setup environment based on scenario tags"""
    # 
    
    # if "reduced_system" in scenario.tags:
        # launch_reduced_system(context)
    # elif "High_frequency_sensor_system" in scenario.tags:
        # launch_high_frequency_system(context)
    # elif "persistance_system" in scenario.tags:
        # launch_persistence_system(context)
    # else:
        
        # launch_default_system(context)
# 
# 
# def launch_reduced_system(context):
    # """Launch minimal BSN system for reduced scenarios"""
    # print("Launching reduced BSN system...")
    # 
    # context.current_launch = subprocess.Popen(
        # ['ros2', 'launch', 'central_hub', 'emergency_detection_simplified_launch.py'],
        # stdout=subprocess.DEVNULL,
        # stderr=subprocess.STDOUT,
        # preexec_fn=os.setsid
    # )
    # context.system_type = "reduced"
    # time.sleep(120)  # Shorter wait for reduced system
# 
# 
# def launch_high_frequency_system(context):
    # """Launch BSN system optimized for high frequency testing"""
    # print("Launching high frequency BSN system...")
    # 
    # context.current_launch = subprocess.Popen(
        # ['ros2', 'launch', 'central_hub', 'emergency_detection_simplified_launch.py'],
        # stdout=subprocess.DEVNULL,
        # stderr=subprocess.STDOUT,
        # preexec_fn=os.setsid
    # )
    # context.system_type = "high_frequency"
    # time.sleep(40)
# 
# 
# def launch_persistence_system(context):
    # """Launch BSN system with persistence components"""
    # print("Launching persistence BSN system...")
    # 
    # context.current_launch = subprocess.Popen(
        # ['ros2', 'launch', 'central_hub', 'emergency_detection_simplified_launch.py'],
        # stdout=subprocess.DEVNULL,
        # stderr=subprocess.STDOUT,
        # preexec_fn=os.setsid
    # )
    # context.system_type = "persistence"
    # time.sleep(50)
# 
# 
# def launch_default_system(context):
    # """Launch default BSN system"""
    # print("Launching default BSN system...")
    # 
    # context.current_launch = subprocess.Popen(
        # ['ros2', 'launch', 'central_hub', 'emergency_detection_launch.py'],
        # stdout=subprocess.DEVNULL,
        # stderr=subprocess.STDOUT,
        # preexec_fn=os.setsid
    # )
    # context.system_type = "default"
    # time.sleep(60)
# 
# 
# def after_scenario(context, scenario):
    # """Cleanup after each scenario"""
    # if hasattr(context, 'current_launch') and context.current_launch.poll() is None:
        # print(f"Cleaning up {context.system_type} system...")
        # 
        # try:
            
            # os.killpg(os.getpgid(context.current_launch.pid), signal.SIGINT)
            # 
            # try:
                # context.current_launch.wait(timeout=10)
                # print(f"{context.system_type} system terminated gracefully")
            # except subprocess.TimeoutExpired:
                # print(f"Force killing {context.system_type} system...")
                # os.killpg(os.getpgid(context.current_launch.pid), signal.SIGKILL)
                # context.current_launch.wait()
                # 
        # except (ProcessLookupError, OSError):
            # print("Process group already terminated")
        # except Exception as e:
            # print(f"Error terminating {context.system_type} system: {e}")
    # 
    
    # time.sleep(2)
# 
# 
def after_all(context):
    """Final cleanup - kill all possible ROS2 nodes and processes"""
    import subprocess
    import time
    import signal
    import os
    
    print("Starting comprehensive cleanup...")
    
    # Step 1: Try graceful ROS2 shutdown first
    try:
        print("Attempting graceful ROS2 shutdown...")
        subprocess.run(['ros2', 'daemon', 'stop'], 
                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5)
    except:
        pass
    
    # Step 2: Kill specific node processes by name
    node_patterns = [
        'logger',
        'patient_node', 
        'reli_engine',
        'thermometer_node',
        'central_hub_node',
        'node_monitor',
        'param_adapter',
        'enactor',
        'data_access',
        'emergency_detection',
        'sensor',
        'patient'
    ]
    
    for pattern in node_patterns:
        try:
            print(f"Killing processes matching: {pattern}")
            subprocess.run(['pkill', '-f', pattern], 
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except:
            pass
    
    # Step 3: Kill ROS2 related processes
    ros_patterns = [
        'ros2',
        '_node',
        'python.*ros2',
        'launch',
        'central_hub',
        'system_monitor',
        'adaptation',
        'bsn'
    ]
    
    for pattern in ros_patterns:
        try:
            print(f"Killing ROS2 processes: {pattern}")
            subprocess.run(['pkill', '-f', pattern], 
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except:
            pass
    
    # Step 4: Force kill by process group if needed
    try:
        # Kill any remaining process groups
        subprocess.run(['pkill', '-SIGKILL', '-f', 'emergency_detection'], 
                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.run(['pkill', '-SIGKILL', '-f', '_node'], 
                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except:
        pass
    
    # Step 5: Clean up any leftover Python processes running ROS nodes
    try:
        print("Cleaning up Python ROS processes...")
        result = subprocess.run(['pgrep', '-f', 'python.*node'], 
                              capture_output=True, text=True)
        if result.stdout:
            pids = result.stdout.strip().split('\n')
            for pid in pids:
                if pid:
                    try:
                        os.kill(int(pid), signal.SIGTERM)
                    except:
                        pass
    except:
        pass
    
    # Step 6: Wait and force kill if still running
    time.sleep(3)
    
    for pattern in node_patterns:
        try:
            subprocess.run(['pkill', '-SIGKILL', '-f', pattern], 
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except:
            pass
    
    # Step 7: Clean up any launch processes
    try:
        subprocess.run(['pkill', '-f', 'launch.*emergency_detection'], 
                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.run(['pkill', '-SIGKILL', '-f', 'launch'], 
                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except:
        pass
    
    # Step 8: Final verification and cleanup
    time.sleep(2)
    try:
        result = subprocess.run(['ros2', 'node', 'list'], 
                              capture_output=True, text=True, timeout=5)
        if result.stdout.strip():
            print(f"Warning: Some nodes may still be running: {result.stdout}")
            # Force kill any remaining nodes
            subprocess.run(['pkill', '-SIGKILL', '-f', 'ros2'], 
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except:
        pass
    
    print("Comprehensive cleanup completed")

