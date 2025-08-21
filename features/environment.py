import signal
import subprocess
import time
import sys
import os

# Add the utils directory to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'steps'))

# Now import normally (not relative)
from utils.parsers import set_node_lifecycle_state

# import signal
# import os
#
# def before_all(context):
    # """Global setup - runs once before all scenarios"""
    # context.launched_systems = {}
# 
# 
def before_scenario(context, scenario):
    """Setup environment based on scenario tags"""
    
    if "reduced_system" in scenario.tags:
        launch_reduced_system(context)
    elif "High_frequency_sensor_system" in scenario.tags:
        launch_high_frequency_system(context)
    elif "persistance_system" in scenario.tags:
        launch_persistence_system(context)



def launch_reduced_system(context):
    """Launch minimal BSN system for reduced scenarios"""
    print("Launching reduced BSN system...")

    set_parameters("patient_node", [
        "temperature_State0",
        "temperature_State1",
        "temperature_State2",
        "temperature_State3",
        "temperature_State4"
    ], [
        '[0.3, 0.0, 0.0, 0.0, 0.7]',
        '[0.5, 0.0, 0.0, 0.0, 0.5]',
        '[0.0, 0.0, 0.1, 0.4, 0.5]',
        '[0.0, 0.0, 0.0, 0.6, 0.4]',
        '[0.0, 0.0, 0.0, 0.1, 0.9]'
    ])
    time.sleep(10)


def launch_high_frequency_system(context):
    """Launch BSN system optimized for high frequency testing"""
    print("Launching high frequency BSN system...")
    
    set_parameters("patient_node", [
    "temperature_State0",
    "temperature_State1",
    "temperature_State2",
    "temperature_State3",
    "temperature_State4"
    ], [
    '[0.0, 0.0, 1.0, 0.0, 0.0]',
    '[0.0, 0.0, 1.0, 0.0, 0.0]',
    '[0.0, 0.0, 1.0, 0.0, 0.0]',
    '[0.0, 0.0, 1.0, 0.0, 0.0]',
    '[0.0, 0.0, 1.0, 0.0, 0.0]'
    ])
    time.sleep(10)
# 
# 
# def launch_persistence_system(context):
#     """Launch BSN system with persistence components"""
#     print("Launching persistence BSN system...")
#     
#     context.current_launch = subprocess.Popen(
#         ['ros2', 'launch', 'central_hub', 'emergency_detection_simplified_launch.py'],
#         stdout=subprocess.DEVNULL,
#         stderr=subprocess.STDOUT,
#         preexec_fn=os.setsid
#     )
#     context.system_type = "persistence"
#     time.sleep(50)
# 
# 
# def launch_default_system(context):
#     """Launch default BSN system"""
#     print("Launching default BSN system...")
#     
#     context.current_launch = subprocess.Popen(
#         ['ros2', 'launch', 'central_hub', 'emergency_detection_launch.py'],
#         stdout=subprocess.DEVNULL,
#         stderr=subprocess.STDOUT,
#         preexec_fn=os.setsid
#     )
#     context.system_type = "default"
#     time.sleep(60)
# 
# 
def after_scenario(context, scenario):
    """Cleanup after each scenario"""
    if "High_frequency_sensor_system" in scenario.tags:
        set_parameters("patient_node", [
            "temperature_State0",
            "temperature_State1",
            "temperature_State2",
            "temperature_State3",
            "temperature_State4"
        ], [
            '[0.25, 0.51, 0.21, 0.03, 0.0]',
            '[0.05, 0.5, 0.43, 0.02, 0.0]',
            '[0.0, 0.04, 0.85, 0.11, 0.0]',
            '[0.0, 0.01, 0.32, 0.67, 0.0]',
            '[0.0, 0.0, 0.0, 0.0, 0.0]'
        ])
        time.sleep(5)


def set_parameters(node_name, params, values):
    """Set parameters for the patient node."""
    for param, value in zip(params, values):
        subprocess.run([
            "ros2", "param", "set", f"/{node_name}", param, value
        ], check=False)