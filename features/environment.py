import subprocess
import time
import sys
import os

# Add the utils directory to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'steps'))

# Now import normally (not relative)
from utils.parsers import set_node_lifecycle_state

# # import signal
# # import os
# #
# # def before_all(context):
#     # """Global setup - runs once before all scenarios"""
#     # context.launched_systems = {}
# # 
# # 
# def before_scenario(context, scenario):
#     """Setup environment based on scenario tags"""
#     
#     if "reduced_system" in scenario.tags:
#         launch_reduced_system(context)
#     elif "High_frequency_sensor_system" in scenario.tags:
#         launch_high_frequency_system(context)
#     elif "persistance_system" in scenario.tags:
#         launch_persistence_system(context)
#     else:
#         launch_default_system(context)
# 
# 
# def launch_reduced_system(context):
#     """Launch minimal BSN system for reduced scenarios"""
#     print("Launching reduced BSN system...")
#     
#     context.current_launch = subprocess.Popen(
#         ['ros2', 'launch', 'central_hub', 'emergency_detection_simplified_launch.py'],
#         stdout=subprocess.DEVNULL,
#         stderr=subprocess.STDOUT,
#         preexec_fn=os.setsid
#     )
#     context.system_type = "reduced"
#     time.sleep(120)  # Shorter wait for reduced system
# 
# 
# def launch_high_frequency_system(context):
#     """Launch BSN system optimized for high frequency testing"""
#     print("Launching high frequency BSN system...")
#     
#     context.current_launch = subprocess.Popen(
#         ['ros2', 'launch', 'central_hub', 'emergency_detection_simplified_launch.py'],
#         stdout=subprocess.DEVNULL,
#         stderr=subprocess.STDOUT,
#         preexec_fn=os.setsid
#     )
#     context.system_type = "high_frequency"
#     time.sleep(40)
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
# def after_scenario(context, scenario):
#     """Cleanup after each scenario"""
#     if hasattr(context, 'current_launch') and context.current_launch.poll() is None:
#         print(f"Cleaning up {context.system_type} system...")
#         
#         try:
#             os.killpg(os.getpgid(context.current_launch.pid), signal.SIGINT)
#             
#             try:
#                 context.current_launch.wait(timeout=10)
#                 print(f"{context.system_type} system terminated gracefully")
#             except subprocess.TimeoutExpired:
#                 print(f"Force killing {context.system_type} system...")
#                 os.killpg(os.getpgid(context.current_launch.pid), signal.SIGKILL)
#                 context.current_launch.wait()
#                 
#         except (ProcessLookupError, OSError):
#             print("Process group already terminated")
#         except Exception as e:
#             print(f"Error terminating {context.system_type} system: {e}")
#     
#     time.sleep(2)

def after_all(context):
    """Final cleanup - kill launch processes and all nodes"""
    set_node_lifecycle_state('central_hub_node', 'activate')
    if hasattr(context, 'current_launch') and context.current_launch.poll() is None:
        print(f"Cleaning up {context.system_type} system...")
        os.killpg(os.getpgid(context.current_launch.pid), signal.SIGINT)
        context.current_launch.wait()