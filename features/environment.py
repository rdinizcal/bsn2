import subprocess
import time


def before_all(context):

    # context.bsn_launch = subprocess.Popen(
    #    ['ros2', 'launch', 'central_hub', 'emergency_detection_launch.py'], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT
    # )
    # time.sleep(30)
    pass


def after_all(context):
    # context.roscore.terminate()
    context.bsn_launch.terminate()
    context.bsn_launch.wait()
