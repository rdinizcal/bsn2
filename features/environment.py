import subprocess
import time
import signal
import os


def before_all(context):
    # Create a new process group for the launch process
    context.bsn_launch = subprocess.Popen(
        ['ros2', 'launch', 'central_hub', 'emergency_detection_launch.py'], 
        stdout=subprocess.DEVNULL, 
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid  # Create new process group
    )
    time.sleep(30)


def after_all(context):
    if hasattr(context, 'bsn_launch') and context.bsn_launch.poll() is None:
        print("Sending SIGINT (Ctrl+C) to BSN launch and all child processes...")
        
        try:
            # Send SIGINT to entire process group (including all child nodes)
            os.killpg(os.getpgid(context.bsn_launch.pid), signal.SIGINT)
            print("SIGINT sent to process group")
            
            # Wait for graceful shutdown
            try:
                context.bsn_launch.wait(timeout=15)
                print("BSN launch terminated gracefully")
            except subprocess.TimeoutExpired:
                print("BSN launch didn't terminate gracefully, sending SIGTERM...")
                
                # Send SIGTERM to process group
                os.killpg(os.getpgid(context.bsn_launch.pid), signal.SIGTERM)
                
                try:
                    context.bsn_launch.wait(timeout=10)
                    print("BSN launch terminated with SIGTERM")
                except subprocess.TimeoutExpired:
                    print("BSN launch still running, forcing termination with SIGKILL...")
                    
                    # Force kill entire process group
                    os.killpg(os.getpgid(context.bsn_launch.pid), signal.SIGKILL)
                    context.bsn_launch.wait()
                    print("BSN launch force terminated")
                    
        except ProcessLookupError:
            print("Process group already terminated")
        except Exception as e:
            print(f"Error terminating BSN launch: {e}")
            # Fallback: kill individual process
            try:
                context.bsn_launch.terminate()
                context.bsn_launch.wait(timeout=5)
            except subprocess.TimeoutExpired:
                context.bsn_launch.kill()
                context.bsn_launch.wait()

