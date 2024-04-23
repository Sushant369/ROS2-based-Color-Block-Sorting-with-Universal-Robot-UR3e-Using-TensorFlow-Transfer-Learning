import subprocess

# Get the PID of the main ROS2 process
pid = subprocess.check_output(['pgrep', '-f', 'ros2']).decode().strip()

# Kill the process
subprocess.run(['kill', '-SIGINT', pid])