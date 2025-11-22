import RPi.GPIO as GPIO
import subprocess
import time
import signal

# -- Configuration --
BUTTON_PIN = 17  # The GPIO pin your button is connected to (using BCM numbering)
DOCKER_CONTAINER_NAME = "your_vins_container_name"  # The name of your VINS fusion Docker container
ROS_LAUNCH_COMMAND = "roslaunch /root/vins_launch/mono_inertial_live.launch"

# -- State --
ros_process = None

def execute_in_docker(command):
    """Executes a command inside the specified Docker container."""
    # The command is executed within a bash shell in the container
    docker_command = f"docker exec {DOCKER_CONTAINER_NAME} /bin/bash -c '{command}'"
    # Using Popen to run the command in a new process
    return subprocess.Popen(docker_command, shell=True)

def button_callback(channel):
    """This function is called when the button is pressed."""
    global ros_process

    if ros_process is None:
        print("Button pressed. Launching VINS-Fusion...")
        ros_process = execute_in_docker(ROS_LAUNCH_COMMAND)
    else:
        print("Button pressed again. Shutting down VINS-Fusion...")
        # Sending SIGINT (Ctrl+C) to gracefully shut down roslaunch
        ros_process.send_signal(signal.SIGINT)
        ros_process.wait() # Wait for the process to terminate
        ros_process = None
        print("VINS-Fusion has been shut down.")

# -- GPIO Setup --
GPIO.setmode(GPIO.BCM)
# Set up the button pin as an input with a pull-up resistor
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# -- Event Detection --
# Add an event detection for a falling edge (button press)
GPIO.add_event_detect(BUTTON_PIN, GPIO.FALLING, callback=button_callback, bouncetime=500)

print("ROS Launch Control script started. Press the button to launch/terminate.")
print("Press Ctrl+C to exit the script.")

try:
    # Keep the script running to listen for button presses
    while True:
        time.sleep(1)

except KeyboardInterrupt:
    print("\nExiting script.")

finally:
    # Clean up GPIO settings on exit
    GPIO.cleanup()