#!/usr/bin/env python3
import rospy
import time
from mavros_msgs.msg import RCOut

class FileTriggerTest:
    def __init__(self):
        rospy.init_node('file_trigger_test', anonymous=True)

        # --- CONFIGURATION ---
        self.CHANNEL_INDEX = 7       # Index 7 = Servo 8
        self.THRESHOLD = 1500
        self.FILE_PATH = "/root/catkin_ws/src/VINS-Fusion/config/switch_output.txt" # Created in the folder where you run the script
        # ---------------------

        self.triggered = False

        rospy.Subscriber("/mavros/rc/out", RCOut, self.callback)
        
        rospy.loginfo("File Test Node Running.")
        rospy.loginfo(f"Flip the switch to create: {self.FILE_PATH}")
        rospy.spin()

    def callback(self, data):
        # Safety check
        if len(data.channels) <= self.CHANNEL_INDEX:
            return

        current_pwm = data.channels[self.CHANNEL_INDEX]

        # Check for Switch ON (Rising Edge)
        if current_pwm > self.THRESHOLD and not self.triggered:
            self.triggered = True
            self.create_hello_world_file()
            
        # Check for Switch OFF (Falling Edge)
        elif current_pwm < self.THRESHOLD and self.triggered:
            self.triggered = False
            rospy.loginfo("Switch reset. Ready for next trigger.")

    def create_hello_world_file(self):
        # 1. Print to Command Line
        rospy.loginfo("SWITCH DETECTED! Writing to file...")

        try:
            # 2. Create/Write the file
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            content = f"Hello World! \nSwitch triggered at: {timestamp}\n"
            
            with open(self.FILE_PATH, "w") as f:
                f.write(content)
            
            rospy.loginfo(f"Success! Wrote 'Hello World' to {self.FILE_PATH}")
            
        except Exception as e:
            rospy.logerr(f"Failed to write file: {e}")

if __name__ == '__main__':
    try:
        FileTriggerTest()
    except rospy.ROSInterruptException:
        pass