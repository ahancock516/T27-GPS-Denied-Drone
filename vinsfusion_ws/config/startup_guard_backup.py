#!/usr/bin/env python3
import rospy
import subprocess
import signal
import time
from mavros_msgs.msg import RCOut

# --- CONFIGURATION ---
TRIGGER_CHANNEL_INDEX = 7  # Channel 8 (Index 7)
TRIGGER_THRESHOLD = 1500   # PWM > 1500 is ON
VINS_CMD = "roslaunch /root/vins_launch/mono_inertial_live.launch"
# ---------------------

class VinsGuard:
    def __init__(self):
        self.process = None
        self.last_pwm = 0
        self.is_running = False
        
        rospy.init_node('vins_launcher_guard', anonymous=True)
        rospy.loginfo("VINS Guard Active. Waiting for Switch on Ch %s...", TRIGGER_CHANNEL_INDEX + 1)
        
        self.sub = rospy.Subscriber("/mavros/rc/out", RCOut, self.rc_callback)

    def rc_callback(self, msg):
        # Update the latest PWM value safely
        if len(msg.channels) > TRIGGER_CHANNEL_INDEX:
            self.last_pwm = msg.channels[TRIGGER_CHANNEL_INDEX]

    def start_vins(self):
        if not self.is_running:
            rospy.loginfo("Switch HIGH: Launching VINS...")
            # Popen is non-blocking, allowing us to keep checking the switch
            self.process = subprocess.Popen(VINS_CMD, shell=True, executable='/bin/bash', preexec_fn=os.setsid)
            self.is_running = True

    def stop_vins(self):
        if self.is_running and self.process:
            rospy.loginfo("Switch LOW: Stopping VINS...")
            # Send SIGINT (Ctrl+C) to the process group to kill ROS nodes cleanly
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
                self.process.wait(timeout=5) # Wait for clean exit
            except Exception as e:
                rospy.logwarn(f"Force killing VINS: {e}")
                # If it doesn't close gracefully, force kill
                os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
            
            self.process = None
            self.is_running = False

    def monitor_loop(self):
        rate = rospy.Rate(10) # Check 10 times a second
        
        while not rospy.is_shutdown():
            # 1. Check if Switch is ON or OFF
            switch_is_on = self.last_pwm > TRIGGER_THRESHOLD

            # 2. Handle State Changes
            if switch_is_on:
                if not self.is_running:
                    self.start_vins()
                else:
                    # logic to auto-restart if it crashed while switch is still ON
                    if self.process.poll() is not None:
                        rospy.logwarn("VINS crashed! Restarting because switch is still ON...")
                        self.is_running = False # Reset state so next loop restarts it
            
            else: # Switch is OFF
                if self.is_running:
                    self.stop_vins()

            rate.sleep()

import os # Needed for process group management

if __name__ == '__main__':
    try:
        guard = VinsGuard()
        guard.monitor_loop()
    except rospy.ROSInterruptException:
        pass
