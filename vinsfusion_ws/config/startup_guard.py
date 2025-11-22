#!/usr/bin/env python3
import rospy
import subprocess
import signal
import time
import os
import sys
import gpiod
from mavros_msgs.msg import RCOut, StatusText

# --- CONFIGURATION ---
TRIGGER_CHANNEL_INDEX = 7  # Channel 8 (Index 7)
TRIGGER_THRESHOLD = 1500   # PWM > 1500 is ON
GPIO_PIN = 24              # BCM Pin 24
GPIO_CHIP = "/dev/gpiochip0"  # Pi 5 uses gpiochip4, Pi 4 uses gpiochip0
VINS_CMD = "roslaunch /root/vins_launch/mono_inertial_live.launch"
# ---------------------

class SystemBuzzer:
    def __init__(self, pin, chip_path=GPIO_CHIP):
        self.pin = pin
        self.chip = None
        self.line = None
        try:
            # gpiod v1.x API (Ubuntu Focal)
            self.chip = gpiod.chip(chip_path)
            self.line = self.chip.get_line(pin)
            
            # Request line as output, default LOW
            config = gpiod.line_request()
            config.consumer = "vins_buzzer"
            config.request_type = gpiod.line_request.DIRECTION_OUTPUT
            self.line.request(config, 0)  # 0 = initial value LOW
            
            rospy.loginfo(f"GPIO initialized on {chip_path}, pin {pin}")
        except Exception as e:
            rospy.logerr(f"GPIO Setup Failed: {e}")
            rospy.logerr("Make sure container has access to /dev/gpiochip*")

    def boot_sequence(self):
        """3 Short Beeps - System Online"""
        for _ in range(3):
            self._beep(0.1, 0.1)

    def success_sequence(self):
        """2 Medium Beeps - VINS Stable"""
        for _ in range(2):
            self._beep(0.4, 0.2)

    def error_sequence(self):
        """1 Long Beep - Crash detected"""
        self._beep(1.5, 0.1)

    def input_ack(self):
        """1 blip - Button press accepted"""
        self._beep(0.05, 0.0)

    def _beep(self, duration, gap):
        if self.line is None:
            return
        try:
            self.line.set_value(1)  # HIGH
            time.sleep(duration)
            self.line.set_value(0)  # LOW
            time.sleep(gap)
        except Exception as e:
            rospy.logwarn(f"Beep failed: {e}")

    def cleanup(self):
        if self.line:
            try:
                self.line.set_value(0)
                self.line.release()
            except:
                pass

class VinsGuard:
    def __init__(self):
        self.process = None
        self.last_pwm = 0
        self.is_running = False
        self.start_timestamp = 0
        self.is_validated = False

        rospy.init_node('vins_launcher_guard', anonymous=True)
        
        # 1. Setup Hardware
        self.buzzer = SystemBuzzer(GPIO_PIN)
        
        # 2. Setup ROS Publishers (For QGC HUD Feedback)
        self.status_pub = rospy.Publisher('/mavros/statustext/send', StatusText, queue_size=10)
        self.sub = rospy.Subscriber("/mavros/rc/out", RCOut, self.rc_callback)

        rospy.loginfo("VINS Guard Active. Waiting for Switch on Ch %s...", TRIGGER_CHANNEL_INDEX + 1)
        self.send_feedback("SYSTEM READY", severity=6)
        
        # 3. Play Boot Sound
        self.buzzer.boot_sequence()

        # Ensure GPIO cleanup on exit
        rospy.on_shutdown(self.shutdown_hook)

    def rc_callback(self, msg):
        if len(msg.channels) > TRIGGER_CHANNEL_INDEX:
            self.last_pwm = msg.channels[TRIGGER_CHANNEL_INDEX]

    def start_vins(self):
        if not self.is_running:
            rospy.loginfo("Switch HIGH: Launching VINS...")
            self.send_feedback("INITIALIZING VINS...", severity=6)
            self.buzzer.input_ack()
            
            self.process = subprocess.Popen(
                VINS_CMD, shell=True, executable='/bin/bash', preexec_fn=os.setsid
            )
            
            self.is_running = True
            self.start_timestamp = time.time()
            self.is_validated = False

    def stop_vins(self):
        if self.is_running and self.process:
            rospy.loginfo("Switch LOW: Stopping VINS...")
            self.send_feedback("STOPPING VINS", severity=6)
            
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
                self.process.wait(timeout=5) 
            except Exception as e:
                rospy.logwarn(f"Force killing VINS: {e}")
                os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)

            self.process = None
            self.is_running = False
            self.buzzer.input_ack()

    def monitor_loop(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            switch_is_on = self.last_pwm > TRIGGER_THRESHOLD

            if switch_is_on:
                if not self.is_running:
                    self.start_vins()
                else:
                    if not self.is_validated and (time.time() - self.start_timestamp > 5):
                        if self.process.poll() is None:
                            self.is_validated = True
                            self.send_feedback("VINS STABLE", severity=2)
                            self.buzzer.success_sequence()
                        else:
                            self.is_validated = True
                            self.send_feedback("STARTUP FAILURE", severity=0)
                            self.buzzer.error_sequence()

                    if self.process.poll() is not None:
                        rospy.logwarn("VINS crashed! Restarting...")
                        if self.is_validated:
                            self.buzzer.error_sequence()
                        self.is_running = False

            else:
                if self.is_running:
                    self.stop_vins()

            rate.sleep()

    def send_feedback(self, text, severity=6):
        msg = StatusText()
        msg.severity = severity
        msg.text = text
        self.status_pub.publish(msg)

    def shutdown_hook(self):
        if self.is_running:
            self.stop_vins()
        self.buzzer.cleanup()

if __name__ == '__main__':
    try:
        guard = VinsGuard()
        guard.monitor_loop()
    except rospy.ROSInterruptException:
        pass