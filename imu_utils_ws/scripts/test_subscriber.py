#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
import time

def imu_callback(msg):
    # This function is called every time a message is received
    print("SUCCESS: Received an IMU message with timestamp: %s" % msg.header.stamp)
    # We can stop the node after receiving one message to prove it works
    rospy.signal_shutdown("Received a message, test is complete.")

if __name__ == '__main__':
    try:
        # Initialize a new ROS node
        rospy.init_node('simple_imu_subscriber', anonymous=True)
        
        # Subscribe to the IMU topic
        rospy.Subscriber('/mavros/imu/data', Imu, imu_callback)
        
        print("Simple subscriber started. Waiting for IMU data on /mavros/imu/data...")
        
        # Keep the node running until it's shut down
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
    print("Test script finished.")