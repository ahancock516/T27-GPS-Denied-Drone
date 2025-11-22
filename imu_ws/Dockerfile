# Use the official ROS Noetic image for ARM64 architecture
FROM arm64v8/ros:noetic-ros-base

# Install system dependencies
RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    python3-pip \
    i2c-tools \
    python3-dev \
    python3-setuptools \
    && rm -rf /var/lib/apt/lists/*

# Install Python libraries - no need for adafruit-blinka or lgpio
RUN pip3 install --no-cache-dir \
    sparkfun-qwiic-icm20948 \
    sparkfun-qwiic-i2c

# Create a catkin workspace
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src

# Clone the ros_icm20948 source code
RUN git clone https://github.com/ahancock516/ros_icm20948.git

# Create the modified node script
RUN cat > /root/catkin_ws/src/ros_icm20948/src/icm20948_node.py << 'EOF'
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import qwiic_icm20948
import sys

class ICM20948Node:
    def __init__(self):
        rospy.init_node('icm20948_node')
        
        # Publisher for IMU data
        self.imu_pub = rospy.Publisher('imu/data', Imu, queue_size=10)
        
        # Initialize the ICM20948 sensor using qwiic
        try:
            # Use qwiic library directly without board
            self.sensor = qwiic_icm20948.QwiicIcm20948(address=0x68)
            
            if not self.sensor.connected:
                rospy.logerr("ICM20948 sensor not detected. Check your I2C connection.")
                sys.exit(1)
                
            self.sensor.begin()
            rospy.loginfo("ICM20948 sensor initialized successfully")
            
        except Exception as e:
            rospy.logerr(f"Failed to initialize ICM20948: {e}")
            sys.exit(1)
        
        # Set up parameters
        self.frame_id = rospy.get_param('~frame_id', 'imu_link')
        self.rate = rospy.get_param('~rate', 100)  # Hz
        
        # Create timer for periodic publishing
        self.timer = rospy.Timer(rospy.Duration(1.0/self.rate), self.publish_imu_data)
        
    def publish_imu_data(self, event):
        try:
            # Check if data is ready
            if self.sensor.dataReady():
                # Read the sensor data
                self.sensor.getAgmt()
                
                # Create IMU message
                imu_msg = Imu()
                imu_msg.header.stamp = rospy.Time.now()
                imu_msg.header.frame_id = self.frame_id
                
                # Fill in accelerometer data (convert from mg to m/s^2)
                imu_msg.linear_acceleration.x = self.sensor.axRaw * 9.80665 / 1000.0
                imu_msg.linear_acceleration.y = self.sensor.ayRaw * 9.80665 / 1000.0
                imu_msg.linear_acceleration.z = self.sensor.azRaw * 9.80665 / 1000.0
                
                # Fill in gyroscope data (convert from DPS to rad/s)
                imu_msg.angular_velocity.x = self.sensor.gxRaw * 0.0174533
                imu_msg.angular_velocity.y = self.sensor.gyRaw * 0.0174533
                imu_msg.angular_velocity.z = self.sensor.gzRaw * 0.0174533
                
                # Orientation quaternion requires sensor fusion - leaving unset
                imu_msg.orientation_covariance[0] = -1
                
                # Set covariances
                for i in [0, 4, 8]:
                    imu_msg.angular_velocity_covariance[i] = 0.001
                    imu_msg.linear_acceleration_covariance[i] = 0.01
                
                # Publish the message
                self.imu_pub.publish(imu_msg)
                
        except Exception as e:
            rospy.logwarn(f"Error reading IMU data: {e}")
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ICM20948Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")
        sys.exit(1)
EOF

# Make the script executable
RUN chmod +x /root/catkin_ws/src/ros_icm20948/src/icm20948_node.py

# Build the workspace
WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; \
     catkin_make"

# Add the workspace's setup file to the .bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# RUN cat <<EOF > /root/catkin_ws/src/ros_icm20948/launches/icm20948.launch
# <launch>
#     <!-- ICM20948 IMU Node -->
#     <node name="icm20948_node" pkg="ros_icm20948" type="icm20948_node.py" output="screen">
#         <!-- Frame ID for the IMU messages -->
#         <param name="frame_id" value="imu_link"/>
        
#         <!-- Publishing rate in Hz -->
#         <param name="rate" value="100"/>
        
#         <!-- I2C address (0x69 is default, 0x68 if AD0 is pulled low) -->
#         <!-- Uncomment and modify if your sensor uses a different address -->
#         <param name="i2c_address" value="0x68"/>
#     </node>
# </launch>
# EOF

# Set the default command to launch the IMU node
CMD ["/bin/bash", "-c", "source /root/catkin_ws/devel/setup.bash && roslaunch ros_icm20948 icm20948.launch"]