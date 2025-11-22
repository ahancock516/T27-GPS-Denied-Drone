#!/bin/bash
set -e

# Source ROS environment
source /root/catkin_ws/devel/setup.bash

# Parameters
IMU_TOPIC="${IMU_TOPIC:-/mavros/imu/data_raw}"
DURATION="${DURATION:-10800}"
OUTPUT_DIR="/data"
CONFIG_FILE="${CONFIG_FILE:-/config/imu_config.yaml}"

echo "================================"
echo "Allan Variance Analysis"
echo "================================"
echo "IMU Topic: $IMU_TOPIC"
echo "Duration: $DURATION seconds ($(echo "scale=2; $DURATION/3600" | bc) hours)"
echo "Output Directory: $OUTPUT_DIR"
echo "Config File: $CONFIG_FILE"
echo "================================"

# Clean up any old bag files
echo "Cleaning up old data..."
rm -f $OUTPUT_DIR/*.bag $OUTPUT_DIR/*.csv $OUTPUT_DIR/*.png $OUTPUT_DIR/imu.yaml

# Wait for ROS master
echo "Waiting for ROS master..."
until rostopic list > /dev/null 2>&1; do
    sleep 1
done
echo "ROS master is ready!"

# Wait for IMU topic
echo "Waiting for IMU topic $IMU_TOPIC..."
until rostopic info $IMU_TOPIC > /dev/null 2>&1; do
    echo "  IMU topic not available yet, waiting..."
    sleep 2
done
echo "IMU topic is available!"

# Check IMU rate
echo ""
echo "Checking IMU rate..."
timeout 5s rostopic hz $IMU_TOPIC || echo "Could not determine rate"

# Step 1: Record IMU data
echo ""
echo "Step 1: Recording IMU data for $DURATION seconds..."
rosbag record -O $OUTPUT_DIR/imu_data.bag $IMU_TOPIC --duration=$DURATION

# Check if bag was recorded
echo ""
echo "Checking recorded bag info..."
rosbag info $OUTPUT_DIR/imu_data.bag

# Step 2: Cook the bag and remove original
echo ""
echo "Step 2: Cooking bag (reorganizing by timestamp)..."
python3 /root/catkin_ws/src/allan_variance_ros/scripts/cookbag.py \
    --input $OUTPUT_DIR/imu_data.bag \
    --output $OUTPUT_DIR/cooked_imu_data.bag

# Remove the original bag so allan_variance only processes the cooked one
rm -f $OUTPUT_DIR/imu_data.bag

# Check cooked bag
echo ""
echo "Checking cooked bag info..."
rosbag info $OUTPUT_DIR/cooked_imu_data.bag

# Step 3: Run Allan Variance computation
echo ""
echo "Step 3: Computing Allan Variance..."
rosrun allan_variance_ros allan_variance $OUTPUT_DIR $CONFIG_FILE

# Check if CSV was created
if [ ! -f "$OUTPUT_DIR/allan_variance.csv" ]; then
    echo "ERROR: allan_variance.csv was not created!"
    exit 1
fi

echo ""
echo "CSV file created successfully!"
LINE_COUNT=$(wc -l < $OUTPUT_DIR/allan_variance.csv)
echo "CSV has $LINE_COUNT lines of data"

if [ $LINE_COUNT -lt 10 ]; then
    echo "WARNING: Very little data in CSV file!"
    cat $OUTPUT_DIR/allan_variance.csv
fi

# Step 4: Run analysis and generate imu.yaml
echo ""
echo "Step 4: Generating analysis plots and imu.yaml..."
cd $OUTPUT_DIR
python3 /root/catkin_ws/src/allan_variance_ros/scripts/analysis.py \
    --data $OUTPUT_DIR/allan_variance.csv \
    --config $CONFIG_FILE

echo ""
echo "================================"
echo "Analysis Complete!"
echo "================================"
ls -lh $OUTPUT_DIR/
echo "================================"

if [ -f "$OUTPUT_DIR/imu.yaml" ]; then
    echo ""
    echo "IMU Parameters (imu.yaml):"
    cat $OUTPUT_DIR/imu.yaml
fi