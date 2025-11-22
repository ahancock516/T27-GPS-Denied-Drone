#!/bin/bash

# ==============================================================================
# extract_odometry.sh
#
# Description:
#   This script extracts the /vins_estimator/odometry topic from a single,
#   specified ROS bag file and saves it as a CSV file. The new CSV file
#   will have the same name as the bag file and be saved in the same directory.
#
#   The script cleans the header row of the CSV by removing the leading '%'
#   character that rostopic echo adds, making it compatible with most parsers.
#
# Usage:
#   1. Make the script executable:  chmod +x extract_odometry.sh
#   2. Run it, passing the path to a bag file as an argument:
#      ./extract_odometry.sh /path/to/your/rosbag.bag
#
# Example (from within your /root/rosbags directory):
#   ./extract_odometry.sh my_flight_data.bag
#
# ==============================================================================

# --- Configuration ---
# The ROS topic to extract.
TOPIC="/vins_estimator/odometry"

# --- Script Logic ---
# Check if an argument (the bag file) was provided.
if [ "$#" -ne 1 ]; then
    echo "Error: You must provide the path to a single ROS bag file."
    echo "Usage: $0 <path_to_rosbag.bag>"
    exit 1
fi

# The bag file is the first argument passed to the script.
BAG_FILE="$1"

# Check if the provided file actually exists.
if [ ! -f "$BAG_FILE" ]; then
    echo "Error: File not found at '$BAG_FILE'"
    exit 1
fi

# Check if the file appears to be a rosbag.
if [[ "$BAG_FILE" != *.bag ]]; then
    echo "Warning: The provided file '$BAG_FILE' does not end with .bag."
    # You can exit here if you want to be strict, but a warning is often enough.
    # exit 1
fi


# Construct the output CSV filename by replacing the .bag extension with .csv
OUTPUT_CSV="${BAG_FILE%.bag}.csv"

echo "Processing ROS Bag:   $BAG_FILE"
echo "Exporting topic '$TOPIC' to: $OUTPUT_CSV"

# Use rostopic echo with the -p flag for CSV-style output.
# The output is piped to `sed` to remove the initial '%' from the header line,
# which can cause issues with some CSV parsers. Finally, the result is
# redirected to the new CSV file.
rostopic echo -b "$BAG_FILE" -p "$TOPIC" | sed '1s/^%//' > "$OUTPUT_CSV"

# Check if the CSV file was created and is not empty.
if [ -s "$OUTPUT_CSV" ]; then
    echo "Successfully created $OUTPUT_CSV"
else
    # If the file is empty, it likely means the topic was not in the bag.
    echo "Warning: Output file is empty. The topic '$TOPIC' might not exist in '$BAG_FILE'."
    # Uncomment the next line to automatically delete empty CSV files.
    # rm "$OUTPUT_CSV"
fi

echo "--------------------------------------------------"
echo "Script finished."```

### How to Use the New Script

# 1.  **Save the File**: Make sure this code is saved as `extract_odometry.sh`.
# 2.  **Make it Executable** (if you haven't already):
#     ```bash
#     chmod +x extract_odometry.sh
#     ```
# 3.  **Run it with an Argument**: Execute the script from within your Docker container, followed by the path to the specific bag file you want to process.

#     **Example:**
#     Let's say you are in the `/root/rosbags` directory and you have a file named `flight_01.bag`.
#     ```bash
#     # Make sure you are in the correct directory
#     cd /root/rosbags

#     # Run the script on your specific bag file
#     ./extract_odometry.sh flight_01.bag
#     ```
#     This will create a new file named `flight_01.csv` in that same `/root/rosbags` directory.

#     If the bag is in a subdirectory, the script will still place the CSV right next to it:
#     ```bash
#     ./extract_odometry.sh day_5/indoor_run.bag
#     ```
#     This will create `day_5/indoor_run.csv`.