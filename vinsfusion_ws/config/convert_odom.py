import pandas as pd
import numpy as np
# The argparse library is needed for command-line arguments
import argparse

def convert_odometry_to_gps(input_csv_path, output_csv_path, initial_lat, initial_lon,
                           initial_heading_from_north, scale_factor=1.0):
    """
    Converts VINS-Fusion odometry data from a ROS topic CSV to a GPS track.

    Assumes ROS ENU (East-North-Up) coordinate convention:
    - X axis points East
    - Y axis points North
    - Z axis points Up

    Args:
        input_csv_path (str): Path to the input CSV with odometry data.
        output_csv_path (str): Path to save the resulting GPS data.
        initial_lat (float): Starting latitude in degrees.
        initial_lon (float): Starting longitude in degrees.
        initial_heading_from_north (float): Initial travel direction in degrees clockwise from North (0=N, 90=E, 180=S, 270=W).
        scale_factor (float): Multiplicative scale factor to apply to odometry distances.
                             Use < 1.0 if odometry is too large (e.g., 0.01 if data is in cm).
    """
    try:
        print("Loading odometry data...")
        df = pd.read_csv(input_csv_path)
    except FileNotFoundError:
        print(f"Error: The file was not found at {input_csv_path}")
        print("Please make sure the path is correct.")
        return

    # --- Step 1: Extract Odometry Positions (relative to first measurement) ---
    start_odom_x = df['field.pose.pose.position.x'].iloc[0]
    start_odom_y = df['field.pose.pose.position.y'].iloc[0]

    # Apply scale factor immediately after extraction
    df['x_relative'] = (df['field.pose.pose.position.x'] - start_odom_x) * scale_factor
    df['y_relative'] = (df['field.pose.pose.position.y'] - start_odom_y) * scale_factor

    print(f"Trajectory origin set to ({start_odom_x:.4f}, {start_odom_y:.4f})")
    print(f"Scale factor applied: {scale_factor}")

    # Calculate raw trajectory extent for diagnostics
    max_x = df['x_relative'].abs().max()
    max_y = df['y_relative'].abs().max()
    max_extent = max(max_x, max_y)
    print(f"Maximum trajectory extent after scaling: {max_extent:.2f} meters")

    # --- Step 2: Extract Initial Heading from Quaternion ---
    q_x = df['field.pose.pose.orientation.x'].iloc[0]
    q_y = df['field.pose.pose.orientation.y'].iloc[0]
    q_z = df['field.pose.pose.orientation.z'].iloc[0]
    q_w = df['field.pose.pose.orientation.w'].iloc[0]

    # Convert quaternion to yaw angle (rotation about Z-axis)
    siny_cosp = 2.0 * (q_w * q_z + q_x * q_y)
    cosy_cosp = 1.0 - 2.0 * (q_y * q_y + q_z * q_z)
    odom_heading_rad = np.arctan2(siny_cosp, cosy_cosp)

    print(f"Initial odometry heading: {np.rad2deg(odom_heading_rad):.2f}° (in odometry frame)")

    # --- Step 3: Calculate Rotation to Align with GPS Frame ---
    gps_heading_rad = np.deg2rad(initial_heading_from_north)
    enu_heading_rad = np.pi/2.0 - gps_heading_rad  # Convert GPS to ENU convention

    rotation_angle_rad = enu_heading_rad - odom_heading_rad

    print(f"GPS heading (from North): {initial_heading_from_north}°")
    print(f"ENU heading (from East): {np.rad2deg(enu_heading_rad):.2f}°")
    print(f"Rotation angle to align: {np.rad2deg(rotation_angle_rad):.2f}°")

    # --- Step 4: Rotate Trajectory to GPS-Aligned Frame ---
    cos_rot = np.cos(rotation_angle_rad)
    sin_rot = np.sin(rotation_angle_rad)

    df['x_aligned'] = df['x_relative'] * cos_rot - df['y_relative'] * sin_rot
    df['y_aligned'] = df['x_relative'] * sin_rot + df['y_relative'] * cos_rot

    print("Trajectory rotated to align with GPS frame (ENU coordinates).")

    # --- Step 5: Convert Meter Offsets to Lat/Lon Changes ---
    EARTH_RADIUS = 6378137.0  # Earth's radius in meters (WGS84)

    lat_rad = np.deg2rad(initial_lat)
    meters_per_deg_lat = EARTH_RADIUS * np.pi / 180.0
    meters_per_deg_lon = EARTH_RADIUS * np.pi / 180.0 * np.cos(lat_rad)

    print(f"At latitude {initial_lat}°:")
    print(f"  Meters per degree latitude: {meters_per_deg_lat:.2f}")
    print(f"  Meters per degree longitude: {meters_per_deg_lon:.2f}")

    df['delta_lat'] = df['y_aligned'] / meters_per_deg_lat
    df['delta_lon'] = df['x_aligned'] / meters_per_deg_lon

    df['gps_latitude'] = initial_lat + df['delta_lat']
    df['gps_longitude'] = initial_lon + df['delta_lon']

    print("Conversion to GPS coordinates complete.")

    # --- Step 6: Calculate and Display Statistics ---
    total_distance = np.sqrt(df['x_aligned'].iloc[-1]**2 + df['y_aligned'].iloc[-1]**2)
    max_north = df['y_aligned'].max()
    max_east = df['x_aligned'].max()
    min_north = df['y_aligned'].min()
    min_east = df['x_aligned'].min()

    print(f"\nTrajectory Statistics:")
    print(f"  Total points: {len(df)}")
    print(f"  Straight-line distance: {total_distance:.2f} meters")
    print(f"  East extent: {min_east:.2f} to {max_east:.2f} meters (range: {max_east-min_east:.2f}m)")
    print(f"  North extent: {min_north:.2f} to {max_north:.2f} meters (range: {max_north-min_north:.2f}m)")
    print(f"  Lat range: {df['gps_latitude'].min():.6f} to {df['gps_latitude'].max():.6f}")
    print(f"  Lon range: {df['gps_longitude'].min():.6f} to {df['gps_longitude'].max():.6f}")

    # --- Step 7: Save Results ---
    output_df = df[['gps_latitude', 'gps_longitude']].copy()
    output_df.to_csv(output_csv_path, index=False)
    print(f"\nSuccess! GPS track saved to {output_csv_path}")

    debug_output = output_csv_path.replace('.csv', '_debug.csv')
    debug_df = df[['x_relative', 'y_relative', 'x_aligned', 'y_aligned',
                   'delta_lat', 'delta_lon', 'gps_latitude', 'gps_longitude']].copy()
    debug_df.to_csv(debug_output, index=False)
    print(f"Debug data saved to {debug_output}")


# ==============================================================================
# --- SCALE CALIBRATION HELPER (Unchanged) ---
# ==============================================================================
# def estimate_scale_factor(input_csv_path, known_north_south_meters, known_east_west_meters):
#     """
#     Estimate the scale factor needed by comparing odometry extent to known GPS extent.
#     ... (rest of the function is unchanged)
#     """
#     df = pd.read_csv(input_csv_path)
#     start_x = df['field.pose.pose.position.x'].iloc[0]
#     start_y = df['field.pose.pose.position.y'].iloc[0]
#     x_rel = df['field.pose.pose.position.x'] - start_x
#     y_rel = df['field.pose.pose.position.y'] - start_y
#     odom_extent_x = x_rel.max() - x_rel.min()
#     odom_extent_y = y_rel.max() - y_rel.min()
#     print(f"\nScale Factor Estimation:")
#     print(f"  Raw odometry extent: {odom_extent_x:.2f} × {odom_extent_y:.2f} units")
#     print(f"  Known GPS extent: {known_east_west_meters:.2f} × {known_north_south_meters:.2f} meters")
#     scale_x = known_east_west_meters / odom_extent_x if odom_extent_x > 0 else 1.0
#     scale_y = known_north_south_meters / odom_extent_y if odom_extent_y > 0 else 1.0
#     suggested_scale = (scale_x + scale_y) / 2.0
#     print(f"  Scale factor (East-West): {scale_x:.6f}")
#     print(f"  Scale factor (North-South): {scale_y:.6f}")
#     print(f"  Suggested scale factor: {suggested_scale:.6f}")
#     if 0.008 < suggested_scale < 0.012:
#         print(f"  → Likely unit: CENTIMETERS (use SCALE_FACTOR = 0.01)")
#     elif 0.0008 < suggested_scale < 0.0012:
#         print(f"  → Likely unit: MILLIMETERS (use SCALE_FACTOR = 0.001)")
#     else:
#         print(f"  → Custom scale factor needed: {suggested_scale:.6f}")
#     return suggested_scale


# ==============================================================================
# --- MAIN EXECUTION BLOCK (MODIFIED TO ACCEPT COMMAND-LINE ARGUMENTS) ---
# ==============================================================================
if __name__ == '__main__':
    # Set up the argument parser to read command-line arguments
    parser = argparse.ArgumentParser(
        description="Converts VINS-Fusion odometry from a CSV to a GPS track.",
        formatter_class=argparse.RawTextHelpFormatter
    )

    # Define the arguments the script will accept
    parser.add_argument('--input', type=str, required=True,
                        help='Path to the input odometry CSV file.')
    parser.add_argument('--output', type=str, required=True,
                        help='Path to save the output GPS track CSV file.')
    parser.add_argument('--lat', type=float, required=True,
                        help='Starting latitude in degrees.')
    parser.add_argument('--lon', type=float, required=True,
                        help='Starting longitude in degrees.')
    parser.add_argument('--heading', type=float, required=True,
                        help='Initial heading in degrees clockwise from North (e.g., 0=N, 90=E).')
    parser.add_argument('--scale', type=float, default=1.0,
                        help='Multiplicative scale factor for odometry distances (default: 1.0).')

    # Parse the arguments provided by the user
    args = parser.parse_args()

    # Print a summary of the provided arguments
    print("\n--- Running Odometry to GPS Conversion ---")
    print(f"  Input File:      {args.input}")
    print(f"  Output File:     {args.output}")
    print(f"  Start Latitude:  {args.lat}")
    print(f"  Start Longitude: {args.lon}")
    print(f"  Start Heading:   {args.heading}°")
    print(f"  Scale Factor:    {args.scale}\n")

    # Call the main conversion function using the arguments from the command line
    convert_odometry_to_gps(
        input_csv_path=args.input,
        output_csv_path=args.output,
        initial_lat=args.lat,
        initial_lon=args.lon,
        initial_heading_from_north=args.heading,
        scale_factor=args.scale
    )