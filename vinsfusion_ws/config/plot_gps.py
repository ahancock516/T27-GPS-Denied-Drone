import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def compare_gps_tracks(odom_csv_path, drone_csv_path):
    """
    Compares odometry-derived GPS track with drone GPS track.
    
    Args:
        odom_csv_path (str): Path to converted odometry GPS CSV
        drone_csv_path (str): Path to drone GPS CSV
    """
    try:
        print("Loading GPS tracks...")
        odom_df = pd.read_csv(odom_csv_path)
        drone_df = pd.read_csv(drone_csv_path)
        print(f"  Odometry track: {len(odom_df)} points")
        print(f"  Drone GPS track: {len(drone_df)} points")
    except FileNotFoundError as e:
        print(f"Error: File not found - {e}")
        return
    except Exception as e:
        print(f"Error loading data: {e}")
        return
    
    # Extract coordinates
    odom_lat = odom_df['gps_latitude'].to_numpy()
    odom_lon = odom_df['gps_longitude'].to_numpy()
    drone_lat = drone_df['field.latitude'].to_numpy()
    drone_lon = drone_df['field.longitude'].to_numpy()
    
    # Calculate centroids
    odom_center_lat = np.mean(odom_lat)
    odom_center_lon = np.mean(odom_lon)
    drone_center_lat = np.mean(drone_lat)
    drone_center_lon = np.mean(drone_lon)
    
    # Calculate offset between centroids
    offset_lat = drone_center_lat - odom_center_lat
    offset_lon = drone_center_lon - odom_center_lon
    
    # Convert offset to meters
    lat_offset_m = offset_lat * 111111
    lon_offset_m = offset_lon * 111111 * np.cos(np.deg2rad(drone_center_lat))
    total_offset_m = np.sqrt(lat_offset_m**2 + lon_offset_m**2)
    
    # Calculate bearing of offset
    bearing = np.degrees(np.arctan2(lon_offset_m, lat_offset_m))
    if bearing < 0:
        bearing += 360
    
    # Calculate extents
    odom_lat_range_m = (odom_lat.max() - odom_lat.min()) * 111111
    odom_lon_range_m = (odom_lon.max() - odom_lon.min()) * 111111 * np.cos(np.deg2rad(odom_center_lat))
    drone_lat_range_m = (drone_lat.max() - drone_lat.min()) * 111111
    drone_lon_range_m = (drone_lon.max() - drone_lon.min()) * 111111 * np.cos(np.deg2rad(drone_center_lat))
    
    # Print alignment analysis
    print("\n" + "="*70)
    print("GPS TRACK ALIGNMENT ANALYSIS")
    print("="*70)
    print(f"\nOdometry Track Center: {odom_center_lat:.6f}, {odom_center_lon:.6f}")
    print(f"Drone GPS Center:      {drone_center_lat:.6f}, {drone_center_lon:.6f}")
    print(f"\nCentroid Offset: {total_offset_m:.2f} meters at bearing {bearing:.1f}°")
    print(f"  Latitude offset:  {offset_lat:.6f}° ({lat_offset_m:+.2f} meters {'North' if lat_offset_m > 0 else 'South'})")
    print(f"  Longitude offset: {offset_lon:.6f}° ({lon_offset_m:+.2f} meters {'East' if lon_offset_m > 0 else 'West'})")
    
    print(f"\nTrack Extents:")
    print(f"  Odometry: {odom_lat_range_m:.1f}m (N-S) × {odom_lon_range_m:.1f}m (E-W)")
    print(f"  Drone:    {drone_lat_range_m:.1f}m (N-S) × {drone_lon_range_m:.1f}m (E-W)")
    
    print(f"\n{'RECOMMENDED CORRECTION:':-^70}")
    print(f"In your convert_odometry_to_gps script, update:")
    current_start_lat = odom_lat[0]
    current_start_lon = odom_lon[0]
    corrected_start_lat = current_start_lat + offset_lat
    corrected_start_lon = current_start_lon + offset_lon
    
    print(f"  START_LATITUDE  = {corrected_start_lat:.6f}  (was {current_start_lat:.6f})")
    print(f"  START_LONGITUDE = {corrected_start_lon:.6f}  (was {current_start_lon:.6f})")
    print("="*70 + "\n")
    
    # Create comparison plot
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
    
    # Plot 1: Original alignment
    ax1.plot(drone_lon, drone_lat, 'r-', linewidth=2.5, 
             label='Drone GPS', alpha=0.8, zorder=2)
    ax1.plot(odom_lon, odom_lat, 'b--', linewidth=2, 
             label='Odometry GPS', alpha=0.7, zorder=1)
    
    # Mark start points
    ax1.plot(drone_lon[0], drone_lat[0], 'go', markersize=12, 
             label='Drone Start', zorder=5)
    ax1.plot(odom_lon[0], odom_lat[0], 'bo', markersize=10, 
             label='Odom Start', zorder=4)
    
    # Mark end points
    ax1.plot(drone_lon[-1], drone_lat[-1], 'rx', markersize=12, 
             markeredgewidth=3, label='Drone End', zorder=5)
    ax1.plot(odom_lon[-1], odom_lat[-1], 'bx', markersize=10, 
             markeredgewidth=2, label='Odom End', zorder=4)
    
    # Draw offset arrow between centroids
    ax1.annotate('', xy=(drone_center_lon, drone_center_lat),
                xytext=(odom_center_lon, odom_center_lat),
                arrowprops=dict(arrowstyle='->', lw=2.5, color='orange'))
    ax1.text((odom_center_lon + drone_center_lon)/2, 
             (odom_center_lat + drone_center_lat)/2,
             f'  {total_offset_m:.1f}m\n  offset', 
             fontsize=11, color='orange', fontweight='bold',
             bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    ax1.set_xlabel('Longitude', fontsize=12)
    ax1.set_ylabel('Latitude', fontsize=12)
    ax1.set_title('Current Alignment (BEFORE Correction)', fontsize=14, fontweight='bold')
    ax1.legend(fontsize=10, loc='best')
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    # Plot 2: Corrected alignment preview
    odom_lat_corrected = odom_lat + offset_lat
    odom_lon_corrected = odom_lon + offset_lon
    
    ax2.plot(drone_lon, drone_lat, 'r-', linewidth=2.5, 
             label='Drone GPS', alpha=0.8, zorder=2)
    ax2.plot(odom_lon_corrected, odom_lat_corrected, 'b--', linewidth=2, 
             label='Odometry GPS (corrected)', alpha=0.7, zorder=1)
    
    # Mark start points
    ax2.plot(drone_lon[0], drone_lat[0], 'go', markersize=12, 
             label='Drone Start', zorder=5)
    ax2.plot(odom_lon_corrected[0], odom_lat_corrected[0], 'bo', 
             markersize=10, label='Odom Start (corrected)', zorder=4)
    
    # Mark end points
    ax2.plot(drone_lon[-1], drone_lat[-1], 'rx', markersize=12, 
             markeredgewidth=3, label='Drone End', zorder=5)
    ax2.plot(odom_lon_corrected[-1], odom_lat_corrected[-1], 'bx', 
             markersize=10, markeredgewidth=2, label='Odom End (corrected)', zorder=4)
    
    ax2.set_xlabel('Longitude', fontsize=12)
    ax2.set_ylabel('Latitude', fontsize=12)
    ax2.set_title('Preview AFTER Correction', fontsize=14, fontweight='bold')
    ax2.legend(fontsize=10, loc='best')
    ax2.grid(True, alpha=0.3)
    ax2.axis('equal')
    
    plt.tight_layout()
    plt.savefig('/rosbags/rosbags/ros_bags/gps_comparison.png', dpi=150, bbox_inches='tight')
    print(f"Comparison plot saved to: gps_comparison.png")
    plt.show()


def plot_single_gps_trajectory(csv_path):
    """
    Original function: plots a single GPS trajectory.
    
    Args:
        csv_path (str): Path to GPS CSV file with 'gps_latitude' and 'gps_longitude' columns
    """
    try:
        df = pd.read_csv(csv_path)
        print(f"Successfully loaded data from {csv_path}")
    except FileNotFoundError:
        print(f"Error: The file was not found at '{csv_path}'")
        return
    
    lat = df['gps_latitude'].to_numpy()
    lon = df['gps_longitude'].to_numpy()
    
    plt.figure(figsize=(10, 10))
    plt.plot(lon, lat, marker='.', linestyle='-', label='GPS Trajectory', markersize=3)
    plt.plot(lon[0], lat[0], 'go', markersize=10, label='Start')
    plt.plot(lon[-1], lat[-1], 'rx', markersize=10, mew=2, label='End')
    
    plt.title('Estimated GPS Trajectory from Odometry')
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.show()


# ==============================================================================
# --- MAIN EXECUTION BLOCK ---
# ==============================================================================
if __name__ == '__main__':
    # Paths to your GPS files
    ODOMETRY_GPS_FILE = '/rosbags/rosbags/ros_bags/gps_track_corrected.csv'
    DRONE_GPS_FILE = '/rosbags/rosbags/ros_bags/bus_loop_1_gps.csv'  # <-- UPDATE THIS
    
    # Choose mode:
    MODE = 'compare'  # Options: 'compare' or 'single'
    
    if MODE == 'compare':
        compare_gps_tracks(ODOMETRY_GPS_FILE, DRONE_GPS_FILE)
    else:
        plot_single_gps_trajectory(ODOMETRY_GPS_FILE)