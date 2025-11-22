import pandas as pd
import matplotlib.pyplot as plt
import argparse
import os.path

# --- Configuration ---
X_COLUMN = 'field.pose.pose.position.x'
Y_COLUMN = 'field.pose.pose.position.y'

# --- Main Script Logic ---
def plot_trajectory(csv_file_path):
    """Reads odometry data from a CSV and plots the 2D trajectory."""
    
    if not os.path.exists(csv_file_path):
        print(f"Error: The file '{csv_file_path}' was not found.")
        return

    try:
        df = pd.read_csv(csv_file_path, index_col=0)
        df.columns = df.columns.str.strip()

        print(f"Successfully loaded '{csv_file_path}'.")
        print("Available columns are:")
        print(df.columns)

        if X_COLUMN not in df.columns or Y_COLUMN not in df.columns:
            print(f"\nError: One or both of the required columns ('{X_COLUMN}', '{Y_COLUMN}') were not found.")
            return

        plt.figure(figsize=(10, 8))
        
        # --- THIS IS THE LINE THAT HAS BEEN CHANGED ---
        # We add .to_numpy() to convert the pandas Series to a numpy array before plotting
        plt.plot(df[X_COLUMN].to_numpy(), df[Y_COLUMN].to_numpy(), label='VINS Odometry')
        
        # Plot start and end points (these lines don't need changing)
        plt.plot(df[X_COLUMN].iloc[0], df[Y_COLUMN].iloc[0], 'go', markersize=10, label='Start')
        plt.plot(df[X_COLUMN].iloc[-1], df[Y_COLUMN].iloc[-1], 'rx', markersize=10, label='End')
        
        # Set plot properties
        plt.title('VINS-Fusion 2D Odometry Trajectory')
        plt.xlabel('X Position (meters)')
        plt.ylabel('Y Position (meters)')
        plt.grid(True)
        plt.legend()
        plt.axis('equal')
        
        # Show the plot
        plt.show()

    except Exception as e:
        print(f"An error occurred while processing the file: {e}")

# --- Argument Parsing and Execution ---
if __name__ == "__main__":
    # --- This part assumes you are passing the file path as an argument ---
    # Set up the argument parser
    parser = argparse.ArgumentParser(description="Plot 2D odometry from a VINS-Fusion CSV file.")
    parser.add_argument("csv_file", type=str, help="The path to the input CSV file.")
    args = parser.parse_args()
    plot_trajectory(args.csv_file)