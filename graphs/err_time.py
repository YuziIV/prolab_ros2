import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np # For handling potential NaN values in PF


def plot_pose_errors(csv_file_name="paths.csv", save_directory="error_plots"):
    """
    Reads a CSV file containing robot pose data, calculates errors relative to ground truth,
    and plots the error in x and y over time for KF, EKF, and PF.
    The generated plots are saved to a specified directory.

    Args:
        csv_file_name (str): The name of the CSV file (e.g., "paths.csv").
                             The script determines its path relative to itself.
        save_directory (str): The name of the directory where the plot images will be saved.
                              This directory will be created relative to the script's location.
    """
    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_file_path = os.path.join(script_dir, '..', 'ros2_ws', csv_file_name)

    if not os.path.exists(csv_file_path):
        print(f"Error: The file '{csv_file_path}' was not found.")
        print("Please ensure 'paths.csv' is located at '/home/yuzi/.localgit/prolab_ros2/ros2_ws/paths.csv'")
        print("and this script ('plot_errors.py') is at '/home/yuzi/.localgit/prolab_ros2/graphs/plot_errors.py'.")
        return

    try:
        df = pd.read_csv(csv_file_path)
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        print("Please ensure the CSV file is correctly formatted.")
        return

    required_columns = ['timestamp', 'gt_x', 'gt_y', 'kf_x', 'kf_y', 'ekf_x', 'ekf_y', 'pf_x', 'pf_y']
    if not all(col in df.columns for col in required_columns):
        print("Error: Missing one or more required columns in the CSV file.")
        print(f"Expected columns: {', '.join(required_columns)}")
        print(f"Found columns: {', '.join(df.columns)}")
        return

    # Calculate errors
    df['kf_error_x'] = df['kf_x'] - df['gt_x']
    df['kf_error_y'] = df['kf_y'] - df['gt_y']

    df['ekf_error_x'] = df['ekf_x'] - df['gt_x']
    df['ekf_error_y'] = df['ekf_y'] - df['gt_y']

    # For PF, handle potential NaN values if it wasn't always available in the logs
    df['pf_error_x'] = df['pf_x'] - df['gt_x']
    df['pf_error_y'] = df['pf_y'] - df['gt_y']

    # --- Plotting Error in X ---
    plt.figure(figsize=(12, 6))
    #plt.plot(df['timestamp'], df['kf_error_x'], label='KF Error X', linestyle='--', color='blue')
    plt.plot(df['timestamp'], df['ekf_error_x'], label='EKF Error X', linestyle='-.', color='red')
    # Filter out NaNs for plotting PF error if they exist, to ensure clean lines
    plt.plot(df['timestamp'], df['pf_error_x'], label='PF Error X', linestyle=':', color='green')
    plt.axhline(0, color='orange', linestyle='-', linewidth=1.5) # Highlighted 0 error line in orange
    plt.title('Error in X Position Over Time')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Error in X (m)')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    # Save X error plot
    full_save_dir = os.path.join(script_dir, save_directory)
    os.makedirs(full_save_dir, exist_ok=True)
    plot_filename_x = os.path.join(full_save_dir, "kidnapped_error_x_over_time_wo_kf.png")
    plt.savefig(plot_filename_x)
    print(f"Graph saved to: {plot_filename_x}")
    plt.show() # Display immediately after saving

    # --- Plotting Error in Y ---
    plt.figure(figsize=(12, 6))
    #plt.plot(df['timestamp'], df['kf_error_y'], label='KF Error Y', linestyle='--', color='blue')
    plt.plot(df['timestamp'], df['ekf_error_y'], label='EKF Error Y', linestyle='-.', color='red')
    # Filter out NaNs for plotting PF error if they exist
    plt.plot(df['timestamp'], df['pf_error_y'], label='PF Error Y', linestyle=':', color='green')
    plt.axhline(0, color='orange', linestyle='-', linewidth=1.5) # Highlighted 0 error line in orange
    plt.title('Error in Y Position Over Time')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Error in Y (m)')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    # Save Y error plot
    plot_filename_y = os.path.join(full_save_dir, "kidnapped_error_y_over_time_wo_kf.png")
    plt.savefig(plot_filename_y)
    print(f"Graph saved to: {plot_filename_y}")
    plt.show() # Display immediately after saving


if __name__ == "__main__":
    plot_pose_errors()