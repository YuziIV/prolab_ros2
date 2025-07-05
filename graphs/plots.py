import pandas as pd
import matplotlib.pyplot as plt
import os

# Added a parameter for the directory where plots will be saved
def plot_robot_paths(csv_file_name="paths.csv", save_directory="plots_output"):
    """
    Reads a CSV file containing robot pose data and plots their trajectories.
    The generated plot is also saved to a specified directory.

    Args:
        csv_file_name (str): The name of the CSV file (e.g., "paths.csv").
                             The script determines its path relative to itself.
        save_directory (str): The name of the directory where the plot image will be saved.
                              This directory will be created relative to the script's location.
    """
    # Get the directory where the current script is located
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Construct the absolute path to paths.csv based on your specified structure
    csv_file_path = os.path.join(script_dir, '..', 'ros2_ws', csv_file_name)

    if not os.path.exists(csv_file_path):
        print(f"Error: The file '{csv_file_path}' was not found.")
        print("Please ensure 'paths.csv' is located at '/home/yuzi/.localgit/prolab_ros2/ros2_ws/paths.csv'")
        print("and this script ('plots.py') is at '/home/yuzi/.localgit/prolab_ros2/graphs/plots.py'.")
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

    plt.figure(figsize=(10, 8))

    plt.plot(df['gt_x'], df['gt_y'], label='Ground Truth', color='black', linestyle='-')
    plt.plot(df['kf_x'], df['kf_y'], label='Kalman Filter', color='blue', linestyle='--')
    plt.plot(df['ekf_x'], df['ekf_y'], label='Extended Kalman Filter', color='red', linestyle='-.')
    plt.plot(df['pf_x'], df['pf_y'], label='Particle Filter', color='green', linestyle=':')

    plt.scatter(df['gt_x'].iloc[0], df['gt_y'].iloc[0], color='black', marker='o', s=100, label='Start')
    plt.scatter(df['gt_x'].iloc[-1], df['gt_y'].iloc[-1], color='black', marker='x', s=100, label='End')

    plt.title('Robot Trajectory Comparison')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.grid(True)
    plt.axis('equal')
    plt.legend()
    plt.tight_layout()

    # --- New code to save the plot ---
    # Define the full path for the save directory (relative to the script)
    full_save_dir = os.path.join(script_dir, save_directory)
    os.makedirs(full_save_dir, exist_ok=True) # Create the directory if it doesn't exist

    # Define the filename for the saved plot
    plot_filename = os.path.join(full_save_dir, "kidnapped.png")

    # Save the plot
    plt.savefig(plot_filename)
    print(f"Graph saved to: {plot_filename}")
    # --- End of new code ---

    plt.show() # Display the plot window

if __name__ == "__main__":
    # The default save_directory is 'plots_output', which will be created
    # in the same directory as your 'plots.py' script.
    plot_robot_paths()