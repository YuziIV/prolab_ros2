import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import os
import numpy as np
import matplotlib.patheffects as pe # Import patheffects for text outlines
import matplotlib.patches as patches # Import patches for drawing circles

def plot_paths_with_map(csv_file_name="paths.csv", map_image_name="playground_map_hq_edited.pgm", save_directory="map_plots", map_resolution=0.05):
    """
    Reads robot pose data and plots their trajectories on top of a map image.
    It aligns the (0,0) point of the robot's coordinates with the geometric center of the map image.
    The map image is flipped vertically (along the x-axis) before plotting.
    The plot is then zoomed to display a range from -10 to 10 on both x and y axes.
    Sensor blindspot areas are highlighted with circles.

    Args:
        csv_file_name (str): The name of the CSV file (e.g., "paths.csv").
        map_image_name (str): The name of the map image file (e.g., "playground_map_hq_edited.pgm").
        save_directory (str): The name of the directory where the plot images will be saved.
        map_resolution (float): The resolution of the map in meters/pixel.
    """
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Construct the absolute paths for the CSV and map image
    csv_file_path = os.path.join(script_dir, '..', 'ros2_ws', csv_file_name)
    map_image_path = os.path.join(script_dir, '..', 'ros2_ws', 'src', 'turtlebot3_full_bringup', 'maps', map_image_name)

    # --- Check if files exist ---
    if not os.path.exists(csv_file_path):
        print(f"Error: CSV file '{csv_file_path}' not found.")
        print("Please ensure 'paths.csv' is located at '/home/yuzi/.localgit/prolab_ros2/ros2_ws/paths.csv'")
        return
    if not os.path.exists(map_image_path):
        print(f"Error: Map image '{map_image_path}' not found.")
        print("Please ensure the map image is located at '/home/yuzi/.localgit/prolab_ros2/ros2_ws/src/turtlebot3_full_bringup/maps/playground_map_hq_edited.pgm'")
        return

    # --- Load Data ---
    try:
        df = pd.read_csv(csv_file_path)
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        return

    required_columns = ['timestamp', 'gt_x', 'gt_y', 'kf_x', 'kf_y', 'ekf_x', 'ekf_y', 'pf_x', 'pf_y']
    if not all(col in df.columns for col in required_columns):
        print("Error: Missing one or more required columns in the CSV file.")
        print(f"Expected columns: {', '.join(required_columns)}")
        print(f"Found columns: {', '.join(df.columns)}")
        return

    # --- Load Map Image ---
    try:
        map_image = mpimg.imread(map_image_path)
        # Flip the image vertically (along the x-axis)
        map_image = np.flipud(map_image)
    except Exception as e:
        print(f"Error loading or processing map image: {e}")
        return

    # Get image dimensions (height, width for grayscale image)
    img_height, img_width = map_image.shape[:2]

    # Calculate the extent for imshow to align (0,0) with the image center
    x_min_map = -img_width / 2 * map_resolution
    x_max_map = img_width / 2 * map_resolution
    y_min_map = -img_height / 2 * map_resolution
    y_max_map = img_height / 2 * map_resolution

    # --- Plotting ---
    plt.figure(figsize=(12, 10))
    ax = plt.gca() # Get current axes

    # Display the map image as background
    ax.imshow(map_image, cmap='gray', origin='lower',
              extent=[x_min_map, x_max_map, y_min_map, y_max_map])

    # Plot trajectories on top of the map
    ax.plot(df['gt_x'], df['gt_y'], label='Ground Truth', color='black', linestyle='-')
    # ax.plot(df['kf_x'], df['kf_y'], label='Kalman Filter', color='blue', linestyle='--') # KF commented out in user's latest code
    ax.plot(df['ekf_x'], df['ekf_y'], label='Extended Kalman Filter', color='red', linestyle='-.')
    ax.plot(df['pf_x'], df['pf_y'], label='Particle Filter', color='green', linestyle=':')

    # Add start and end points for Ground Truth
    ax.scatter(df['gt_x'].iloc[0], df['gt_y'].iloc[0], color='orange', marker='o', s=100, label='Start', zorder=5,
               path_effects=[pe.Stroke(linewidth=3, foreground='black'), pe.Normal()])
    ax.scatter(df['gt_x'].iloc[-1], df['gt_y'].iloc[-1], color='orange', marker='x', s=100, label='End', zorder=5,
               path_effects=[pe.Stroke(linewidth=3, foreground='black'), pe.Normal()])

    # --- Add Blindspot Circles ---
    blindspot_radius = 3.5

    # Blindspot 1 (at x -2.8 and y 0.1)
    blindspot1_center_x, blindspot1_center_y = -2.8, 0.1
    circle1 = patches.Circle((blindspot1_center_x, blindspot1_center_y), blindspot_radius,
                             edgecolor='red', facecolor='red', alpha=0.2, linestyle='--',
                             linewidth=2, label='Sensor Blindspot', zorder=2) # zorder to be above map, below trajectories
    ax.add_patch(circle1)
    # Adjusted text position for Blindspot 1
    ax.text(blindspot1_center_x + 0.5, blindspot1_center_y + 0.5, 'Blindspot 1', color='red', fontsize=9,
            ha='center', va='center', fontweight='bold', zorder=7,
            path_effects=[pe.Stroke(linewidth=2, foreground='white'), pe.Normal()])

    # Blindspot 2 (at x 4.15 and y 4.15)
    blindspot2_center_x, blindspot2_center_y = 4.15, 4.15
    circle2 = patches.Circle((blindspot2_center_x, blindspot2_center_y), blindspot_radius,
                             edgecolor='orange', facecolor='orange', alpha=0.2, linestyle='--',
                             linewidth=2, zorder=2) # No label here to avoid duplicate in legend
    ax.add_patch(circle2)
    # Adjusted text position for Blindspot 2
    ax.text(blindspot2_center_x - 1, blindspot2_center_y - 1, 'Blindspot 2', color='orange', fontsize=9,
            ha='center', va='center', fontweight='bold', zorder=7,
            path_effects=[pe.Stroke(linewidth=2, foreground='white'), pe.Normal()])
    # --- End Blindspot Circles ---


    plt.title('Robot Trajectory Comparison with Map Background')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.grid(True)
    plt.axis('equal')

    # Set axis limits to zoom in
    plt.xlim(-11, 11)
    plt.ylim(-11, 11)

    plt.legend()
    plt.tight_layout()

    # --- Save Plot ---
    full_save_dir = os.path.join(script_dir, save_directory)
    os.makedirs(full_save_dir, exist_ok=True)
    plot_filename = os.path.join(full_save_dir, "trajectories_on_map_blindspot_only.png")
    plt.savefig(plot_filename)
    print(f"Graph saved to: {plot_filename}")

    plt.show()

if __name__ == "__main__":
    plot_paths_with_map()