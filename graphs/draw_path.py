import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import os
import numpy as np
import matplotlib.patheffects as pe # Import patheffects for text outlines

def plot_paths_with_map(csv_file_name="paths.csv", map_image_name="playground_map_hq_edited.pgm", save_directory="map_plots", map_resolution=0.05):
    """
    Reads robot pose data and plots their trajectories on top of a map image.
    It aligns the (0,0) point of the robot's coordinates with the geometric center of the map image.
    The map image is flipped vertically (along the x-axis) before plotting.
    The plot is then zoomed to display a range from -10 to 10 on both x and y axes.
    Kidnapping points are marked and labeled with an outline.

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
        # Flip the image vertically (around the x-axis)
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
    ax.plot(df['kf_x'], df['kf_y'], label='Kalman Filter', color='blue', linestyle='--') # KF commented out in user's latest code
    ax.plot(df['ekf_x'], df['ekf_y'], label='Extended Kalman Filter', color='red', linestyle='-.')
    ax.plot(df['pf_x'], df['pf_y'], label='Particle Filter', color='green', linestyle=':')

    # Add start and end points for Ground Truth
    ax.scatter(df['gt_x'].iloc[0], df['gt_y'].iloc[0], color='orange', marker='o', s=100, label='Start', zorder=5,
               path_effects=[pe.Stroke(linewidth=3, foreground='black'), pe.Normal()])
    ax.scatter(df['gt_x'].iloc[-1], df['gt_y'].iloc[-1], color='orange', marker='x', s=100, label='End', zorder=5,
               path_effects=[pe.Stroke(linewidth=3, foreground='black'), pe.Normal()])

    # # --- Highlight Kidnapping Points ---
    # # KNP 1
    # knp1_orig_x, knp1_orig_y = -3.4, 0.4
    # knp1_dest_x, knp1_dest_y = -4.74, 2.87
    # ax.scatter([knp1_orig_x, knp1_dest_x], [knp1_orig_y, knp1_dest_y],
    #            color='magenta', s=150, marker='X', zorder=6, label='Kidnapping Events')

    # ax.plot([knp1_orig_x, knp1_dest_x], [knp1_orig_y, knp1_dest_y],
    #         color='magenta', linestyle='--', linewidth=1.5, zorder=6)

    # # KNP 1 Annotation with outline
    # ax.annotate('KNP 1', xy=(knp1_dest_x, knp1_dest_y), xytext=(knp1_dest_x + 0.5, knp1_dest_y + 0.5),
    #             arrowprops=dict(facecolor='magenta', shrink=0.05, width=2, headwidth=8),
    #             color='magenta', fontsize=10, fontweight='bold', zorder=6,
    #             # Add path effects for outline
    #             path_effects=[pe.Stroke(linewidth=2, foreground='black'), pe.Normal()])

    # # KNP 2
    # knp2_orig_x, knp2_orig_y = 4.69, -3.01
    # knp2_dest_x, knp2_dest_y = 3.15, -2.96
    # ax.scatter([knp2_orig_x, knp2_dest_x], [knp2_orig_y, knp2_dest_y],
    #            color='cyan', s=150, marker='X', zorder=6)

    # ax.plot([knp2_orig_x, knp2_dest_x], [knp2_orig_y, knp2_dest_y],
    #         color='cyan', linestyle='--', linewidth=1.5, zorder=6)

    # # KNP 2 Annotation with outline
    # ax.annotate('KNP 2', xy=(knp2_dest_x, knp2_dest_y), xytext=(knp2_dest_x + 0.5, knp2_dest_y - 0.5),
    #             arrowprops=dict(facecolor='cyan', shrink=0.05, width=2, headwidth=8),
    #             color='cyan', fontsize=10, fontweight='bold', zorder=6,
    #             # Add path effects for outline
    #             path_effects=[pe.Stroke(linewidth=2, foreground='black'), pe.Normal()])
    # # --- End Kidnapping Points ---


    plt.title('Robot Trajectory Comparison with Map Background')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.grid(True)
    plt.axis('equal')

    # Set axis limits to zoom in
    plt.xlim(-11, 15)
    plt.ylim(-11, 11)

    plt.legend()
    plt.tight_layout()

    # --- Save Plot ---
    full_save_dir = os.path.join(script_dir, save_directory)
    os.makedirs(full_save_dir, exist_ok=True)
    plot_filename = os.path.join(full_save_dir, "trajectories_on_map_fixed_KF.png")
    plt.savefig(plot_filename)
    print(f"Graph saved to: {plot_filename}")

    plt.show()

if __name__ == "__main__":
    plot_paths_with_map()