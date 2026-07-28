import sys

import numpy as np
import open3d as o3d

sys.path.append(".")
import pyransac3d as pyrsc

# Live-plot each RANSAC candidate while it is being evaluated: the box keeps
# its real point colors, the current iteration's inliers are highlighted in
# red, the best inliers found so far in green, and the sampled points that
# generated the current candidate in blue.
pcd_load = o3d.io.read_point_cloud("tests/dataset/caixa.ply")
points = np.asarray(pcd_load.points)
n_points = points.shape[0]
original_colors = np.asarray(pcd_load.colors)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
pcd.colors = o3d.utility.Vector3dVector(original_colors)

vis = o3d.visualization.Visualizer()
vis.create_window(window_name="RANSAC plane fitting - live view")
vis.add_geometry(pcd)


def plot_callback(state):
    colors = original_colors.copy()
    colors[state["best_inliers"]] = [0.0, 1.0, 0.0]
    colors[state["inliers"]] = [1.0, 0.0, 0.0]
    colors[state["sample_indices"]] = [0.1, 0.3, 0.9]
    pcd.colors = o3d.utility.Vector3dVector(colors)

    vis.update_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()

    return False  # Return True here (or from a custom condition) to stop fitting early.


plane1 = pyrsc.Plane()
best_eq, best_inliers = plane1.fit(points, thresh=0.01, maxIteration=1000, callback=plot_callback)

print(f"Plane equation: {best_eq}")
print(f"Inliers: {len(best_inliers)}")

vis.run()
vis.destroy_window()
