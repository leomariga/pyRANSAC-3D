import time
from pathlib import Path
from typing import Any

import numpy as np
import open3d as o3d

import pyransac3d as pyrsc

DATASET_DIR = Path(__file__).resolve().parent / "dataset"

# Live-plot each RANSAC candidate while it is being evaluated: the box keeps
# its real point colors, the current iteration's inliers are highlighted in
# red, the best inliers found so far in green, and the sampled points that
# generated the current candidate in blue. The camera turns a little on every
# iteration to show the cloud in 3d.
pcd_load = o3d.io.read_point_cloud(str(DATASET_DIR / "caixa.ply"))
points = np.asarray(pcd_load.points)
n_points = points.shape[0]
original_colors = np.asarray(pcd_load.colors)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
pcd.colors = o3d.utility.Vector3dVector(original_colors)

vis = o3d.visualization.Visualizer()
vis.create_window(window_name="RANSAC plane fitting - live view")
vis.add_geometry(pcd)

view_control = vis.get_view_control()


def plot_callback(state: dict[str, Any]) -> bool:
    colors = original_colors.copy()
    colors[state["best_inliers"]] = [0.0, 1.0, 0.0]
    colors[state["inliers"]] = [1.0, 0.0, 0.0]
    colors[state["sample_indices"]] = [0.1, 0.3, 0.9]
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # Turn the camera a bit on every frame so we can see the cloud in 3d while it is fitted.
    # The argument is a horizontal drag in pixels, the same as rotating the view with the mouse
    view_control.rotate(-3.0, 0.0)

    vis.update_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()
    time.sleep(0.07)

    return False  # Return True here (or from a custom condition) to stop fitting early.


plane1 = pyrsc.Plane()
best_eq, best_inliers = plane1.fit(points, thresh=0.01, maxIteration=1000, callback=plot_callback)

print(f"Plane equation: {best_eq}")
print(f"Inliers: {len(best_inliers)}")

vis.run()
vis.destroy_window()
