import time
from typing import Any

import numpy as np
import open3d as o3d

import pyransac3d as pyrsc

# Live-plot each RANSAC candidate while it is being evaluated: the current iteration's inliers are
# highlighted in red, the best inliers found so far in green, and the point sampled to generate the
# current candidate in blue. The wireframe is the ball of radius thresh around the best candidate,
# which is the region where its neighbors are counted, and the camera turns a little on every
# iteration to show the cloud in 3d.
thresh = 0.3
points = pyrsc.ShapeGenerator(seed=0).point([0, 0, 0], n_points=600, noise=0.1, n_outliers=600, outlier_bounds=3.0)
n_points = points.shape[0]
original_colors = np.tile([0.6, 0.6, 0.6], (n_points, 1))

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
pcd.colors = o3d.utility.Vector3dVector(original_colors)

# The ball is always the same size, so it is built once around the origin and only moved from there
ball = o3d.geometry.LineSet.create_from_triangle_mesh(
    o3d.geometry.TriangleMesh.create_sphere(radius=thresh, resolution=10)
)
ball_points = np.asarray(ball.points).copy()
ball.paint_uniform_color([0.0, 0.0, 1.0])

vis = o3d.visualization.Visualizer()
vis.create_window(window_name="RANSAC point fitting - live view")
vis.add_geometry(pcd)

# The ball is somewhere else until the first candidate is found, so the camera is fit to the cloud
vis.add_geometry(ball, reset_bounding_box=False)

view_control = vis.get_view_control()

point = pyrsc.Point()


def plot_callback(state: dict[str, Any]) -> bool:
    colors = original_colors.copy()
    colors[state["best_inliers"]] = [0.0, 1.0, 0.0]
    colors[state["inliers"]] = [1.0, 0.0, 0.0]
    colors[state["sample_indices"]] = [0.1, 0.3, 0.9]
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # The ball only moves when RANSAC finds a point with more neighbors, so we redraw it just there
    if state["is_best"]:
        ball.points = o3d.utility.Vector3dVector(ball_points + state["best_model"]["center"])
        vis.update_geometry(ball)

    # Turn the camera a bit on every frame so we can see the cloud in 3d while it is fitted. The
    # argument is a horizontal drag in pixels, the same as rotating the view with the mouse
    view_control.rotate(-5.0, 0.0)

    vis.update_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()
    time.sleep(0.05)

    return False  # Return True here (or from a custom condition) to stop fitting early.


center, best_inliers = point.fit(points, thresh=thresh, maxIteration=200, callback=plot_callback)

print(f"Center: {center}")
print(f"Inliers: {len(best_inliers)} of {n_points}")

vis.run()
vis.destroy_window()
