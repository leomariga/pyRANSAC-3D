import time

import numpy as np
import open3d as o3d

import pyransac3d as pyrsc

# Live-plot each RANSAC candidate while it is being evaluated: the current iteration's inliers are
# highlighted in red, the best inliers found so far in green, and the 2 sampled points that
# generated the current candidate in blue. The blue segment is the line of the best candidate so
# far, and the camera turns a little on every iteration to show the cloud in 3d.
length = 10.0
points = pyrsc.ShapeGenerator(seed=0).line(
    [0, 0, 0], [1, 1, 0.5], length=length, n_points=600, noise=0.1, n_outliers=400
)
n_points = points.shape[0]
original_colors = np.tile([0.6, 0.6, 0.6], (n_points, 1))

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
pcd.colors = o3d.utility.Vector3dVector(original_colors)

# The segment starts collapsed on the origin and is only stretched once a candidate is found
segment = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(np.zeros((2, 3))),
    lines=o3d.utility.Vector2iVector([[0, 1]]),
)
segment.paint_uniform_color([0.0, 0.0, 1.0])

vis = o3d.visualization.Visualizer()
vis.create_window(window_name="RANSAC line fitting - live view")
vis.add_geometry(pcd)

# The empty segment would ruin the framing of the scene, so the camera is fit to the cloud only
vis.add_geometry(segment, reset_bounding_box=False)

view_control = vis.get_view_control()

line = pyrsc.Line()


def plot_callback(state):
    colors = original_colors.copy()
    colors[state["best_inliers"]] = [0.0, 1.0, 0.0]
    colors[state["inliers"]] = [1.0, 0.0, 0.0]
    colors[state["sample_indices"]] = [0.1, 0.3, 0.9]
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # A line is infinite, so we draw the piece of it which is as long as the cloud, walking from
    # the point given by the fit to both sides along the direction it found
    if state["is_best"]:
        best_direction = np.asarray(state["best_model"]["A"])
        best_point = np.asarray(state["best_model"]["B"])
        ends = [best_point - best_direction * length / 2, best_point + best_direction * length / 2]
        segment.points = o3d.utility.Vector3dVector(ends)
        vis.update_geometry(segment)

    # Turn the camera a bit on every frame so we can see the cloud in 3d while it is fitted. The
    # argument is a horizontal drag in pixels, the same as rotating the view with the mouse
    view_control.rotate(-5.0, 0.0)

    vis.update_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()
    time.sleep(0.05)

    return False  # Return True here (or from a custom condition) to stop fitting early.


direction, anchor, best_inliers = line.fit(points, thresh=0.3, maxIteration=200, callback=plot_callback)

print(f"Direction: {direction}")
print(f"Point of the line: {anchor}")
print(f"Inliers: {len(best_inliers)} of {n_points}")

vis.run()
vis.destroy_window()
