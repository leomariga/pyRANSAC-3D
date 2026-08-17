import time

import numpy as np
import open3d as o3d

import pyransac3d as pyrsc

# Live-plot each RANSAC candidate while it is being evaluated: the current iteration's inliers are
# highlighted in red, the best inliers found so far in green, and the 4 sampled points that
# generated the current candidate in blue. The wireframe is the sphere of the best candidate so
# far, and the camera turns a little on every iteration to show the cloud in 3d.
points = pyrsc.ShapeGenerator(seed=0).sphere([0, 0, 0], radius=3.0, n_points=1000, noise=0.05, n_outliers=500)
n_points = points.shape[0]
original_colors = np.tile([0.6, 0.6, 0.6], (n_points, 1))

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
pcd.colors = o3d.utility.Vector3dVector(original_colors)

# Every sphere is the unit sphere scaled by its radius and moved to its center, so the wireframe is
# built once with radius 1 and only resized from there
ball = o3d.geometry.LineSet.create_from_triangle_mesh(o3d.geometry.TriangleMesh.create_sphere(radius=1.0, resolution=10))
unit_points = np.asarray(ball.points).copy()
ball.paint_uniform_color([0.0, 0.0, 1.0])

vis = o3d.visualization.Visualizer()
vis.create_window(window_name="RANSAC sphere fitting - live view")
vis.add_geometry(pcd)

# The unit sphere would ruin the framing of the scene, so the camera is fit to the cloud only
vis.add_geometry(ball, reset_bounding_box=False)

view_control = vis.get_view_control()

sphere = pyrsc.Sphere()


def plot_callback(state):
    colors = original_colors.copy()
    colors[state["best_inliers"]] = [0.0, 1.0, 0.0]
    colors[state["inliers"]] = [1.0, 0.0, 0.0]
    colors[state["sample_indices"]] = [0.1, 0.3, 0.9]
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # The sphere only changes when RANSAC finds a better candidate, so we redraw it just there
    if state["is_best"]:
        best = state["best_model"]
        ball.points = o3d.utility.Vector3dVector(unit_points * best["radius"] + np.asarray(best["center"]))
        vis.update_geometry(ball)

    # Turn the camera a bit on every frame so we can see the cloud in 3d while it is fitted. The
    # argument is a horizontal drag in pixels, the same as rotating the view with the mouse
    view_control.rotate(-5.0, 0.0)

    vis.update_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()
    time.sleep(0.05)

    return False  # Return True here (or from a custom condition) to stop fitting early.


center, radius, best_inliers = sphere.fit(points, thresh=0.2, maxIteration=200, callback=plot_callback)

print(f"Center: {center}")
print(f"Radius: {radius}")
print(f"Inliers: {len(best_inliers)} of {n_points}")

vis.run()
vis.destroy_window()
