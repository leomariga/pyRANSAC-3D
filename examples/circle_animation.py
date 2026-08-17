import time

import numpy as np
import open3d as o3d

import pyransac3d as pyrsc

N_SEGMENTS = 60


def circle_points(center, axis, radius):
    """Points which draw a circle of the given radius around a center, on the plane normal to axis"""
    axis = np.asarray(axis) / np.linalg.norm(axis)

    # Crossing the axis with the coordinate direction it is the least aligned with is what keeps
    # the result far from a zero vector
    helper = np.zeros(3)
    helper[np.argmin(np.abs(axis))] = 1.0
    first_axis = np.cross(axis, helper)
    first_axis = first_axis / np.linalg.norm(first_axis)
    second_axis = np.cross(axis, first_axis)

    angles = np.linspace(0, 2 * np.pi, N_SEGMENTS, endpoint=False)
    return center + radius * (np.cos(angles)[:, np.newaxis] * first_axis + np.sin(angles)[:, np.newaxis] * second_axis)


# Live-plot each RANSAC candidate while it is being evaluated: the current iteration's inliers are
# highlighted in red, the best inliers found so far in green, and the 3 sampled points that
# generated the current candidate in blue. The blue ring is the circle of the best candidate so
# far, and the camera turns a little on every iteration to show the cloud in 3d.
points = pyrsc.ShapeGenerator(seed=0).circle([0, 0, 0], [0, 1, 1], radius=3.0, n_points=800, noise=0.05, n_outliers=400)
n_points = points.shape[0]
original_colors = np.tile([0.6, 0.6, 0.6], (n_points, 1))

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
pcd.colors = o3d.utility.Vector3dVector(original_colors)

# The ring starts collapsed on the origin and is only drawn once a candidate is found. Its last
# point closes the ring by going back to the first one
ring = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(np.zeros((N_SEGMENTS, 3))),
    lines=o3d.utility.Vector2iVector([[i, (i + 1) % N_SEGMENTS] for i in range(N_SEGMENTS)]),
)
ring.paint_uniform_color([0.0, 0.0, 1.0])

vis = o3d.visualization.Visualizer()
vis.create_window(window_name="RANSAC circle fitting - live view")
vis.add_geometry(pcd)

# The empty ring would ruin the framing of the scene, so the camera is fit to the cloud only
vis.add_geometry(ring, reset_bounding_box=False)

view_control = vis.get_view_control()

circle = pyrsc.Circle()


def plot_callback(state):
    colors = original_colors.copy()
    colors[state["best_inliers"]] = [0.0, 1.0, 0.0]
    colors[state["inliers"]] = [1.0, 0.0, 0.0]
    colors[state["sample_indices"]] = [0.1, 0.3, 0.9]
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # The circle only changes when RANSAC finds a better candidate, so we redraw it just there
    if state["is_best"]:
        best = state["best_model"]
        ring.points = o3d.utility.Vector3dVector(circle_points(best["center"], best["axis"], best["radius"]))
        vis.update_geometry(ring)

    # Turn the camera a bit on every frame so we can see the cloud in 3d while it is fitted. The
    # argument is a horizontal drag in pixels, the same as rotating the view with the mouse
    view_control.rotate(-5.0, 0.0)

    vis.update_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()
    time.sleep(0.05)

    return False  # Return True here (or from a custom condition) to stop fitting early.


center, axis, radius, best_inliers = circle.fit(points, thresh=0.2, maxIteration=300, callback=plot_callback)

print(f"Center: {center}")
print(f"Axis: {axis}")
print(f"Radius: {radius}")
print(f"Inliers: {len(best_inliers)} of {n_points}")

vis.run()
vis.destroy_window()
