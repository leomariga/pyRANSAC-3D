import time
from typing import Any

import numpy as np
import open3d as o3d
from numpy.typing import ArrayLike, NDArray

import pyransac3d as pyrsc

N_SEGMENTS = 40
N_RINGS = 6
HEIGHT = 6.0


def cone_points(apex: ArrayLike, axis: ArrayLike, angle: float) -> NDArray[np.float64]:
    """Points which draw the rings of a cone of the given half-angle, growing from its apex"""
    apex = np.asarray(apex, dtype=float)
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)

    # Crossing the axis with the coordinate direction it is the least aligned with is what keeps
    # the result far from a zero vector
    helper = np.zeros(3)
    helper[np.argmin(np.abs(axis))] = 1.0
    first_axis = np.cross(axis, helper)
    first_axis = first_axis / np.linalg.norm(first_axis)
    second_axis = np.cross(axis, first_axis)

    angles = np.linspace(0, 2 * np.pi, N_SEGMENTS, endpoint=False)
    ring = np.cos(angles)[:, np.newaxis] * first_axis + np.sin(angles)[:, np.newaxis] * second_axis

    # The cone found by the fit is infinite, so we draw the piece of it which is as tall as the
    # cloud, as a few rings which grow while they walk away from the apex
    distances = np.linspace(0, HEIGHT, N_RINGS)
    return np.vstack([apex + axis * d + ring * (d * np.tan(angle)) for d in distances])


# Each ring is closed on itself, and then a few lines run along the surface to join them all,
# which is what gives the wireframe its 3d look
edges = []
for r in range(N_RINGS):
    base = r * N_SEGMENTS
    edges += [[base + i, base + (i + 1) % N_SEGMENTS] for i in range(N_SEGMENTS)]
for i in range(0, N_SEGMENTS, 5):
    edges += [[r * N_SEGMENTS + i, (r + 1) * N_SEGMENTS + i] for i in range(N_RINGS - 1)]

# Live-plot each RANSAC candidate while it is being evaluated: the current iteration's inliers are
# highlighted in red, the best inliers found so far in green, and the 3 sampled points that
# generated the current candidate in blue. The blue wireframe is the cone of the best candidate so
# far, and the camera turns a little on every iteration to show the cloud in 3d.
points = pyrsc.ShapeGenerator(seed=0).cone(
    [0, 0, 0], [0, 0, 1], angle=np.deg2rad(25.0), height=HEIGHT, n_points=1000, noise=0.03, n_outliers=300
)
n_points = points.shape[0]
original_colors = np.tile([0.6, 0.6, 0.6], (n_points, 1))

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
pcd.colors = o3d.utility.Vector3dVector(original_colors)

# The wireframe starts collapsed on the origin and is only drawn once a candidate is found
surface = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(np.zeros((N_RINGS * N_SEGMENTS, 3))),
    lines=o3d.utility.Vector2iVector(edges),
)
surface.paint_uniform_color([0.0, 0.0, 1.0])

vis = o3d.visualization.Visualizer()
vis.create_window(window_name="RANSAC cone fitting - live view")
vis.add_geometry(pcd)

# The empty wireframe would ruin the framing of the scene, so the camera is fit to the cloud only
vis.add_geometry(surface, reset_bounding_box=False)

view_control = vis.get_view_control()

cone = pyrsc.Cone()


def plot_callback(state: dict[str, Any]) -> bool:
    colors = original_colors.copy()
    colors[state["best_inliers"]] = [0.0, 1.0, 0.0]
    colors[state["inliers"]] = [1.0, 0.0, 0.0]
    colors[state["sample_indices"]] = [0.1, 0.3, 0.9]
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # The cone only changes when RANSAC finds a better candidate, so we redraw it just there
    if state["is_best"]:
        best = state["best_model"]
        surface.points = o3d.utility.Vector3dVector(cone_points(best["apex"], best["axis"], best["angle"]))
        vis.update_geometry(surface)

    # Turn the camera a bit on every frame so we can see the cloud in 3d while it is fitted. The
    # argument is a horizontal drag in pixels, the same as rotating the view with the mouse
    view_control.rotate(-5.0, 0.0)

    vis.update_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()
    time.sleep(0.05)

    return False  # Return True here (or from a custom condition) to stop fitting early.


apex, axis, angle, best_inliers = cone.fit(points, thresh=0.15, maxIteration=300, callback=plot_callback)

print(f"Apex: {apex}")
print(f"Axis: {axis}")
print(f"Angle (deg): {np.rad2deg(angle)}")
print(f"Inliers: {len(best_inliers)} of {n_points}")

vis.run()
vis.destroy_window()
