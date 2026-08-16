import sys
import time

import numpy as np
import open3d as o3d

sys.path.append(".")
import pyransac3d as pyrsc

# Live-plot the cuboid while it is being fitted: the box keeps its real point colors, the current
# iteration's inliers are highlighted in red, the best inliers found so far in green, and the 6
# sampled points that generated the current candidate in blue. The wireframe is the box of the best
# candidate so far, and the camera turns a little on every iteration to show its 3d shape.
pcd_load = o3d.io.read_point_cloud("tests/dataset/caixa.ply")
points = np.asarray(pcd_load.points)
n_points = points.shape[0]
original_colors = np.asarray(pcd_load.colors)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
pcd.colors = o3d.utility.Vector3dVector(original_colors)

# The box starts collapsed on the origin and is only moved once the first candidate is measured.
# The 4 first corners are on one face and the 4 last ones are on the opposite face, in the same order
edges = [[0, 1], [1, 2], [2, 3], [3, 0], [4, 5], [5, 6], [6, 7], [7, 4], [0, 4], [1, 5], [2, 6], [3, 7]]
box = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(np.zeros((8, 3))),
    lines=o3d.utility.Vector2iVector(edges),
)
box.paint_uniform_color([0.0, 0.0, 1.0])

vis = o3d.visualization.Visualizer()
vis.create_window(window_name="RANSAC cuboid fitting - live view")
vis.add_geometry(pcd)

# The empty box would ruin the framing of the scene, so the camera is fit to the point cloud only
vis.add_geometry(box, reset_bounding_box=False)

view_control = vis.get_view_control()

cuboid1 = pyrsc.Cuboid()


def plot_callback(state):
    colors = original_colors.copy()
    colors[state["best_inliers"]] = [0.0, 1.0, 0.0]
    colors[state["inliers"]] = [1.0, 0.0, 0.0]
    colors[state["sample_indices"]] = [0.1, 0.3, 0.9]
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # The box only changes when RANSAC finds a better set of 3 planes, so we measure it just there
    if state["is_best"]:
        cuboid1.measure_box(points[state["best_inliers"]], state["best_model"]["equation"])
        box.points = o3d.utility.Vector3dVector(cuboid1.get_corners())
        vis.update_geometry(box)

    # Turn the camera a bit on every frame so we can see the box in 3d while it is fitted. The
    # argument is a horizontal drag in pixels, the same as rotating the view with the mouse
    view_control.rotate(-5.0, 0.0)

    vis.update_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()
    time.sleep(0.07)

    return False  # Return True here (or from a custom condition) to stop fitting early.


center, extents, axes, best_inliers = cuboid1.fit(points, thresh=0.02, maxIteration=1000, callback=plot_callback)

print(f"Center: {center}")
print(f"Extents: {extents}")
print(f"Axes: {axes}")
print(f"Inliers: {len(best_inliers)} of {n_points}")

vis.run()
vis.destroy_window()
