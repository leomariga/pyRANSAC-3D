import numpy as np
import open3d as o3d
from numpy.typing import ArrayLike, NDArray

import pyransac3d as pyrsc

N_SEGMENTS = 40
N_RINGS = 6


def cone_wireframe(
    apex: ArrayLike, axis: ArrayLike, angle: float, height: float
) -> tuple[NDArray[np.float64], list[list[int]]]:
    """Points and edges which draw a cone of the given half-angle, growing from its apex"""

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
    distances = np.linspace(0, height, N_RINGS)
    points = np.vstack([apex + axis * d + ring * (d * np.tan(angle)) for d in distances])

    # Each ring is closed on itself, and then a few lines run along the surface to join them all,
    # which is what gives the wireframe its 3d look
    edges = []
    for r in range(N_RINGS):
        base = r * N_SEGMENTS
        edges += [[base + i, base + (i + 1) % N_SEGMENTS] for i in range(N_SEGMENTS)]
    for i in range(0, N_SEGMENTS, 5):
        edges += [[r * N_SEGMENTS + i, (r + 1) * N_SEGMENTS + i] for i in range(N_RINGS - 1)]

    return points, edges


mesh_cone = o3d.geometry.TriangleMesh.create_cone(radius=2.0, height=6.0)
mesh_cone.compute_vertex_normals()
mesh_cone.paint_uniform_color([0.1, 0.9, 0.1])
o3d.visualization.draw_geometries([mesh_cone])

pcd_load = mesh_cone.sample_points_uniformly(number_of_points=3000)
o3d.visualization.draw_geometries([pcd_load])

points = np.asarray(pcd_load.points)

# A cone is found from the normals of the surface, not from the points alone. Open3D already
# knows how to estimate them, so we hand them over instead of paying for the search twice. Their
# orientation does not matter to the fit, only their direction
pcd_load.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=20))
normals = np.asarray(pcd_load.normals)

cone = pyrsc.Cone()
apex, axis, angle, inliers = cone.fit(points, thresh=0.05, maxIteration=1000, normals=normals)

print("apex: " + str(apex))
print("axis: " + str(axis))
print("angle (deg): " + str(np.rad2deg(angle)))
print(f"inliers: {len(inliers)} of {len(points)}")

# The fit describes a cone of infinite height, so we measure how far the inliers reach along the
# axis to draw only the piece of it which matches the cloud
reach = float(((points[inliers] - apex) @ np.asarray(axis)).max())
wireframe_points, wireframe_edges = cone_wireframe(apex, axis, angle, reach)

surface = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(wireframe_points),
    lines=o3d.utility.Vector2iVector(wireframe_edges),
)
surface.paint_uniform_color([0.0, 0.0, 1.0])

on_cone = pcd_load.select_by_index(inliers).paint_uniform_color([1, 0, 0])
off_cone = pcd_load.select_by_index(inliers, invert=True)
origin = o3d.geometry.TriangleMesh.create_coordinate_frame(origin=[0, 0, 0], size=0.5)
at_apex = o3d.geometry.TriangleMesh.create_coordinate_frame(origin=apex, size=1.0)

o3d.visualization.draw_geometries([on_cone, off_cone, origin, at_apex, surface])
