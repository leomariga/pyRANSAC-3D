from pathlib import Path

import numpy as np
import open3d as o3d

import pyransac3d as pyrsc

DATASET_DIR = Path(__file__).resolve().parent / "dataset"

# Load saved point cloud and visualize it
pcd_load = o3d.io.read_point_cloud(str(DATASET_DIR / "caixa.ply"))
# o3d.visualization.draw_geometries([pcd_load])
points = np.asarray(pcd_load.points)

plano1 = pyrsc.Cuboid()

center, extents, axes, best_inliers = plano1.fit(points, 0.02)
plane = pcd_load.select_by_index(best_inliers).paint_uniform_color([1, 0, 0])
not_plane = pcd_load.select_by_index(best_inliers, invert=True)

print("Center: ", center)
print("Extents: ", extents)
print("Axes: ", axes)

# The equations of the 3 faces are not returned by fit, but they are kept in the object
print("Plane equations: ", plano1.equation)

# Rotation and translation of the box, which is what we need to place a mesh on it
transform = plano1.get_transform()
print("Rotation: ", transform[0:3, 0:3])
print("Translation: ", transform[0:3, 3])

# create_box builds the box on the origin corner, so we move it half of its size on every
# direction to center it, and then a single transform puts it on the pose we just fitted
mesh_box = o3d.geometry.TriangleMesh.create_box(width=extents[0], height=extents[1], depth=extents[2])
mesh_box = mesh_box.translate(-extents / 2)
mesh_box = mesh_box.transform(transform)
mesh_box.compute_vertex_normals()
mesh_box.paint_uniform_color([0, 0, 1])

# draw_geometries has no transparency at all, it always draws a mesh opaque, and a solid box would
# hide the points it rests on. Drawing the 12 edges of the same box is what lets us see them.
# The 4 first corners are on one face and the 4 last ones are on the opposite face, in the same order
edges = [[0, 1], [1, 2], [2, 3], [3, 0], [4, 5], [5, 6], [6, 7], [7, 4], [0, 4], [1, 5], [2, 6], [3, 7]]
box = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(plano1.get_corners()),
    lines=o3d.utility.Vector2iVector(edges),
)
box.paint_uniform_color([0, 0, 1])

# mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(origin=[0, 0, 0])
# Replace box with mesh_box below to draw it solid instead of see-through
o3d.visualization.draw_geometries([plane, not_plane, box])
