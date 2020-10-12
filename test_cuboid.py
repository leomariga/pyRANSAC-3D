import open3d as o3d
import numpy as np
import random
import matplotlib
import matplotlib.pyplot as plt
import pyransac3d as pyrsc


# Load saved point cloud and visualize it
pcd_load = o3d.io.read_point_cloud("dataset/caixa.ply")
#o3d.visualization.draw_geometries([pcd_load])
points = np.asarray(pcd_load.points)

plano1 = pyrsc.Cuboid()

best_eq, best_inliers = plano1.fit(points,pcd_load,thresh= 0.02)
plane = pcd_load.select_by_index(best_inliers).paint_uniform_color([1, 0, 0])
not_plane = pcd_load.select_by_index(best_inliers, invert=True)
# mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(origin=[0, 0, 0])
o3d.visualization.draw_geometries([plane, not_plane])