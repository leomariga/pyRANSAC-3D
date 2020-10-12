import open3d as o3d
import numpy as np
import random
import copy 
import pyransac3d as pyrsc


# Load saved point cloud and visualize it
color_raw = o3d.io.read_image("dataset/1_rgb.jpg")
depth_raw = o3d.io.read_image("dataset/1_depth.png")
pcd_load = pyrsc.open_pointCloud_from_rgb_and_depth(color_raw, depth_raw, meters_trunc=100, showImages = False)
#pcd_load = o3d.io.read_point_cloud("dataset/caixa.ply")
#o3d.visualization.draw_geometries([pcd_load])
points = np.asarray(pcd_load.points)

plano1 = pyrsc.Plane()

best_eq, best_inliers = plano1.fit(points,pcd_load, 0.02)
plane = pcd_load.select_by_index(best_inliers).paint_uniform_color([1, 0, 0])
obb = plane.get_oriented_bounding_box()
obb2 = plane.get_axis_aligned_bounding_box()
obb.color = [0, 0, 1]
obb2.color = [0, 1, 0]
not_plane = pcd_load.select_by_index(best_inliers, invert=True)

o3d.visualization.draw_geometries([not_plane, plane, obb, obb2])