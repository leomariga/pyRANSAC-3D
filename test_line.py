import open3d as o3d
import numpy as np
import random
import copy 
import pyransac3d as pyrsc



print('create noisy mesh')
mesh_in = o3d.geometry.TriangleMesh.create_cylinder(radius=1, height=700.0)
vertices = np.asarray(mesh_in.vertices)
noise = 10
vertices += np.random.logistic(0,noise, size=vertices.shape)*2
mesh_in.vertices = o3d.utility.Vector3dVector(vertices)
mesh_in.compute_vertex_normals()
mesh_in.paint_uniform_color([0.2, 0.2, 0.8])
o3d.visualization.draw_geometries([mesh_in])
pcd_load=mesh_in.sample_points_uniformly(number_of_points=10000)
o3d.visualization.draw_geometries([pcd_load])



points = np.asarray(pcd_load.points)

line = pyrsc.Line()

A, B, inliers = line.fit(points,pcd_load, thresh=15)



R = pyrsc.get_rotationMatrix_from_vectors([0, 0, 1], A)

plane = pcd_load.select_by_index(inliers).paint_uniform_color([1, 0, 0])
# # obb = plane.get_oriented_bounding_box()
# # obb2 = plane.get_axis_aligned_bounding_box()
# # obb.color = [0, 0, 1]
# # obb2.color = [0, 1, 0]
# not_plane = pcd_load.select_by_index(inliers, invert=True)
# mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(origin=[0,0,0], size = 0.2)
# mesh_rot = copy.deepcopy(mesh).rotate(R, center=[0, 0, 0])

mesh_cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=1, height=1000)
mesh_cylinder.compute_vertex_normals()
mesh_cylinder.paint_uniform_color([1, 0, 0])
mesh_cylinder = mesh_cylinder.rotate(R, center=[0, 0, 0])
mesh_cylinder = mesh_cylinder.translate((B[0], B[1], B[2]))
o3d.visualization.draw_geometries([pcd_load, plane, mesh_cylinder])

# o3d.visualization.draw_geometries([plane, not_plane,  mesh,mesh_rot, mesh_cylinder])