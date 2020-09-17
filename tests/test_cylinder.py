import open3d as o3d
import numpy as np
import random
import copy 
import pyransac3d as pyrsc

mesh_cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=1, height=10.0)
mesh_cylinder.compute_vertex_normals()
mesh_cylinder.paint_uniform_color([0.1, 0.9, 0.1])
o3d.visualization.draw_geometries([mesh_cylinder])
pcd_load=mesh_cylinder.sample_points_uniformly(number_of_points=2000)
o3d.visualization.draw_geometries([pcd_load])

points = np.asarray(pcd_load.points)

cil = pyrsc.Cylinder()

center, normal, radius,  inliers = cil.fit(points, thresh=0.05)
print("center: "+str(center))
print("radius: "+str(radius))
print("vecC: "+str(normal))


R = pyrsc.get_rotationMatrix_from_vectors([0, 0, 1], normal)

plane = pcd_load.select_by_index(inliers).paint_uniform_color([1, 0, 0])
# obb = plane.get_oriented_bounding_box()
# obb2 = plane.get_axis_aligned_bounding_box()
# obb.color = [0, 0, 1]
# obb2.color = [0, 1, 0]
not_plane = pcd_load.select_by_index(inliers, invert=True)
mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(origin=[0,0,0], size = 0.2)
cen = o3d.geometry.TriangleMesh.create_coordinate_frame(origin=center, size = 0.5)
mesh_rot = copy.deepcopy(mesh).rotate(R, center=[0, 0, 0])

mesh_cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=radius,
                                                            height=0.5)
mesh_cylinder.compute_vertex_normals()
mesh_cylinder.paint_uniform_color([0.1, 0.9, 0.1])
mesh_cylinder = mesh_cylinder.rotate(R, center=[0, 0, 0])
mesh_cylinder = mesh_cylinder.translate((center[0], center[1], center[2]))
o3d.visualization.draw_geometries([mesh_cylinder])

o3d.visualization.draw_geometries([plane, not_plane,  mesh,mesh_rot, mesh_cylinder])