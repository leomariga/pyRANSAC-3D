import open3d as o3d
import numpy as np
import pyransac3d as pyrsc

mesh_in = o3d.geometry.TriangleMesh.create_sphere(radius=5.0)
vertices = np.asarray(mesh_in.vertices)
noise = 0.2
vertices += np.random.logistic(0,noise, size=vertices.shape)
mesh_in.vertices = o3d.utility.Vector3dVector(vertices)
mesh_in.compute_vertex_normals()
mesh_in.paint_uniform_color([0.1, 0.1, 0.9])
o3d.visualization.draw_geometries([mesh_in])
pcd_load=mesh_in.sample_points_uniformly(number_of_points=2000)
o3d.visualization.draw_geometries([pcd_load])

points = np.asarray(pcd_load.points)

sph = pyrsc.Sphere()

center, radius, inliers = sph.fit(points,pcd_load, thresh=0.4)
print("center: "+str(center))
print("radius: "+str(radius))


inline = pcd_load.select_by_index(inliers).paint_uniform_color([1, 0, 0])
outline = pcd_load.select_by_index(inliers, invert=True).paint_uniform_color([0, 1, 0])

mesh_circle = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
mesh_circle.compute_vertex_normals()
mesh_circle.paint_uniform_color([0.9, 0.1, 0.1])
mesh_circle = mesh_circle.translate((center[0], center[1], center[2]))
o3d.visualization.draw_geometries([outline,mesh_circle,  inline])