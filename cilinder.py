import open3d as o3d
import numpy as np
import random
import copy 
from aux import *

class Cilinder:

	def Cilinder(self):
		self.inliers = []
		self.equation = []
		self.tMatrix = [] # env to plane
		self.rMatrix = [] # env to plane


	def find(self, pts, thresh=0.2, minPoints=50, maxIteration=5000):
		n_points = pts.shape[0]
		print(n_points)
		best_eq = []
		best_inliers = []

		for it in range(maxIteration):
			# Samples 3 random points 
			id_samples = random.sample(range(1, n_points-1), 3)
			#print(id_samples)
			#pt_samples = np.asarray([[0, 0, 0], [0.866*2, 0, 0], [0.866, 0, 1.5]])
			pt_samples = pts[id_samples]
			#print(pt_samples)

			# We have to find the plane equation described by those 3 points
			# We find first 2 vectors that are part of this plane
			# A = pt2 - pt1
			# B = pt3 - pt1

			vecA = pt_samples[1,:] - pt_samples[0,:]
			vecA_norm = vecA / np.linalg.norm(vecA)
			vecB = pt_samples[2,:] - pt_samples[0,:]
			vecB_norm = vecB / np.linalg.norm(vecB)
			#print(vecA)
			#print(vecB)

			# Now we compute the cross product of vecA and vecB to get vecC which is normal to the plane
			vecC = np.cross(vecA_norm, vecB_norm)
			vecC = vecC / np.linalg.norm(vecC)

			# Now we calculate the rotation of the points with rodrigues equation
			P_rot = rodrigues_rot(pt_samples, vecC, [0,0,1])
			#print("P_rot:")
			#print(P_rot)

			# Find center from 3 points
			# http://paulbourke.net/geometry/circlesphere/
			# Find lines that intersect the points
			# Slope:
			ma = 0
			mb = 0
			while(ma == 0):
				ma = (P_rot[1, 1]-P_rot[0, 1])/(P_rot[1, 0]-P_rot[0, 0])
				#print("ma: "+str(ma))
				mb = (P_rot[2, 1]-P_rot[1, 1])/(P_rot[2, 0]-P_rot[1, 0])
				#print("mb: "+str(mb))
				if(ma == 0):
					#print("ma zero, rolling order")
					P_rot = np.roll(P_rot,-1,axis=0)
				else:
					break
			# Calulate the center by verifying intersection of each orthogonal line
			p_center_x = (ma*mb*(P_rot[0, 1]-P_rot[2, 1]) + mb*(P_rot[0, 0]+P_rot[1, 0]) - ma*(P_rot[1, 0]+P_rot[2, 0]))/(2*(mb-ma))
			p_center_y = -1/(ma)*(p_center_x - (P_rot[0, 0]+P_rot[1, 0])/2)+(P_rot[0, 1]+P_rot[1, 1])/2
			p_center = [p_center_x, p_center_y, 0]
			radius = np.linalg.norm(p_center - P_rot[0, :])

			#print("Centro revolution:")
			#print("x: "+str(p_center_x)+"  y: "+str(p_center_y))
			#print("Radius: "+str(radius))
			#print("Distance to center:")
			#print(str(np.linalg.norm(p_center - P_rot[0, :]))+ " - "+str(np.linalg.norm(p_center - P_rot[1, :]))+ " - "+str(np.linalg.norm(p_center - P_rot[2, :]))+ " - ")

			# Remake rodrigues rotation
			center = rodrigues_rot(p_center, [0,0,1], vecC)[0]
			#print("New center: " + str(center))
			#print("Distance to center:")
			#print(str(np.linalg.norm(center - pt_samples[0, :]))+ " - "+str(np.linalg.norm(center - pt_samples[1, :]))+ " - "+str(np.linalg.norm(center - pt_samples[2, :]))+ " - ")


			# # Distance from a point to a plane 
		
			pt_id_inliers = [] # list of inliers ids
			vecC_stakado =  np.stack([vecC]*n_points,0)

			
			dist_pt = np.cross(vecC_stakado, (center- pts))
			dist_pt = np.linalg.norm(dist_pt, axis=1)
			#print(dist_pt)

			# Select indexes where distance is biggers than the threshold
			pt_id_inliers = np.where(np.abs(dist_pt-radius) <= thresh)[0]
			#print(len(pt_id_inliers))
			if(len(pt_id_inliers) > len(best_inliers)):
				best_inliers = pt_id_inliers
				# p1 = o3d.geometry.TriangleMesh.create_coordinate_frame(origin=pt_samples[0,:], size = 0.5)
				# p2 = o3d.geometry.TriangleMesh.create_coordinate_frame(origin=pt_samples[1,:], size = 0.5)
				# p3 = o3d.geometry.TriangleMesh.create_coordinate_frame(origin=pt_samples[2,:], size = 0.5)
				# pcd = o3d.geometry.PointCloud()
				# pcd.points = o3d.utility.Vector3dVector(pts)
				# o3d.visualization.draw_geometries([pcd.paint_uniform_color([0.1, 0.9, 0.1]), p1, p2, p3])
				# print(len(best_inliers))
				# print("Centro: "+str(center))
				# print("Raio: "+str(radius))
				self.inliers = best_inliers
				self.center = center
				self.normal = vecC
				self.radius = radius

		return self.center, self.normal, self.radius,  self.inliers

		








# Load saved point cloud and visualize it
pcd_load = o3d.io.read_point_cloud("caixa4.ply")
pcd_load = pcd_load.voxel_down_sample(voxel_size=0.01)
o3d.visualization.draw_geometries([pcd_load])


# mesh_cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=1, height=10.0)
# mesh_cylinder.compute_vertex_normals()
# mesh_cylinder.paint_uniform_color([0.1, 0.9, 0.1])
# o3d.visualization.draw_geometries([mesh_cylinder])
# pcd_load=mesh_cylinder.sample_points_uniformly(number_of_points=2000)
# o3d.visualization.draw_geometries([pcd_load])

points = np.asarray(pcd_load.points)

cil = Cilinder()

center, normal, radius,  inliers = cil.find(points, thresh=0.02)
print("center: "+str(center))
print("radius: "+str(radius))
print("vecC: "+str(normal))


R = get_rotationMatrix_from_vectors([0, 0, 1], normal)

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