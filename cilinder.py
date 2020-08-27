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


	def find(self, pts, thresh=0.05, minPoints=50, maxIteration=1):
		n_points = pts.shape[0]
		print(n_points)
		best_eq = []
		best_inliers = []

		for it in range(maxIteration):
			# Samples 3 random points 
			id_samples = random.sample(range(1, n_points-1), 3)
			#print(id_samples)
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

			# The plane equation will be vecC[0]*x + vecC[1]*y + vecC[0]*z = -k
			# We have to use a point to find k
			
			k = -np.sum(np.multiply(vecC, pt_samples[1,:]))
			plane_eq = [vecC[0], vecC[1], vecC[2], k]
			
			# Now we have to find the radius and the center

			# The center should be a point in which the distance is equal for every selected point point
			# abs(p1- c) = abs(p2- c) = abs(p3- c) = R

			# We calculate the 2d projection
			# https://math.stackexchange.com/questions/1076177/3d-coordinates-of-circle-center-given-three-point-on-the-circle
			# u = vecA, v = vecB, w = vecC
			# We take the 2D coordinated by using dot product
			# One of the poins is the center. The other we align with x axis like this figure https://math.stackexchange.com/questions/1907749/how-to-find-the-center-of-a-circle-and-its-radius-in-a-3d-space-given-3-points
			a = np.asarray([0, 0])
			b = np.asarray([np.dot(vecA, vecA_norm), 0])
			c = np.asarray([np.dot(vecB, vecA_norm), np.dot(vecB, vecB_norm)])

			# The center in x will aways be the middle way between a and b
			# The center in y can be caculated making the distance from c, iguals the distance from the orign
			h = (((c[0]-b[0]/2)**2 + c[1]**2 - (b[0]/2)**2))/(2*c[1])
			#print("Centro projetado: "+str(h) )
			# The projected center is:
			p_center = np.asarray([b[0]/2, h])
			rad1 = np.linalg.norm(p_center - a)
			rad2 = np.linalg.norm(p_center - b)
			rad3 = np.linalg.norm(p_center - c)
			print(str(rad1)+ " - " +str(rad2)+ " - "+ str(rad3))

			# The centered in the 3D space will be:
			center = pt_samples[0,:] + np.dot(vecA_norm, p_center[0]) + np.dot(vecB_norm, p_center[1])

			# We can calculate the radius
			rad1 = np.linalg.norm(center-pt_samples[0,:])
			rad2 = np.linalg.norm(center-pt_samples[1,:])
			rad3 = np.linalg.norm(center-pt_samples[2,:])
			print(str(rad1)+ " - " +str(rad2)+ " - "+ str(rad3))


			# # Distance from a point to a plane 
			# # https://mathworld.wolfram.com/Point-PlaneDistance.html
			pt_id_inliers = [] # list of inliers ids
			vecC_stakado =  np.stack([vecC]*n_points,0)

			
			dist_pt = np.cross(vecC_stakado, (pts-center))/np.linalg.norm(vecC)
			dist_pt = np.sqrt(np.einsum('ij,ij->i', dist_pt, dist_pt))
			#dist_pt = (plane_eq[0]*pts[:,0]+plane_eq[1]*pts[:, 1]+plane_eq[2]*pts[:, 2]+plane_eq[3])/np.sqrt(plane_eq[0]**2+plane_eq[1]**2+plane_eq[2]**2)
			#print(dist_pt)

			# Select indexes where distance is biggers than the threshold
			pt_id_inliers = np.where(np.abs(dist_pt-h) <= thresh)[0]
			if(len(pt_id_inliers) > len(best_inliers)):
				best_eq = plane_eq
				best_inliers = pt_id_inliers
			self.inliers = best_inliers
			self.center = center
			self.normal = vecC
			self.radius = h

		return center, p_center, vecC, h,  best_inliers

		








# Load saved point cloud and visualize it
#pcd_load = o3d.io.read_point_cloud("caixa4.ply")
#o3d.visualization.draw_geometries([pcd_load])


mesh_cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=0.3,
                                                            height=4.0)
mesh_cylinder.compute_vertex_normals()
mesh_cylinder.paint_uniform_color([0.1, 0.9, 0.1])
o3d.visualization.draw_geometries([mesh_cylinder])
pcd_load=mesh_cylinder.sample_points_uniformly(number_of_points=2000)
o3d.visualization.draw_geometries([pcd_load])

points = np.asarray(pcd_load.points)

cil = Cilinder()

center, p_center, vecC, h,  best_inliers = cil.find(points, 0.1)
print("center: "+str(center))
print("p_center: "+str(p_center))
print("vecC: "+str(vecC))
print("h: "+str(h))

R = get_rotationMatrix_from_vectors([0, 0, 1], vecC)

plane = pcd_load.select_by_index(best_inliers).paint_uniform_color([1, 0, 0])
# obb = plane.get_oriented_bounding_box()
# obb2 = plane.get_axis_aligned_bounding_box()
# obb.color = [0, 0, 1]
# obb2.color = [0, 1, 0]
not_plane = pcd_load.select_by_index(best_inliers, invert=True)
mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(origin=[0, 0, 0])
mesh_rot = copy.deepcopy(mesh).rotate(R, center=[0, 0, 0])

o3d.visualization.draw_geometries([not_plane, plane, mesh, mesh_rot])