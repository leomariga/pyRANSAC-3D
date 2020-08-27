import open3d as o3d
import numpy as np
import random
import copy 
from aux import *

class Plane:

	def Plane(self):
		self.inliers = []
		self.equation = []
		self.tMatrix = [] # env to plane
		self.rMatrix = [] # env to plane


	def findPlane(self, pts, thresh=0.05, minPoints=100, maxIteration=1000):
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
			vecB = pt_samples[2,:] - pt_samples[0,:]

			#print(vecA)
			#print(vecB)

			# Now we compute the cross product of vecA and vecB to get vecC which is normal to the plane
			vecC = np.cross(vecA, vecB)
			

			# The plane equation will be vecC[0]*x + vecC[1]*y + vecC[0]*z = -k
			# We have to use a point to find k
			vecC = vecC / np.linalg.norm(vecC)
			k = -np.sum(np.multiply(vecC, pt_samples[1,:]))
			plane_eq = [vecC[0], vecC[1], vecC[2], k]
			
			#print(plane_eq)

			# Distance from a point to a plane 
			# https://mathworld.wolfram.com/Point-PlaneDistance.html
			pt_id_inliers = [] # list of inliers ids
			dist_pt = (plane_eq[0]*pts[:,0]+plane_eq[1]*pts[:, 1]+plane_eq[2]*pts[:, 2]+plane_eq[3])/np.sqrt(plane_eq[0]**2+plane_eq[1]**2+plane_eq[2]**2)
			
			# Select indexes where distance is biggers than the threshold
			pt_id_inliers = np.where(np.abs(dist_pt) <= thresh)[0]
			if(len(pt_id_inliers) > len(best_inliers)):
				best_eq = plane_eq
				best_inliers = pt_id_inliers
			self.inliers = pts[best_inliers]
			self.equation = best_eq
		return best_eq, best_inliers

		








# Load saved point cloud and visualize it
pcd_load = o3d.io.read_point_cloud("caixa.ply")
#o3d.visualization.draw_geometries([pcd_load])
points = np.asarray(pcd_load.points)

plano1 = Plane()

best_eq, best_inliers = plano1.findPlane(points, 0.01)
plane = pcd_load.select_by_index(best_inliers).paint_uniform_color([1, 0, 0])
obb = plane.get_oriented_bounding_box()
obb2 = plane.get_axis_aligned_bounding_box()
obb.color = [0, 0, 1]
obb2.color = [0, 1, 0]
not_plane = pcd_load.select_by_index(best_inliers, invert=True)
#mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(origin=[0, 0, 0])

o3d.visualization.draw_geometries([not_plane, plane, obb, obb2])