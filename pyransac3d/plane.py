import open3d as o3d
import numpy as np
import random
import copy 

class Plane:
	""" 
	Implementation of planar RANSAC.

	Class for Plane object, which finds the equation of a infinite plane using RANSAC algorithim. 

	Call `fit(.)` to randomly take 3 points of pointcloud to verify inliers based on a threshold.

	![Plane](https://raw.githubusercontent.com/leomariga/pyRANSAC-3D/master/doc/plano.gif "Plane")

	---
	"""

	def __init__(self):
		self.inliers = []
		self.equation = []



	def fit(self, pts, thresh=0.05, minPoints=100, maxIteration=1000):
		""" 
		Find the best equation for a plane.

		:param pts: 3D point cloud as a `np.array (N,3)`.
		:param thresh: Threshold distance from the plane which is considered inlier.
		:param maxIteration: Number of maximum iteration which RANSAC will loop over.
		:returns:
		- `self.equation`:  Parameters of the plane using Ax+By+Cy+D `np.array (1, 4)`
		- `self.inliers`: points from the dataset considered inliers

		---
		"""
		n_points = pts.shape[0]
		print(n_points)
		best_eq = []
		best_inliers = []

		for it in range(maxIteration):

			# Samples 3 random points 
			id_samples = random.sample(range(1, n_points-1), 3)
			pt_samples = pts[id_samples]

			# We have to find the plane equation described by those 3 points
			# We find first 2 vectors that are part of this plane
			# A = pt2 - pt1
			# B = pt3 - pt1

			vecA = pt_samples[1,:] - pt_samples[0,:]
			vecB = pt_samples[2,:] - pt_samples[0,:]

			# Now we compute the cross product of vecA and vecB to get vecC which is normal to the plane
			vecC = np.cross(vecA, vecB)
			

			# The plane equation will be vecC[0]*x + vecC[1]*y + vecC[0]*z = -k
			# We have to use a point to find k
			vecC = vecC / np.linalg.norm(vecC)
			k = -np.sum(np.multiply(vecC, pt_samples[1,:]))
			plane_eq = [vecC[0], vecC[1], vecC[2], k]

			# Distance from a point to a plane 
			# https://mathworld.wolfram.com/Point-PlaneDistance.html
			pt_id_inliers = [] # list of inliers ids
			dist_pt = (plane_eq[0]*pts[:,0]+plane_eq[1]*pts[:, 1]+plane_eq[2]*pts[:, 2]+plane_eq[3])/np.sqrt(plane_eq[0]**2+plane_eq[1]**2+plane_eq[2]**2)
			
			# Select indexes where distance is biggers than the threshold
			pt_id_inliers = np.where(np.abs(dist_pt) <= thresh)[0]
			if(len(pt_id_inliers) > len(best_inliers)):
				best_eq = plane_eq
				best_inliers = pt_id_inliers
			self.inliers = best_inliers
			self.equation = best_eq

		return self.equation, self.inliers