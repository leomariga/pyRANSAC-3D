import numpy as np
import random

class Cuboid:
	""" 
	Implementation for box (Cuboid) RANSAC.

	A cuboid is defined as convex polyhedron bounded by six faces formed by three orthogonal normal vectors. Cats love to play with this kind of geometry.
	This method uses 6 points to find 3 best plane equations orthogonal to eachother. 

	We could use a recursive planar RANSAC, but it would use 9 points instead. Orthogonality makes this algorithm more efficient. 

	![Cuboid](https://raw.githubusercontent.com/leomariga/pyRANSAC-3D/master/doc/cuboid.gif "Cuboid")

	---
	"""
	def __init__(self):
		self.inliers = []
		self.equation = []




	def fit(self, pts, thresh=0.05, maxIteration=5000):
		""" 
		Find the best equation for 3 planes which define a complete cuboid.

		:param pts: 3D point cloud as a `np.array (N,3)`.
		:param thresh: Threshold distance from the cylinder radius which is considered inlier.
		:param maxIteration: Number of maximum iteration which RANSAC will loop over.
		:returns:
		- `center`:  Point in space in which the cylinder axis will pass through. `np.array (1, 3)`
		- `axis`: Unitary vector in the direction of cylinder axis `np.array (1, 3)`
		- `radius`: Radius of the cylinder
		- `inliers`: Inlier's index from the original point cloud. `np.array (1, M)`
		---
		"""
		n_points = pts.shape[0]
		print(n_points)
		best_eq = []
		best_inliers = []

		for it in range(maxIteration):
			plane_eq = []

			# Samples 3 random points 
			id_samples = random.sample(range(1, n_points-1), 6)
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
			vecC = vecC / np.linalg.norm(vecC) # Normal

			k = -np.sum(np.multiply(vecC, pt_samples[1,:]))
			plane_eq.append([vecC[0], vecC[1], vecC[2], k])

			# Now we use another point to find a orthogonal plane 2
			# Calculate distance from the point to the first plane
			dist_p4_plane = (plane_eq[0][0]*pt_samples[3,0]+plane_eq[0][1]*pt_samples[3,1]+plane_eq[0][2]*pt_samples[3,2]+plane_eq[0][3])/np.sqrt(plane_eq[0][0]**2+plane_eq[0][1]**2+plane_eq[0][2]**2)
			
			# vecC is already normal (module 1) so we only have to discount from the point, the distance*unity = distance*normal
			# A simple way of understanding this is we move our point along the normal until it reaches the plane
			p4_proj_plane = pt_samples[3,0]-dist_p4_plane*vecC

			# Now, with help of our point p5 we can find another plane P2 which contains p4, p4_proj, p5 and 
			vecD = p4_proj_plane - pt_samples[3,:]
			vecE = pt_samples[4,:] - pt_samples[3,:]
			vecF = np.cross(vecD, vecE)
			vecF = vecF / np.linalg.norm(vecF) # Normal
			k = -np.sum(np.multiply(vecF, pt_samples[4,:]))
			plane_eq.append([vecF[0], vecF[1], vecF[2], k])

			# The last plane will be orthogonal to the first and sacond plane (and its normals will be orthogonal to first and second planes' normal)
			vecG = np.cross(vecC, vecF)

			k = -np.sum(np.multiply(vecG, pt_samples[5,:]))
			plane_eq.append([vecG[0], vecG[1], vecG[2], k])
			plane_eq = np.asarray(plane_eq)
			# We have to find the value D for the last plane.

			# Distance from a point to a plane 
			# https://mathworld.wolfram.com/Point-PlaneDistance.html
			pt_id_inliers = [] # list of inliers ids
			dist_pt = []
			for id_plane in range(plane_eq.shape[0]):
				dist_pt.append(np.abs((plane_eq[id_plane,0]*pts[:,0]+plane_eq[id_plane,1]*pts[:, 1]+plane_eq[id_plane,2]*pts[:, 2]+plane_eq[id_plane,3])/np.sqrt(plane_eq[id_plane,0]**2+plane_eq[id_plane,1]**2+plane_eq[id_plane,2]**2)))
			
			# Select indexes where distance is biggers than the threshold
			dist_pt = np.asarray(dist_pt)
			min_dist_pt = np.amin(dist_pt, axis=0)
			pt_id_inliers = np.where(np.abs(min_dist_pt) <= thresh)[0]


			if(len(pt_id_inliers) > len(best_inliers)):
				best_eq = plane_eq
				best_inliers = pt_id_inliers
			self.inliers = best_inliers
			self.equation = best_eq
		return best_eq, best_inliers
