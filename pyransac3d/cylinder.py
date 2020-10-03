import numpy as np
import random
import copy 
from .aux import *

class Cylinder:
	""" 
	Implementation for cylinder RANSAC.

	This class finds a infinite height cilinder and returns the cylinder axis, center and radius. 
	This method uses 6 points to find 3 best plane equations orthogonal to each other. We could use a recursive planar RANSAC, but it would use 9 points instead, making this algorithm more efficient. 

	---
	"""

	def __init__(self):
		self.inliers = []
		self.center = []
		self.axis = []
		self.radius = 0

	def fit(self, pts, thresh=0.2, maxIteration=10000):
		""" 
        Find the parameters (axis and radius) to define a cylinder. 

        :param pts: 3D point cloud as a numpy array (N,3).
        :param thresh: Threshold distance from the cylinder hull which is considered inlier.
        :param maxIteration: Number of maximum iteration which RANSAC will loop over.

        :returns: 
        - `center`: Center of the cylinder np.array(1,3) which the cylinder axis is passing through.
		- `axis`: Vector describing cylinder's axis np.array(1,3).
        - `radius`: Radius of cylinder.
		- `inliers`: Inlier's index from the original point cloud.
        ---
        """

		n_points = pts.shape[0]
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
			vecA_norm = vecA / np.linalg.norm(vecA)
			vecB = pt_samples[2,:] - pt_samples[0,:]
			vecB_norm = vecB / np.linalg.norm(vecB)

			# Now we compute the cross product of vecA and vecB to get vecC which is normal to the plane
			vecC = np.cross(vecA_norm, vecB_norm)
			vecC = vecC / np.linalg.norm(vecC)

			# Now we calculate the rotation of the points with rodrigues equation
			P_rot = rodrigues_rot(pt_samples, vecC, [0,0,1])

			# Find center from 3 points
			# http://paulbourke.net/geometry/circlesphere/
			# Find lines that intersect the points
			# Slope:
			ma = 0
			mb = 0
			while(ma == 0):
				ma = (P_rot[1, 1]-P_rot[0, 1])/(P_rot[1, 0]-P_rot[0, 0])
				mb = (P_rot[2, 1]-P_rot[1, 1])/(P_rot[2, 0]-P_rot[1, 0])
				if(ma == 0):
					P_rot = np.roll(P_rot,-1,axis=0)
				else:
					break

			# Calulate the center by verifying intersection of each orthogonal line
			p_center_x = (ma*mb*(P_rot[0, 1]-P_rot[2, 1]) + mb*(P_rot[0, 0]+P_rot[1, 0]) - ma*(P_rot[1, 0]+P_rot[2, 0]))/(2*(mb-ma))
			p_center_y = -1/(ma)*(p_center_x - (P_rot[0, 0]+P_rot[1, 0])/2)+(P_rot[0, 1]+P_rot[1, 1])/2
			p_center = [p_center_x, p_center_y, 0]
			radius = np.linalg.norm(p_center - P_rot[0, :])

			# Remake rodrigues rotation
			center = rodrigues_rot(p_center, [0,0,1], vecC)[0]

			# Distance from a point to a line
			pt_id_inliers = [] # list of inliers ids
			vecC_stakado =  np.stack([vecC]*n_points,0)
			dist_pt = np.cross(vecC_stakado, (center- pts))
			dist_pt = np.linalg.norm(dist_pt, axis=1)


			# Select indexes where distance is biggers than the threshold
			pt_id_inliers = np.where(np.abs(dist_pt-radius) <= thresh)[0]

			if(len(pt_id_inliers) > len(best_inliers)):
				best_inliers = pt_id_inliers
				self.inliers = best_inliers
				self.center = center
				self.axis = vecC
				self.radius = radius

		return self.center, self.axis, self.radius,  self.inliers


