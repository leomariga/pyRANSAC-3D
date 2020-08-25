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

		self.limits_f_plane = []
		self.size = []


	def findEnviromentParameters(self):
		t, r = self.rawAlignToXY(self.equation)
		self.rMatrix = r
		self.tMatrix = t
		translated_inliers = copy.deepcopy(self.inliers)-t
		rotated_inliers = np.matmul(r,copy.deepcopy(translated_inliers).transpose()).T

		xmin, ymin, zmin = np.amin(rotated_inliers,0)
		xmax, ymax, zmax = np.amax(rotated_inliers,0)

		# Defining bounded plane with max and min limits from cloud point

		#lim_plane.append([0, 0, 0])


		lim_plane = []
		lim_plane.append([xmin, ymin, 0])
		lim_plane.append([xmin, ymax, 0])
		lim_plane.append([xmax, ymin, 0])
		lim_plane.append([xmax, ymax, 0])
		lim_plane = np.asarray(lim_plane)
		self.limits_f_plane = lim_plane

		self.size = np.asarray([abs(xmax-xmin), abs(ymax-ymin)])
		print(lim_plane)

		return rotated_inliers

	def rawAlignToXY(self, eq):
		a = eq[0]
		b = eq[1]
		c = eq[2]
		d = eq[3]
		print(np.sqrt(a*a+b*b+c*c))
		c_theta = c/(np.sqrt(a*a+b*b+c*c))
		s_theta = np.sqrt((a*a+b*b)/(a*a+b*b+c*c))
		u1 = b/(np.sqrt(a*a+b*b+c*c))
		u2 = - a/(np.sqrt(a*a+b*b+c*c))
		t_matrix = np.asarray([0, 0, -d/c])
		r_matrix = np.asarray([[c_theta+u1*u1*(1-c_theta), u1*u2*(1-c_theta), u2*s_theta],
								[u1*u2*(1-c_theta), c_theta+u2*u2*(1-c_theta), -u1*s_theta],
								[-u2*s_theta, u1*s_theta, c_theta]])
		return t_matrix, r_matrix



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
plane = pcd_load.select_by_index(best_inliers)#.paint_uniform_color([1, 0, 0])
obb = plane.get_oriented_bounding_box()
obb.color = [0, 0, 1]
not_plane = pcd_load.select_by_index(best_inliers, invert=True)
o3d.visualization.draw_geometries([not_plane, plane, obb])
#pcd = o3d.geometry.PointCloud()
#pcd.points = o3d.utility.Vector3dVector(plano1.findEnviromentParameters())
t, r = plano1.rawAlignToXY(best_eq)

pcd_rotacionado = copy.deepcopy(pcd_load).translate(-t).rotate(r, center=(0,0,0))#.paint_uniform_color([0, 0, 1])
mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()


#box = drawPlane(plano1)

o3d.visualization.draw_geometries([plane, pcd_rotacionado, mesh])