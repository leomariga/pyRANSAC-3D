import open3d as o3d
import numpy as np
import random
import matplotlib
import matplotlib.pyplot as plt

class Cuboid:

	def Cuboid(self, inliers, equation):
		self.inliers = []
		self.equation = []

	def fit(self, pts, thresh=0.05, minPoints=100, maxIteration=5000):
		n_points = pts.shape[0]
		print(n_points)
		best_eq = []
		best_inliers = []

		for it in range(maxIteration):
			plane_eq = []
			# Samples 3 random points 
			id_samples = random.sample(range(1, n_points-1), 6)
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
			#print(vecC)

			# The plane equation will be vecC[0]*x + vecC[1]*y + vecC[0]*z = -k
			# We have to use a point to find k
			vecC = vecC / np.linalg.norm(vecC) # Normal

			k = -np.sum(np.multiply(vecC, pt_samples[1,:]))
			plane_eq.append([vecC[0], vecC[1], vecC[2], k])
			
			#print(plane_eq)

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

			
			#print(plane_eq)

			# The last plane will be orthogonal to the first and sacond plane (and its normals will be orthogonal to first and second planes' normal)
			vecG = np.cross(vecC, vecF)

			k = -np.sum(np.multiply(vecG, pt_samples[5,:]))
			plane_eq.append([vecG[0], vecG[1], vecG[2], k])
			plane_eq = np.asarray(plane_eq)
			# We have to find the value D for the last plane.
			# We can make a better estimation for all Ds calculating the D value for all points for all normals. 
			# When analysing a perfect plane, all points of the same face when doing a scalar multiplication will result in the same D value
			# Therefore, the most frequent D value would be the chosen D
			#normals = np.asarray([vecC, vecF, vecG])
			

			#print(normals)
			# D_n = []
			# D_best = []
			# for id_normal in range(normals.shape[0]):
			# 	D_n.append(-(normals[id_normal, 0]*pts[:,0]+normals[id_normal,1]*pts[:, 1]+normals[id_normal,2]*pts[:, 2]))


			# # Verify how many nearby points exists for each D. The most frequent is probrably the one we want.
			# for id_normal in range(normals.shape[0]):
			# 	hist, bin_edges = np.histogram(D_n[id_normal], bins=np.arange(np.min(D_n[id_normal]), np.max(D_n[id_normal]), thresh/2))
			# 	if(hist.shape[0] > 0):
			# 		id_max_hist = np.argmax(np.asarray(hist))
			# 		D_best.append((bin_edges[id_max_hist] + bin_edges[id_max_hist+1])/2)
			# 	else:
			# 		break
			# D_best = np.asarray(D_best)

			# plane_eq[0] = [normals[0, 0], normals[1, 0], normals[2, 0], D_best[0]]
			# plane_eq[1] = [normals[1, 0], normals[1, 1], normals[2, 1], D_best[1]]
			# plane_eq.append([normals[2, 0], normals[1, 2], normals[2, 2], D_best[2]])
			# plane_eq = np.asarray(plane_eq)

			# if(it == 4999):
			# 	print(D_best)
			# 	print(bin_edges)
			# 	print(D_best.shape)
			# 	print(bin_edges.shape)
			# 	num_bins = 1000
			# 	fig, axs = plt.subplots(1, 3)
			# 	# the histogram of the data
			# 	axs[0].hist(D_n[0], bins = num_bins)
			# 	axs[1].hist(D_n[1], bins = num_bins)
			# 	axs[2].hist(D_n[2], bins = num_bins)
			# 	# Tweak spacing to prevent clipping of ylabel
			# 	fig.tight_layout()
			# 	plt.show()

			# Distance from a point to a plane 
			# https://mathworld.wolfram.com/Point-PlaneDistance.html
			pt_id_inliers = [] # list of inliers ids
			dist_pt = []
			for id_plane in range(plane_eq.shape[0]):
				dist_pt.append(np.abs((plane_eq[id_plane,0]*pts[:,0]+plane_eq[id_plane,1]*pts[:, 1]+plane_eq[id_plane,2]*pts[:, 2]+plane_eq[id_plane,3])/np.sqrt(plane_eq[id_plane,0]**2+plane_eq[id_plane,1]**2+plane_eq[id_plane,2]**2)))
			
			# Select indexes where distance is biggers than the threshold
			dist_pt = np.asarray(dist_pt)
			#print(dist_pt.shape)
			min_dist_pt = np.amin(dist_pt, axis=0)
			#print(min_dist_pt.shape)
			pt_id_inliers = np.where(np.abs(min_dist_pt) <= thresh)[0]



			if(len(pt_id_inliers) > len(best_inliers)):
				best_eq = plane_eq
				best_inliers = pt_id_inliers
			self.inliers = best_inliers
			self.equation = best_eq
		return best_eq, best_inliers

		








# Load saved point cloud and visualize it
pcd_load = o3d.io.read_point_cloud("caixa.ply")
#o3d.visualization.draw_geometries([pcd_load])
points = np.asarray(pcd_load.points)

plano1 = Cuboid()

best_eq, best_inliers = plano1.fit(points, 0.04)
plane = pcd_load.select_by_index(best_inliers).paint_uniform_color([1, 0, 0])
not_plane = pcd_load.select_by_index(best_inliers, invert=True)
o3d.visualization.draw_geometries([not_plane, plane])
