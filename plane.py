import open3d as o3d
import numpy as np
import random


def findPlane(pts, thresh=0.1, minPoints=100):
	n_points = pts.shape[0]
	print(n_points)

	# Samples 3 random points 
	id_samples = random.sample(range(1, n_points-1), 3)
	print(id_samples)
	pt_saples = pts[id_samples]
	print(pt_saples)

	# We have to find the plane equation described by those 3 points
	# We find first 2 vectors that are part of this plane
	# A = pt2 - pt1
	# B = pt3 - pt1

	vecA = pt_saples[1,:] - pt_saples[0,:]
	vecB = pt_saples[2,:] - pt_saples[0,:]
	print(vecA)
	print(vecB)

	# Now we compute the cross product of vecA and vecB to get vecC which is normal to the plane
	vecC = np.cross(vecA, vecB)
	print(vecC)

	# The plane equation will be vecC[0]*x + vecC[1]*y + vecC[0]*z = -k
	# We have to use a point to find k

	k = -np.sum(np.multiply(vecC, pt_saples[1,:]))
	plane_eq = [vecC[0], vecC[1], vecC[2], k]
	plane_eq = plane_eq/np.linalg.norm(plane_eq)
	print(plane_eq)






# Load saved point cloud and visualize it
pcd_load = o3d.io.read_point_cloud("caixa.ply")
#o3d.visualization.draw_geometries([pcd_load])
points = np.asarray(pcd_load.points)

findPlane(points)
