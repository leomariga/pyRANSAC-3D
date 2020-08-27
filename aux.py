import open3d as o3d
import numpy as np
import random
import copy 

import open3d as o3d
import numpy as np
import copy

def drawPlane(plane):
	print(plane.rMatrix.T)
	box = o3d.geometry.TriangleMesh.create_box(width=plane.size[0], height=plane.size[1], depth=0.05).translate(plane.limits_f_plane[0, :])
	box = box.rotate(plane.rMatrix.T, center=(0,0,0)).translate(plane.tMatrix)
	return box


def get_rotationMatrix_from_vectors(u, v):
    # Lets find a vector which is ortogonal to both u and v
    w = np.cross(u, v)
    
    # This orthogonal vector w has some interesting proprieties
    # |w| = sin of the required rotation 
    # dot product of w and goal_normal_plane is the cos of the angle
    c = np.dot(u, v)
    s = np.linalg.norm(w)


    # Now, we compute rotation matrix from rodrigues formula 
    # https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
    # https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d

    # We calculate the skew symetric matrix of the ort_vec
    Sx = np.asarray([[0, -w[2], w[1]],
                   [w[2], 0, -w[0] ],
                   [-w[1], w[0], 0]])
    R = np.eye(3) + Sx + Sx.dot(Sx) * ((1 - c) / (s ** 2))
    return R