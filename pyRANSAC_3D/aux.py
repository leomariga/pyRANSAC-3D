import numpy as np
import random
import copy 

def get_rotationMatrix_from_vectors(u, v):
    """ 
    Create a rotation matrix that rotates the space from a 3D vector `u` to a 3D vector `v`

    :param u: Orign vector `np.array (1,3)`.
    :param v: Destiny vector `np.array (1,3)`.

    :returns: Rotation matrix `np.array (3, 3)`

    ---
    """

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



def rodrigues_rot(P, n0, n1):
    """ 
    Rotate a set of point between two normal vectors using Rodrigues' formula. 

    :param P: Set of points `np.array (N,3)`.
    :param n0: Orign vector `np.array (1,3)`.
    :param n1: Destiny vector `np.array (1,3)`.

    :returns: Set of points P, but rotated `np.array (N, 3)`

    ---
    """

    # If P is only 1d array (coords of single point), fix it to be matrix
    P = np.asarray(P)
    if P.ndim == 1:
        P = P[np.newaxis,:]
    
    # Get vector of rotation k and angle theta
    n0 = n0/np.linalg.norm(n0)
    n1 = n1/np.linalg.norm(n1)
    k = np.cross(n0,n1)
    P_rot = np.zeros((len(P),3))
    if(np.linalg.norm(k)!=0):
        k = k/np.linalg.norm(k)
        theta = np.arccos(np.dot(n0,n1))
        
        # Compute rotated points
        for i in range(len(P)):
            P_rot[i] = P[i]*np.cos(theta) + np.cross(k,P[i])*np.sin(theta) + k*np.dot(k,P[i])*(1-np.cos(theta))
    else:
        P_rot = P
    return P_rot