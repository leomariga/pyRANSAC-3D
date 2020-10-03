import numpy as np
import random
import copy 
from .aux import *

class Line:
    """ 
    Implementation for 3D Line RANSAC.

    This object finds the equation of a line in 3D space using RANSAC method. 
    This method uses 2 points from 3D space and computes a line. The selected candidate will be the line with more inliers inside the radius theshold. 

    ![3D line](https://raw.githubusercontent.com/leomariga/pyRANSAC-3D/master/doc/line.gif "3D line")

    ---
    """

    def __init__(self):
        self.inliers = []
        self.A = []
        self.B = []

    def fit(self, pts, thresh=0.2, maxIteration=1000):
        """ 
        Find the best equation for the 3D line. The line in a 3d enviroment is defined as y = Ax+B, but A and B are vectors intead of scalars.

        :param pts: 3D point cloud as a `np.array (N,3)`.
        :param thresh: Threshold distance from the line which is considered inlier.
        :param maxIteration: Number of maximum iteration which RANSAC will loop over.
        :returns:
        - `A`: 3D slope of the line (angle) `np.array (1, 3)`
        - `B`: Axis interception as `np.array (1, 3)`
        - `inliers`: Inlier's index from the original point cloud. `np.array (1, M)`
        ---
        """
        n_points = pts.shape[0]
        best_eq = []
        best_inliers = []

        for it in range(maxIteration):

            # Samples 3 random points 
            id_samples = random.sample(range(1, n_points-1), 2)
            pt_samples = pts[id_samples]

            # The line defined by two points is defined as P2 - P1
            vecA = pt_samples[1,:] - pt_samples[0,:]
            vecA_norm = vecA / np.linalg.norm(vecA)


            # Distance from a point to a line
            pt_id_inliers = [] # list of inliers ids
            vecC_stakado =  np.stack([vecA_norm]*n_points,0)
            dist_pt = np.cross(vecC_stakado, (pt_samples[0,:]- pts))
            dist_pt = np.linalg.norm(dist_pt, axis=1)


            # Select indexes where distance is biggers than the threshold
            pt_id_inliers = np.where(np.abs(dist_pt) <= thresh)[0]

            if(len(pt_id_inliers) > len(best_inliers)):
                best_inliers = pt_id_inliers
                self.inliers = best_inliers
                self.A = vecA_norm
                self.B = pt_samples[0,:]

        return self.A, self.B, self.inliers


