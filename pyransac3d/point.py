import numpy as np
import random
import copy 
from .aux_functions import *

class Point:
    """ 
    Implementation for Point RANSAC.

    This object finds the coordinate of a point in 3D space using RANSAC method. 
    The point with more neighbors in a determined radius (`thresh`) will be selected as the best candidate.

    ![3D point](https://raw.githubusercontent.com/leomariga/pyRANSAC-3D/master/doc/point.gif "3D Point")

    ---
    """

    def __init__(self):
        self.inliers = []
        self.center = []

    def fit(self, pts, thresh=0.2, maxIteration=10000):
        """ 
        Find the best point for the 3D Point representaiton. The Point in a 3d enviroment is defined as a X, Y Z coordinate with more neighbors around.

        :param pts: 3D point cloud as a `np.array (N,3)`.
        :param thresh: Threshold radius from the point which is considered inlier.
        :param maxIteration: Number of maximum iteration which RANSAC will loop over.
        :returns:
        - `center`: Point selected as best candidate `np.array (1, 3)`
        - `inliers`: Inlier's index from the original point cloud. `np.array (1, M)`

        ---
        """
        n_points = pts.shape[0]
        best_inliers = []

        for it in range(maxIteration):

            # Samples 1 random points 
            id_samples = random.sample(range(1, n_points-1), 1)
            pt_samples = pts[id_samples]

            # Verify the distance from this point to the other

            pt_id_inliers = [] # list of inliers ids
            dist_pt = pt_samples[0,:] - pts
            dist_pt = np.linalg.norm(dist_pt, axis=1)


            # Select indexes where distance is biggers than the threshold
            pt_id_inliers = np.where(np.abs(dist_pt) <= thresh)[0]

            if(len(pt_id_inliers) > len(best_inliers)):
                best_inliers = pt_id_inliers
                self.inliers = best_inliers
                self.center = pt_samples[0,:]

        return self.center, self.inliers


