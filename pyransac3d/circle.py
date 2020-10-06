import numpy as np
import random
import copy 
from .aux import *

class Circle:
    """ 
    Implementation for Circle RANSAC.

    This class finds the circle's parameters based on 3 sampled points. 
    This method uses 3 points to find the circle's plane, center and radius.

    ![Circle](https://raw.githubusercontent.com/leomariga/pyRANSAC-3D/master/doc/circle.gif "Circle")


    ---
    """

    def __init__(self):
        self.inliers = []
        self.center = []
        self.axis = []
        self.radius = 0

    def fit(self, pts, thresh=0.2, maxIteration=1000):
        """ 
        Find the parameters (axis and radius and center) to define a circle. 

        :param pts: 3D point cloud as a numpy array (N,3).
        :param thresh: Threshold distance from the cylinder hull which is considered inlier.
        :param maxIteration: Number of maximum iteration which RANSAC will loop over.

        :returns: 
        - `center`: Center of the circle np.array(1,3) which the circle center is passing through.
        - `axis`: Vector describing circle's plane normal as np.array(1,3).
        - `radius`: Radius of the circle.
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

            k = -np.sum(np.multiply(vecC, pt_samples[1,:]))
            plane_eq = [vecC[0], vecC[1], vecC[2], k]

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

            # Distance from a point to the circle's plane
            dist_pt_plane = (plane_eq[0]*pts[:,0]+plane_eq[1]*pts[:, 1]+plane_eq[2]*pts[:, 2]+plane_eq[3])/np.sqrt(plane_eq[0]**2+plane_eq[1]**2+plane_eq[2]**2)
            vecC_stakado =  np.stack([vecC]*n_points,0)
            # Distance from a point to the circle hull if it is infinite along its axis (perpendicular distance to the plane)
            dist_pt_inf_circle = np.cross(vecC_stakado, (center- pts))
            dist_pt_inf_circle = np.linalg.norm(dist_pt_inf_circle, axis=1) - radius

            # https://math.stackexchange.com/questions/31049/distance-from-a-point-to-circles-closest-point
            # The distance from a point to a circle will be the hipotenusa 
            dist_pt = np.sqrt(np.square(dist_pt_inf_circle)+np.square(dist_pt_plane))


            # Select indexes where distance is biggers than the threshold
            pt_id_inliers = np.where(dist_pt <= thresh)[0]

            if(len(pt_id_inliers) > len(best_inliers)):
                best_inliers = pt_id_inliers
                self.inliers = best_inliers
                self.center = center
                self.axis = vecC
                self.radius = radius

        return self.center, self.axis, self.radius,  self.inliers


