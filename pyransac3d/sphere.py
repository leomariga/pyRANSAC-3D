import numpy as np
import random
import copy 
from .aux import *
import time

class Sphere:
    """ 
    Implementation for Sphere RANSAC. A Sphere is defined as points spaced from the center by a constant radius. 


    This class finds the center and radius of a sphere. Base on article "PGP2X: Principal Geometric Primitives Parameters Extraction"

    ---
    """

    def __init__(self):
        self.inliers = []
        self.center = []
        self.radius = 0

    def fit(self, pts, pcd_load, thresh=0.2, maxIteration=1000):
        """ 
        Find the parameters (center and radius) to define a Sphere. 

        :param pts: 3D point cloud as a numpy array (N,3).
        :param thresh: Threshold distance from the Sphere hull which is considered inlier.
        :param maxIteration: Number of maximum iteration which RANSAC will loop over.

        :returns: 
        - `center`: Center of the cylinder np.array(1,3) which the cylinder axis is passing through.
        - `radius`: Radius of cylinder.
        - `inliers`: Inlier's index from the original point cloud.

        ---
        """

        def rotate_view(vis):
            ctr.rotate(0.1, 0.0)
            return False



        vis = o3d.visualization.Visualizer()
        vis.create_window()

        n_points = pts.shape[0]
        best_inliers = []

        for it in range(maxIteration):

            # Samples 4 random points 
            id_samples = random.sample(range(1, n_points-1), 4)
            pt_samples = pts[id_samples]

            # We calculate the 4x4 determinant by dividing the problem in determinants of 3x3 matrix

            # Multiplied by (x²+y²+z²)
            d_matrix = np.ones((4, 4))
            for i in range(4):
                d_matrix[i, 0] = pt_samples[i, 0]
                d_matrix[i, 1] = pt_samples[i, 1]
                d_matrix[i, 2] = pt_samples[i, 2]
            M11 = np.linalg.det(d_matrix)

            # Multiplied by x
            for i in range(4):
                d_matrix[i, 0] = np.dot(pt_samples[i], pt_samples[i])
                d_matrix[i, 1] = pt_samples[i, 1]
                d_matrix[i, 2] = pt_samples[i, 2]
            M12 = np.linalg.det(d_matrix)

            # Multiplied by y
            for i in range(4):
                d_matrix[i, 0] = np.dot(pt_samples[i], pt_samples[i])
                d_matrix[i, 1] = pt_samples[i, 0]
                d_matrix[i, 2] = pt_samples[i, 2]
            M13 = np.linalg.det(d_matrix)

            # Multiplied by z
            for i in range(4):
                d_matrix[i, 0] = np.dot(pt_samples[i], pt_samples[i])
                d_matrix[i, 1] = pt_samples[i, 0]
                d_matrix[i, 2] = pt_samples[i, 1]
            M14 = np.linalg.det(d_matrix)


            # Multiplied by 1
            for i in range(4):
                d_matrix[i, 0] = np.dot(pt_samples[i], pt_samples[i])
                d_matrix[i, 1] = pt_samples[i, 0]
                d_matrix[i, 2] = pt_samples[i, 1]
                d_matrix[i, 3] = pt_samples[i, 2]
            M15 = np.linalg.det(d_matrix)

            # Now we calculate the center and radius
            center = [0.5*(M12/M11), -0.5*(M13/M11), 0.5*(M14/M11)]
            radius = np.sqrt(np.dot(center, center) - (M15 / M11))


            # Distance from a point to a line
            pt_id_inliers = [] # list of inliers ids
            dist_pt = center - pts
            dist_pt = np.linalg.norm(dist_pt, axis=1)


            # Select indexes where distance is biggers than the threshold
            pt_id_inliers = np.where(np.abs(dist_pt-radius) <= thresh)[0]

            if(len(pt_id_inliers) > len(best_inliers)):
                best_inliers = pt_id_inliers
                self.inliers = best_inliers
                self.center = center
                self.radius = radius


            if(radius > 8):
                continue
            mesh_cylinder2 = o3d.geometry.TriangleMesh.create_sphere(radius=self.radius)
            mesh_cylinder2.compute_vertex_normals()
            mesh_cylinder2.paint_uniform_color([0, 1, 0])
            print(self.center)
            mesh_cylinder2 = mesh_cylinder2.translate((self.center[0], self.center[1], self.center[2]))

            mesh_cylinder = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
            mesh_cylinder.compute_vertex_normals()
            mesh_cylinder.paint_uniform_color([1, 0, 0])
            mesh_cylinder = mesh_cylinder.translate((center[0], center[1], center[2]))


            lin = pcd_load.select_by_index(pt_id_inliers).paint_uniform_color([1, 0, 0])
            lin2 = pcd_load.select_by_index(self.inliers).paint_uniform_color([0, 1, 0])
            vis.clear_geometries()
            #time.sleep(0.01)
            vis.add_geometry(copy.deepcopy(pcd_load))
            vis.add_geometry(mesh_cylinder)
            vis.add_geometry(mesh_cylinder2)
            vis.add_geometry(lin)
            vis.add_geometry(lin2)

            ctr = vis.get_view_control()
            ctr.rotate(-it*6, 0)
            ctr.set_zoom(0.7)
            ctr.set_lookat([0, 0, 0])

            vis.poll_events()
            vis.update_renderer()
            time.sleep(0.04)

        return self.center, self.radius,  self.inliers


