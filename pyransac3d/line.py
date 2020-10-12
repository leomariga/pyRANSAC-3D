import numpy as np
import random
import copy 
from .aux import *
import open3d as o3d
import time

class Line:
    """ 
    Implementation for 3D Line RANSAC.

    This object finds the equation of a line in 3D space using RANSAC method. 
    This method uses 2 points from 3D space and computes a line. The selected candidate will be the line with more inliers inside the radius theshold. 

    ---
    """

    def __init__(self):
        self.inliers = []
        self.A = []
        self.B = []

    def fit(self, pts,pcd_load, thresh=0.2, maxIteration=1000):
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


        def rotate_view(vis):
            ctr.rotate(0.1, 0.0)
            return False


        vis = o3d.visualization.Visualizer()
        vis.create_window()

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


            # R = get_rotationMatrix_from_vectors([0, 0, 1], vecA_norm )
            # mesh_cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=1, height=5000)
            # mesh_cylinder.compute_vertex_normals()
            # mesh_cylinder.paint_uniform_color([1, 0, 0])
            # mesh_cylinder = mesh_cylinder.rotate(R, center=[0, 0, 0])
            # mesh_cylinder = mesh_cylinder.translate((pt_samples[0,:][0], pt_samples[0,:][1], pt_samples[0,:][2]))

            R2 = get_rotationMatrix_from_vectors([0, 0, 1], self.A )
            mesh_cylinder2 = o3d.geometry.TriangleMesh.create_cylinder(radius=1, height=5000)
            mesh_cylinder2.compute_vertex_normals()
            mesh_cylinder2.paint_uniform_color([0, 1, 0])
            mesh_cylinder2 = mesh_cylinder2.rotate(R2, center=[0, 0, 0])
            mesh_cylinder2 = mesh_cylinder2.translate((self.B[0], self.B[1], self.B[2]))


            lin = pcd_load.select_by_index(pt_id_inliers).paint_uniform_color([1, 0, 0])
            lin2 = pcd_load.select_by_index(self.inliers).paint_uniform_color([0, 1, 0])
            vis.clear_geometries()
            #time.sleep(0.01)
            vis.add_geometry(copy.deepcopy(pcd_load))
            # vis.add_geometry(mesh_cylinder)
            vis.add_geometry(mesh_cylinder2)
            vis.add_geometry(lin)
            vis.add_geometry(lin2)

            ctr = vis.get_view_control()
            ctr.rotate(-it*6, 0)
            ctr.set_zoom(0.1)
            ctr.set_lookat([0, 0, 0])

            vis.poll_events()
            vis.update_renderer()
            time.sleep(0.04)

        return self.A, self.B, self.inliers


