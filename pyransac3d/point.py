import numpy as np
import random
import copy 
from .aux import *
import time

class Point:
    """ 
    Implementation for Point RANSAC.

    This object finds the coordinate of a point in 3D space using RANSAC method. 
    The point with more neighbors in a determined radius (`thresh`) will be selected as the best candidate.

    ![3D line](https://raw.githubusercontent.com/leomariga/pyRANSAC-3D/master/doc/line.gif "3D line")

    ---
    """

    def __init__(self):
        self.inliers = []
        self.center = []

    def fit(self, pts, pcd_load, thresh=0.2, maxIteration=10000):
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

        def rotate_view(vis):
            ctr.rotate(0.1, 0.0)
            return False



        vis = o3d.visualization.Visualizer()
        vis.create_window()

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


            mesh_cylinder2 = o3d.geometry.TriangleMesh.create_sphere(radius=0.2)
            mesh_cylinder2.compute_vertex_normals()
            mesh_cylinder2.paint_uniform_color([0, 1, 0])
            mesh_cylinder2 = mesh_cylinder2.translate((self.center[0], self.center[1], self.center[2]))

            mesh_cylinder = o3d.geometry.TriangleMesh.create_sphere(radius=0.2)
            mesh_cylinder.compute_vertex_normals()
            mesh_cylinder.paint_uniform_color([1, 0, 0])
            mesh_cylinder = mesh_cylinder.translate((pt_samples[0,:][0], pt_samples[0,:][1], pt_samples[0,:][2]))


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

        return self.center, self.inliers


