# Draft from github post
from concurrent.futures import ThreadPoolExecutor

import numpy as np
from random import Random
from typing import List, Tuple
from abc import ABC, abstractmethod
import time
import sys

import open3d as o3d

class BaseRansac(ABC):
    def __init__(self, seed=None, n_workers=None):
        self.random = Random(seed)
        self.n_workers = n_workers

    def fit(self, points: np.ndarray, thresh: float = 0.05, max_iteration: int = 5000) -> Tuple[List[float], List[float]]:
        """
        :param points: A numpy array of points, of shape (# points, 3)
        :param thresh: The distance threshold to include points as inliers
        :param max_iteration: How many (parallel) Ransac iterations to run
        :returns:
            best_eq: A list of integers representing the best 'equation' for the primitive shape.
            best_inliers: A list of indices of points that fit the shape.
        """
        best_eq = []
        best_inliers = []
        # by default, the number of workers is the same as the max_itertion
        # so that the maximum number of iterations can be executed concurrently

        jobs = ((points, float(thresh)) for _ in range(max_iteration))
        # We can use a with statement to ensure threads are cleaned up promptly
        with ThreadPoolExecutor(max_workers=self.n_workers) as executor:
            for eq, point_id_inliers in executor.map(self.iteration, *zip(*jobs)):
                if len(point_id_inliers) > len(best_inliers):
                    best_eq = eq
                    best_inliers = point_id_inliers
        return best_eq, best_inliers

    @abstractmethod
    def iteration(self, points: np.ndarray, thresh: float) -> Tuple[List[float], List[float]]:
        pass

class PlaneClass(BaseRansac):
    def iteration(self, pts: np.ndarray, thresh: float)-> Tuple[List[float], List[float]]:
        # Samples 3 random points
        id_samples = self.random.sample(range(0, pts.shape[0]), 3)
        pt_samples = pts[id_samples]

        # We have to find the plane equation described by those 3 points
        # We find first 2 vectors that are part of this plane
        # A = pt2 - pt1
        # B = pt3 - pt1

        vecA = pt_samples[1, :] - pt_samples[0, :]
        vecB = pt_samples[2, :] - pt_samples[0, :]

        # Now we compute the cross product of vecA and vecB to get vecC which is normal to the plane
        vecC = np.cross(vecA, vecB)

        # The plane equation will be vecC[0]*x + vecC[1]*y + vecC[0]*z = -k
        # We have to use a point to find k
        vecC = vecC / np.linalg.norm(vecC)
        k = -np.sum(np.multiply(vecC, pt_samples[1, :]))
        plane_eq = [vecC[0], vecC[1], vecC[2], k]

        # Distance from a point to a plane
        # https://mathworld.wolfram.com/Point-PlaneDistance.html
        pt_id_inliers = []  # list of inliers ids
        dist_pt = (
            plane_eq[0] * pts[:, 0] + plane_eq[1] * pts[:, 1] + plane_eq[2] * pts[:, 2] + plane_eq[3]
        ) / np.sqrt(plane_eq[0] ** 2 + plane_eq[1] ** 2 + plane_eq[2] ** 2)

        # Select indexes where distance is biggers than the threshold
        pt_id_inliers = np.where(np.abs(dist_pt) <= thresh)[0]
        return list(pts), list(pt_id_inliers)


# Load saved point cloud and visualize it
pcd_load = o3d.io.read_point_cloud("dataset/caixa.ply")
# o3d.visualization.draw_geometries([pcd_load])
points = np.asarray(pcd_load.points)
t_before = time.time()
plano1 = PlaneClass(n_workers = 3)
best_eq, best_inliers = plano1.fit(points, 0.01, max_iteration=500)
dt = time.time() - t_before
print("dt: ", dt)
plane = pcd_load.select_by_index(best_inliers).paint_uniform_color([1, 0, 0])
obb = plane.get_oriented_bounding_box()
obb2 = plane.get_axis_aligned_bounding_box()
obb.color = [0, 0, 1]
obb2.color = [0, 1, 0]
not_plane = pcd_load.select_by_index(best_inliers, invert=True)

o3d.visualization.draw_geometries([not_plane, plane, obb, obb2])