import random

import numpy as np


class Plane:
    """
    Implementation of planar RANSAC.

    Class for Plane object, which finds the equation of a infinite plane using RANSAC algorithim.

    Call `fit(.)` to randomly take 3 points of pointcloud to verify inliers based on a threshold.

    ![Plane](https://raw.githubusercontent.com/leomariga/pyRANSAC-3D/master/doc/plano.gif "Plane")

    ---
    """

    def __init__(self):
        self.inliers = []
        self.equation = []
        self.distances = []

    def fit(self, pts, thresh=0.05, maxIteration=1000, callback=None):
        """
        Find the best equation for a plane.

        :param pts: 3D point cloud as a `np.array (N,3)`.
        :param thresh: Threshold distance from the plane which is considered inlier.
        :param maxIteration: Number of maximum iteration which RANSAC will loop over.
        :param callback: Optional callable invoked after every non-degenerate iteration
            with a state `dict`. Useful to plot the fitting progress, inspect
            intermediate results, or implement a custom early-stopping criterion. If it
            returns a truthy value, fitting stops early and the current best result is
            returned. Treat the arrays in the state `dict` as read-only. State keys:
            - `iteration`: current iteration index (0-based)
            - `sample_indices`: indices of the points sampled this iteration
            - `sample_points`: the sampled points, `np.array (3, 3)`
            - `model`: `dict` with this iteration's candidate `equation`
            - `inliers`: inlier indices found for this iteration's candidate
            - `best_model`: `dict` with the best `equation` found so far
            - `best_inliers`: best inlier indices found so far
            - `is_best`: `True` if this iteration became the new best candidate
        :returns:
        - `self.equation`:  Parameters of the plane using Ax+By+Cy+D `np.array (1, 4)`
        - `self.inliers`: points from the dataset considered inliers

        The distances used to select the inliers are not returned, but they are kept in the object
        and can be read from `self.distances` as a `np.array (N,)`, in the same order of `pts`.
        They are signed, so the sign tells on which side of the plane the point is.

        ---
        """
        n_points = pts.shape[0]
        best_eq = []
        best_inliers = []
        best_distances = []

        if n_points < 3:
            raise ValueError("Point cloud must contain at least 3 points!")

        for it in range(maxIteration):

            # Samples 3 random points
            id_samples = random.sample(range(0, n_points), 3)
            pt_samples = pts[id_samples]

            # We have to find the plane equation described by those 3 points
            # We find first 2 vectors that are part of this plane
            # A = pt2 - pt1
            # B = pt3 - pt1

            vecA = pt_samples[1, :] - pt_samples[0, :]
            vecB = pt_samples[2, :] - pt_samples[0, :]

            # Now we compute the cross product of vecA and vecB to get vecC which is normal to the plane
            vecC = np.cross(vecA, vecB)
            
            # if vectors A and B are parallel vector C will be a zero vector resulting in a divide by zero error in the next step
            if np.any(vecC) == False:
                continue

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
            is_best = len(pt_id_inliers) > len(best_inliers)
            if is_best:
                best_eq = plane_eq
                best_inliers = pt_id_inliers
                best_distances = dist_pt
            self.inliers = best_inliers
            self.equation = best_eq
            self.distances = best_distances

            if callback is not None:
                stop = callback(
                    {
                        "iteration": it,
                        "sample_indices": id_samples,
                        "sample_points": pt_samples,
                        "model": {"equation": plane_eq},
                        "inliers": pt_id_inliers,
                        "best_model": {"equation": best_eq},
                        "best_inliers": best_inliers,
                        "is_best": is_best,
                    }
                )
                if stop:
                    break

        return self.equation, self.inliers
