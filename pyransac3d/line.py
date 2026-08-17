import random
from collections.abc import Callable
from typing import Any

import numpy as np
from numpy.typing import NDArray


class Line:
    """
    Implementation for 3D Line RANSAC.

    This object finds the equation of a line in 3D space using RANSAC method.
    This method uses 2 points from 3D space and computes a line. The selected candidate will be the line with more inliers inside the radius theshold.

    ![3D line](https://raw.githubusercontent.com/leomariga/pyRANSAC-3D/master/doc/line.gif "3D line")

    ---
    """

    def __init__(self) -> None:
        self.inliers: NDArray[np.intp] | list[int] = []
        self.A: NDArray[np.float64] | list[float] = []
        self.B: NDArray[np.float64] | list[float] = []
        self.distances: NDArray[np.float64] | list[float] = []

    def fit(
        self,
        pts: NDArray[np.float64],
        thresh: float = 0.2,
        maxIteration: int = 1000,
        callback: Callable[[dict[str, Any]], bool | None] | None = None,
    ) -> tuple[
        NDArray[np.float64] | list[float],
        NDArray[np.float64] | list[float],
        NDArray[np.intp] | list[int],
    ]:
        """
        Find the best equation for the 3D line. The line in a 3d enviroment is defined as y = Ax+B, but A and B are vectors intead of scalars.

        :param pts: 3D point cloud as a `np.array (N,3)`.
        :param thresh: Threshold distance from the line which is considered inlier.
        :param maxIteration: Number of maximum iteration which RANSAC will loop over.
        :param callback: Optional callable invoked after every iteration with a state
            `dict`. Useful to plot the fitting progress, inspect intermediate results,
            or implement a custom early-stopping criterion. If it returns a truthy
            value, fitting stops early and the current best result is returned. Treat
            the arrays in the state `dict` as read-only. State keys:
            - `iteration`: current iteration index (0-based)
            - `sample_indices`: indices of the points sampled this iteration
            - `sample_points`: the sampled points, `np.array (2, 3)`
            - `model`: `dict` with this iteration's candidate `A` and `B`
            - `inliers`: inlier indices found for this iteration's candidate
            - `best_model`: `dict` with the best `A` and `B` found so far
            - `best_inliers`: best inlier indices found so far
            - `is_best`: `True` if this iteration became the new best candidate
        :returns:
        - `A`: 3D slope of the line (angle) `np.array (1, 3)`
        - `B`: Axis interception as `np.array (1, 3)`
        - `inliers`: Inlier's index from the original point cloud. `np.array (1, M)`

        The distances used to select the inliers are not returned, but they are kept in the object
        and can be read from `self.distances` as a `np.array (N,)`, in the same order of `pts`.
        ---
        """
        n_points = pts.shape[0]
        best_inliers = []
        best_A = self.A
        best_B = self.B

        if n_points < 2:
            raise ValueError("Point cloud must contain at least 2 points!")

        for it in range(maxIteration):

            # Samples 2 random points
            id_samples = random.sample(range(0, n_points), 2)
            pt_samples = pts[id_samples]

            # The line defined by two points is defined as P2 - P1
            vecA = pt_samples[1, :] - pt_samples[0, :]
            vecA_norm = vecA / np.linalg.norm(vecA)

            # Distance from a point to a line
            pt_id_inliers = []  # list of inliers ids
            vecC_stakado = np.stack([vecA_norm] * n_points, 0)
            dist_pt = np.cross(vecC_stakado, (pt_samples[0, :] - pts))
            dist_pt = np.linalg.norm(dist_pt, axis=1)

            # Select indexes where distance is biggers than the threshold
            pt_id_inliers = np.where(np.abs(dist_pt) <= thresh)[0]

            is_best = len(pt_id_inliers) > len(best_inliers)
            if is_best:
                best_inliers = pt_id_inliers
                best_A = vecA_norm
                best_B = pt_samples[0, :]
                self.inliers = best_inliers
                self.A = best_A
                self.B = best_B
                self.distances = dist_pt

            if callback is not None:
                stop = callback(
                    {
                        "iteration": it,
                        "sample_indices": id_samples,
                        "sample_points": pt_samples,
                        "model": {"A": vecA_norm, "B": pt_samples[0, :]},
                        "inliers": pt_id_inliers,
                        "best_model": {"A": best_A, "B": best_B},
                        "best_inliers": best_inliers,
                        "is_best": is_best,
                    }
                )
                if stop:
                    break

        return self.A, self.B, self.inliers
