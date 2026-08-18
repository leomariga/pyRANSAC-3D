import random
from collections.abc import Callable

import numpy as np
from numpy.typing import NDArray

from .fit_results import LineResult
from .fit_state import FitState


class Line:
    """
    Implementation for 3D Line RANSAC.

    This object finds the equation of a line in 3D space using RANSAC method.
    This method uses 2 points from 3D space and computes a line. The selected candidate will be the line with more inliers inside the radius theshold.

    ![3D line](https://raw.githubusercontent.com/leomariga/pyRANSAC-3D/master/doc/line.gif "3D line")

    Attributes:
        A: 3D slope of the line, which is a unit vector along its direction, `np.array (3,)`.
        B: Axis interception, which is a point the line passes through, `np.array (3,)`.
        inliers: Index of the points of `pts` which fit the line.
        distances: Distance from each point of `pts` to the line, `np.array (N,)`, in the same order
            of `pts`.

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
        callback: Callable[[FitState], bool | None] | None = None,
    ) -> LineResult:
        """
        Find the best equation for the 3D line. The line in a 3d enviroment is defined as y = Ax+B, but A and B are vectors intead of scalars.

        :param pts: 3D point cloud as a `np.array (N,3)`.
        :param thresh: Threshold distance from the line which is considered inlier.
        :param maxIteration: Number of maximum iteration which RANSAC will loop over.
        :param callback: Optional callable which receives a `FitState` after every iteration.
            Return a truthy value from it to stop the fit early.
        :returns: `LineResult` with the slope `A` and the interception `B` of the line, and its
            `inliers`.

        Everything else measured while fitting is kept on the object, described in the attributes
        of this class.

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

        return LineResult(self.A, self.B, self.inliers)
