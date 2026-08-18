import random
from collections.abc import Callable

import numpy as np
from numpy.typing import NDArray

from .fit_results import PointResult
from .fit_state import FitState


class Point:
    """
    Implementation for Point RANSAC.

    This object finds the coordinate of a point in 3D space using RANSAC method.
    The point with more neighbors in a determined radius (`thresh`) will be selected as the best candidate.

    ![3D point](https://raw.githubusercontent.com/leomariga/pyRANSAC-3D/master/doc/point.gif "3D Point")

    Attributes:
        center: Point selected as best candidate, `np.array (3,)`.
        inliers: Index of the points of `pts` which are neighbors of `center`.
        distances: Distance from each point of `pts` to `center`, `np.array (N,)`, in the same order
            of `pts`.

    ---
    """

    def __init__(self) -> None:
        self.inliers: NDArray[np.intp] | list[int] = []
        self.center: NDArray[np.float64] | list[float] = []
        self.distances: NDArray[np.float64] | list[float] = []

    def fit(
        self,
        pts: NDArray[np.float64],
        thresh: float = 0.2,
        maxIteration: int = 10000,
        callback: Callable[[FitState], bool | None] | None = None,
    ) -> PointResult:
        """
        Find the best point for the 3D Point representaiton. The Point in a 3d enviroment is defined as a X, Y Z coordinate with more neighbors around.

        :param pts: 3D point cloud as a `np.array (N,3)`.
        :param thresh: Threshold radius from the point which is considered inlier.
        :param maxIteration: Number of maximum iteration which RANSAC will loop over.
        :param callback: Optional callable which receives a `FitState` after every iteration.
            Return a truthy value from it to stop the fit early.
        :returns: `PointResult` with the `center` selected as best candidate and its `inliers`.

        Everything else measured while fitting is kept on the object, described in the attributes
        of this class.

        ---
        """
        n_points = pts.shape[0]
        best_inliers = []
        best_center = self.center

        if n_points < 1:
            raise ValueError("Point cloud must contain at least 1 point!")

        for it in range(maxIteration):

            # Samples 1 random points
            id_samples = random.sample(range(0, n_points), 1)
            pt_samples = pts[id_samples]

            # Verify the distance from this point to the other

            pt_id_inliers = []  # list of inliers ids
            dist_pt = pt_samples[0, :] - pts
            dist_pt = np.linalg.norm(dist_pt, axis=1)

            # Select indexes where distance is biggers than the threshold
            pt_id_inliers = np.where(np.abs(dist_pt) <= thresh)[0]

            is_best = len(pt_id_inliers) > len(best_inliers)
            if is_best:
                best_inliers = pt_id_inliers
                best_center = pt_samples[0, :]
                self.inliers = best_inliers
                self.center = best_center
                self.distances = dist_pt

            if callback is not None:
                stop = callback(
                    {
                        "iteration": it,
                        "sample_indices": id_samples,
                        "sample_points": pt_samples,
                        "model": {"center": pt_samples[0, :]},
                        "inliers": pt_id_inliers,
                        "best_model": {"center": best_center},
                        "best_inliers": best_inliers,
                        "is_best": is_best,
                    }
                )
                if stop:
                    break

        return PointResult(self.center, self.inliers)
