import random
from collections.abc import Callable

import numpy as np
from numpy.typing import NDArray

from .fit_results import SphereResult
from .fit_state import FitState


class Sphere:
    """
    Implementation for Sphere RANSAC. A Sphere is defined as points spaced from the center by a constant radius.


    This class finds the center and radius of a sphere. Base on article "PGP2X: Principal Geometric Primitives Parameters Extraction"

    ![3D Sphere](https://raw.githubusercontent.com/leomariga/pyRANSAC-3D/master/doc/sphere.gif "3D Sphere")

    Attributes:
        center: Center of the sphere, `np.array (3,)`.
        radius: Radius of the sphere.
        inliers: Index of the points of `pts` which fit the sphere.
        radial_distances: Distance from each point of `pts` to the center of the sphere,
            `np.array (N,)`, in the same order of `pts`.
        distances: Distance from each point of `pts` to the hull of the sphere, `np.array (N,)`, in
            the same order of `pts`.

    ---
    """

    def __init__(self) -> None:
        self.inliers: NDArray[np.intp] | list[int] = []
        self.center: list[float] = []
        self.radius: float = 0
        self.distances: NDArray[np.float64] | list[float] = []
        self.radial_distances: NDArray[np.float64] | list[float] = []

    def fit(
        self,
        pts: NDArray[np.float64],
        thresh: float = 0.2,
        maxIteration: int = 1000,
        callback: Callable[[FitState], bool | None] | None = None,
    ) -> SphereResult:
        """
        Find the parameters (center and radius) to define a Sphere.

        :param pts: 3D point cloud as a numpy array (N,3).
        :param thresh: Threshold distance from the Sphere hull which is considered inlier.
        :param maxIteration: Number of maximum iteration which RANSAC will loop over.
        :param callback: Optional callable which receives a `FitState` after every iteration.
            Return a truthy value from it to stop the fit early.

        :returns: `SphereResult` with the `center` and the `radius` of the sphere, and its `inliers`.

        Everything else measured while fitting is kept on the object, described in the attributes
        of this class.

        ---
        """

        n_points = pts.shape[0]
        best_inliers = self.inliers
        best_center = self.center
        best_radius = self.radius

        if n_points < 4:
            raise ValueError("Point cloud must contain at least 4 points!")

        for it in range(maxIteration):

            # Samples 4 random points
            id_samples = random.sample(range(0, n_points), 4)
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
            center = [0.5 * (M12 / M11), -0.5 * (M13 / M11), 0.5 * (M14 / M11)]
            radius = np.sqrt(np.dot(center, center) - (M15 / M11))

            # Distance from a point
            pt_id_inliers = []  # list of inliers ids
            dist_pt = center - pts
            dist_pt = np.linalg.norm(dist_pt, axis=1)

            # The distance to the hull is how much the point is away from the radius
            dist_hull = np.abs(dist_pt - radius)

            # Select indexes where distance is biggers than the threshold
            pt_id_inliers = np.where(dist_hull <= thresh)[0]

            is_best = len(pt_id_inliers) > len(best_inliers)
            if is_best:
                best_inliers = pt_id_inliers
                best_center = center
                best_radius = radius
                self.inliers = best_inliers
                self.center = best_center
                self.radius = best_radius
                self.radial_distances = dist_pt
                self.distances = dist_hull

            if callback is not None:
                stop = callback(
                    {
                        "iteration": it,
                        "sample_indices": id_samples,
                        "sample_points": pt_samples,
                        "model": {"center": center, "radius": radius},
                        "inliers": pt_id_inliers,
                        "best_model": {"center": best_center, "radius": best_radius},
                        "best_inliers": best_inliers,
                        "is_best": is_best,
                    }
                )
                if stop:
                    break

        return SphereResult(self.center, self.radius, self.inliers)
