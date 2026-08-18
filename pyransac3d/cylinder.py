import random
from collections.abc import Callable

import numpy as np
from numpy.typing import NDArray

from .aux_functions import rodrigues_rot
from .fit_results import CylinderResult
from .fit_state import FitState


class Cylinder:
    """
    !!! warning "The cylinder RANSAC does NOT present good results on real data on the current version"

    We are working to make a better algorithim using normals. If you want to contribute, please create a MR on github.
    Or give us ideas on [this issue](https://github.com/leomariga/pyRANSAC-3D/issues/13)

    Implementation for cylinder RANSAC.

    This class finds a infinite height cilinder and returns the cylinder axis, center and radius.

    Attributes:
        center: A point the axis of the cylinder passes through, `np.array (3,)`.
        axis: Unit vector describing the direction of the cylinder's axis, `np.array (3,)`.
        radius: Radius of the cylinder.
        inliers: Index of the points of `pts` which fit the cylinder.
        radial_distances: Distance from each point of `pts` to the axis of the cylinder,
            `np.array (N,)`, in the same order of `pts`.
        distances: Distance from each point of `pts` to the hull of the cylinder, `np.array (N,)`,
            in the same order of `pts`.

    ---
    """

    def __init__(self) -> None:
        self.inliers: NDArray[np.intp] | list[int] = []
        self.center: NDArray[np.float64] | list[float] = []
        self.axis: NDArray[np.float64] | list[float] = []
        self.radius: float = 0
        self.distances: NDArray[np.float64] | list[float] = []
        self.radial_distances: NDArray[np.float64] | list[float] = []

    def fit(
        self,
        pts: NDArray[np.float64],
        thresh: float = 0.2,
        maxIteration: int = 10000,
        callback: Callable[[FitState], bool | None] | None = None,
    ) -> CylinderResult:
        """
        Find the parameters (axis and radius) defining a cylinder.

        :param pts: 3D point cloud as a numpy array (N,3).
        :param thresh: Threshold distance from the cylinder hull which is considered inlier.
        :param maxIteration: Number of maximum iteration which RANSAC will loop over.
        :param callback: Optional callable which receives a `FitState` after every iteration.
            Return a truthy value from it to stop the fit early.

        :returns: `CylinderResult` with the `center`, the `axis` and the `radius` of the cylinder,
            and its `inliers`.

        Everything else measured while fitting is kept on the object, described in the attributes
        of this class.

        ---
        """

        n_points = pts.shape[0]
        best_inliers = []
        best_center = self.center
        best_axis = self.axis
        best_radius = self.radius

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
            vecA_norm = vecA / np.linalg.norm(vecA)
            vecB = pt_samples[2, :] - pt_samples[0, :]
            vecB_norm = vecB / np.linalg.norm(vecB)

            # Now we compute the cross product of vecA and vecB to get vecC which is normal to the plane
            vecC = np.cross(vecA_norm, vecB_norm)
            # Avoid parallel vectors (division by zero error)
            if np.any(vecC) == False:
                continue
            vecC = vecC / np.linalg.norm(vecC)

            # Now we calculate the rotation of the points with rodrigues equation
            P_rot = rodrigues_rot(pt_samples, vecC, [0, 0, 1])

            # Find center from 3 points
            # http://paulbourke.net/geometry/circlesphere/
            # Find lines that intersect the points
            # Slope:
            ma = 0
            mb = 0
            while ma == 0:
                ma = (P_rot[1, 1] - P_rot[0, 1]) / (P_rot[1, 0] - P_rot[0, 0])
                mb = (P_rot[2, 1] - P_rot[1, 1]) / (P_rot[2, 0] - P_rot[1, 0])
                if ma == 0:
                    P_rot = np.roll(P_rot, -1, axis=0)
                else:
                    break

            # Calulate the center by verifying intersection of each orthogonal line
            p_center_x = (
                ma * mb * (P_rot[0, 1] - P_rot[2, 1])
                + mb * (P_rot[0, 0] + P_rot[1, 0])
                - ma * (P_rot[1, 0] + P_rot[2, 0])
            ) / (2 * (mb - ma))
            p_center_y = -1 / (ma) * (p_center_x - (P_rot[0, 0] + P_rot[1, 0]) / 2) + (P_rot[0, 1] + P_rot[1, 1]) / 2
            p_center = [p_center_x, p_center_y, P_rot[0, 2]]
            radius = np.linalg.norm(p_center - P_rot[0, :])

            # Remake rodrigues rotation
            center = rodrigues_rot(p_center, [0, 0, 1], vecC)[0]

            # Distance from a point to a line
            pt_id_inliers = []  # list of inliers ids
            vecC_stakado = np.stack([vecC] * n_points, 0)
            dist_pt = np.cross(vecC_stakado, (center - pts))
            dist_pt = np.linalg.norm(dist_pt, axis=1)

            # The distance to the hull is how much the point is away from the radius
            dist_hull = np.abs(dist_pt - radius)

            # Select indexes where distance is biggers than the threshold
            pt_id_inliers = np.where(dist_hull <= thresh)[0]

            is_best = len(pt_id_inliers) > len(best_inliers)
            if is_best:
                best_inliers = pt_id_inliers
                best_center = center
                best_axis = vecC
                best_radius = radius
                self.inliers = best_inliers
                self.center = best_center
                self.axis = best_axis
                self.radius = best_radius
                self.radial_distances = dist_pt
                self.distances = dist_hull

            if callback is not None:
                stop = callback(
                    {
                        "iteration": it,
                        "sample_indices": id_samples,
                        "sample_points": pt_samples,
                        "model": {"center": center, "axis": vecC, "radius": radius},
                        "inliers": pt_id_inliers,
                        "best_model": {"center": best_center, "axis": best_axis, "radius": best_radius},
                        "best_inliers": best_inliers,
                        "is_best": is_best,
                    }
                )
                if stop:
                    break

        return CylinderResult(self.center, self.axis, self.radius, self.inliers)
