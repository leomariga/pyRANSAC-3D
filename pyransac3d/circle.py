import random
from collections.abc import Callable

import numpy as np
from numpy.typing import NDArray

from .aux_functions import rodrigues_rot
from .fit_results import CircleResult
from .fit_state import FitState


class Circle:
    """
    Implementation for Circle RANSAC.

    This class finds the circle's parameters based on 3 sampled points.
    This method uses 3 points to find the circle's plane, center and radius.

    ![Circle](https://raw.githubusercontent.com/leomariga/pyRANSAC-3D/master/doc/circle.gif "Circle")


    Attributes:
        center: Center of the circle, `np.array (3,)`.
        axis: Unit normal vector of the plane which contains the circle, `np.array (3,)`.
        radius: Radius of the circle.
        inliers: Index of the points of `pts` which fit the circle.
        plane_equation: Equation of the plane which contains the circle, using Ax+By+Cz+D,
            `np.array (4,)`, where A, B and C are the same as `axis`.
        plane_distances: Signed distance from each point of `pts` to the plane of the circle,
            `np.array (N,)`, in the same order of `pts`.
        radial_distances: Distance from each point of `pts` to the hull of the circle if it was
            extruded along its axis, `np.array (N,)`, in the same order of `pts`, which is negative
            for points inside of it.
        distances: Distance from each point of `pts` to the hull of the circle, `np.array (N,)`, in
            the same order of `pts`.

    ---
    """

    def __init__(self) -> None:
        self.inliers: NDArray[np.intp] | list[int] = []
        self.center: NDArray[np.float64] | list[float] = []
        self.axis: NDArray[np.float64] | list[float] = []
        self.radius: float = 0
        self.plane_equation: list[float] = []
        self.distances: NDArray[np.float64] | list[float] = []
        self.plane_distances: NDArray[np.float64] | list[float] = []
        self.radial_distances: NDArray[np.float64] | list[float] = []

    def fit(
        self,
        pts: NDArray[np.float64],
        thresh: float = 0.2,
        maxIteration: int = 1000,
        callback: Callable[[FitState], bool | None] | None = None,
    ) -> CircleResult:
        """
        Find the parameters (axis and radius and center) to define a circle.

        :param pts: 3D point cloud as a numpy array (N,3).
        :param thresh: Threshold distance from the cylinder hull which is considered inlier.
        :param maxIteration: Number of maximum iteration which RANSAC will loop over.
        :param callback: Optional callable which receives a `FitState` after every iteration.
            Return a truthy value from it to stop the fit early.

        :returns: `CircleResult` with the `center`, the `axis` and the `radius` of the circle, and
            its `inliers`.

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

            k = -np.sum(np.multiply(vecC, pt_samples[1, :]))
            plane_eq = [vecC[0], vecC[1], vecC[2], k]

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

            # Distance from a point to the circle's plane
            # vecC is already unitary, so no division by its norm is needed
            dist_pt_plane = pts.dot(vecC) + k
            vecC_stakado = np.stack([vecC] * n_points, 0)
            # Distance from a point to the circle hull if it is infinite along its axis (perpendicular distance to the plane)
            # The norm of the cross product in this case is the distance from the point to circle's axis (using the normal vector of the plane)
            #https://math.stackexchange.com/questions/3525110/how-to-prove-the-norm-of-a-cross-product-equals-the-norm-of-a-projection
            dist_pt_inf_circle = np.cross(vecC_stakado, (center - pts))
            # We need to subtract the radius to get the distance from the point to the circle's hull, not to the center axis.
            dist_pt_inf_circle = np.linalg.norm(dist_pt_inf_circle, axis=1) - radius

            # https://math.stackexchange.com/questions/31049/distance-from-a-point-to-circles-closest-point
            # The distance from a point to a circle will be the hipotenusa
            dist_pt = np.sqrt(np.square(dist_pt_inf_circle) + np.square(dist_pt_plane))

            # Select indexes where distance is biggers than the threshold
            pt_id_inliers = np.where(dist_pt <= thresh)[0]

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
                self.plane_equation = plane_eq
                self.plane_distances = dist_pt_plane
                self.radial_distances = dist_pt_inf_circle
                self.distances = dist_pt

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

        return CircleResult(self.center, self.axis, self.radius, self.inliers)
