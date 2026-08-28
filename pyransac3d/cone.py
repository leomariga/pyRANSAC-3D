import random
from collections.abc import Callable

import numpy as np
from numpy.typing import NDArray

from .aux_functions import estimate_normals
from .fit_results import ConeResult
from .fit_state import FitState


class Cone:
    """
    Implementation for cone RANSAC.

    This class finds an infinite right circular cone, and returns its apex, the direction of its
    axis and its half-angle, which is the angle between the axis and the surface of the cone.

    A cone cannot be found from points alone: three points are not enough to define the six
    parameters of a cone, but three points **and their normals** are, because every plane tangent
    to a cone passes through its apex. So `fit(.)` works on the normals of the surface, which it
    takes from `normals` when they are given, and estimates from the cloud itself when they are
    not.

    Attributes:
        apex: Point where the surface of the cone converges, `np.array (3,)`.
        axis: Unit vector describing the direction of the cone's axis, pointing from the apex
            towards the opening, `np.array (3,)`.
        angle: Half-angle of the cone in radians, between its axis and its surface.
        inliers: Index of the points of `pts` which fit the cone.
        distances: Distance from each point of `pts` to the surface of the cone, `np.array (N,)`,
            in the same order of `pts`.

    ---
    """

    def __init__(self) -> None:
        self.inliers: NDArray[np.intp] | list[int] = []
        self.apex: NDArray[np.float64] | list[float] = []
        self.axis: NDArray[np.float64] | list[float] = []
        self.angle: float = 0
        self.distances: NDArray[np.float64] | list[float] = []

    def fit(
        self,
        pts: NDArray[np.float64],
        thresh: float = 0.2,
        maxIteration: int = 1000,
        normals: NDArray[np.float64] | None = None,
        k_neighbors: int = 16,
        callback: Callable[[FitState], bool | None] | None = None,
    ) -> ConeResult:
        """
        Find the parameters (apex, axis and half-angle) defining a cone.

        :param pts: 3D point cloud as a numpy array (N,3).
        :param thresh: Threshold distance from the cone surface which is considered inlier.
        :param maxIteration: Number of maximum iteration which RANSAC will loop over.
        :param normals: Optional unit normals of the surface at each point of `pts`, `np.array
            (N,3)`. When `None`, they are estimated from `pts` with `estimate_normals(.)`, which
            is quadratic in the cloud size. Passing the normals you already have, such as the ones
            of an Open3D cloud, skips it. Their orientation does not matter, only their direction.
        :param k_neighbors: Number of neighbors used to estimate the normals, ignored when
            `normals` is given.
        :param callback: Optional callable which receives a `FitState` after every iteration.
            Return a truthy value from it to stop the fit early.

        :returns: `ConeResult` with the `apex`, the `axis` and the `angle` of the cone, and its
            `inliers`.

        Everything else measured while fitting is kept on the object, described in the attributes
        of this class.

        ---
        """

        pts = np.asarray(pts, dtype=float)
        n_points = pts.shape[0]

        if n_points < 3:
            raise ValueError("Point cloud must contain at least 3 points!")

        if normals is None:
            normals = estimate_normals(pts, k=k_neighbors)
        else:
            normals = np.asarray(normals, dtype=float)
            if normals.shape != pts.shape:
                raise ValueError("Normals must have the same shape as the point cloud!")
            norms = np.linalg.norm(normals, axis=1)
            if np.any(norms == 0):
                raise ValueError("Normals must not contain zero vectors!")
            normals = normals / norms[:, np.newaxis]

        best_inliers: NDArray[np.intp] | list[int] = []
        best_apex = self.apex
        best_axis = self.axis
        best_angle = self.angle

        for it in range(maxIteration):

            # Samples 3 random points, which are taken with the normal of the surface on them
            id_samples = random.sample(range(0, n_points), 3)
            pt_samples = pts[id_samples]
            normal_samples = normals[id_samples]

            candidate = self._cone_from_samples(pt_samples, normal_samples)
            if candidate is None:
                continue
            apex, axis, angle = candidate

            dist_pt = self._distance_to_cone(pts, apex, axis, angle)
            pt_id_inliers = np.where(dist_pt <= thresh)[0]

            is_best = len(pt_id_inliers) > len(best_inliers)
            if is_best:
                best_inliers = pt_id_inliers
                best_apex = apex
                best_axis = axis
                best_angle = angle
                self.inliers = best_inliers
                self.apex = best_apex
                self.axis = best_axis
                self.angle = best_angle
                self.distances = dist_pt

            if callback is not None:
                stop = callback(
                    {
                        "iteration": it,
                        "sample_indices": id_samples,
                        "sample_points": pt_samples,
                        "model": {"apex": apex, "axis": axis, "angle": angle},
                        "inliers": pt_id_inliers,
                        "best_model": {"apex": best_apex, "axis": best_axis, "angle": best_angle},
                        "best_inliers": best_inliers,
                        "is_best": is_best,
                    }
                )
                if stop:
                    break

        return ConeResult(self.apex, self.axis, self.angle, self.inliers)

    @staticmethod
    def _cone_from_samples(
        pt_samples: NDArray[np.float64],
        normal_samples: NDArray[np.float64],
    ) -> tuple[NDArray[np.float64], NDArray[np.float64], float] | None:
        """
        Solve the cone which has the three sampled points on its surface.

        Every plane tangent to a cone contains its apex, so the apex is where the three tangent
        planes of the samples meet. Seen from the apex, the three points are then three unit
        directions which make the same angle with the axis, which means they lie on a circle of
        the unit sphere, and the axis is the normal of the plane of that circle.

        :param pt_samples: The three sampled points `np.array (3,3)`.
        :param normal_samples: Unit normal of the surface on each sample `np.array (3,3)`.

        :returns: The `apex`, the unit `axis` and the half-`angle` of the cone, or `None` when
            the samples do not define one

        ---
        """

        # Each tangent plane is dot(x, n) = dot(p, n), so the apex solves the 3x3 system.
        # A singular system means the normals are parallel or coplanar, and the planes either
        # never meet or meet along a whole line
        rhs = np.einsum("ij,ij->i", normal_samples, pt_samples)
        try:
            apex = np.linalg.solve(normal_samples, rhs)
        except np.linalg.LinAlgError:
            return None
        if not np.all(np.isfinite(apex)):
            return None

        # Directions from the apex to each sample, which all make the half-angle with the axis
        directions = pt_samples - apex
        lengths = np.linalg.norm(directions, axis=1)
        if np.any(lengths < 1e-12):
            return None
        directions = directions / lengths[:, np.newaxis]

        # The three directions lie on a circle of the unit sphere, and the axis is normal to it
        axis = np.cross(directions[1] - directions[0], directions[2] - directions[0])
        axis_norm = np.linalg.norm(axis)
        if axis_norm < 1e-12:
            return None
        axis = axis / axis_norm

        # Point the axis from the apex towards the samples, so the cone is the single nappe
        # which actually holds them
        cos_angle = float(np.dot(directions[0], axis))
        if cos_angle < 0:
            axis = -axis
            cos_angle = -cos_angle

        angle = float(np.arccos(np.clip(cos_angle, -1.0, 1.0)))

        # A degenerate cone is either a line or a plane, and neither is a cone
        if not 1e-6 < angle < np.pi / 2 - 1e-6:
            return None

        return apex, axis, angle

    @staticmethod
    def _distance_to_cone(
        pts: NDArray[np.float64],
        apex: NDArray[np.float64],
        axis: NDArray[np.float64],
        angle: float,
    ) -> NDArray[np.float64]:
        """
        Measure how far every point is from the surface of a cone.

        The distance is taken perpendicular to the surface: a point which is seen from the apex
        under the angle `alpha` sits `d * sin(alpha - angle)` away from it, where `d` is how far
        it is from the apex.

        :param pts: 3D point cloud as a numpy array (N,3).
        :param apex: Apex of the cone `np.array (3,)`.
        :param axis: Unit axis of the cone `np.array (3,)`.
        :param angle: Half-angle of the cone in radians.

        :returns: Distance from each point to the surface of the cone `np.array (N,)`

        ---
        """

        to_pts = pts - apex
        lengths = np.linalg.norm(to_pts, axis=1)

        # The apex itself is on the surface, whatever the angle is, and normalizing it would
        # divide by zero
        safe_lengths = np.where(lengths < 1e-12, 1.0, lengths)
        cos_alpha = np.einsum("ij,j->i", to_pts, axis) / safe_lengths
        alpha = np.arccos(np.clip(cos_alpha, -1.0, 1.0))

        distances = np.abs(lengths * np.sin(alpha - angle))
        return np.where(lengths < 1e-12, 0.0, distances)
