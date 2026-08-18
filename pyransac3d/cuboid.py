import random
from collections.abc import Callable

import numpy as np
from numpy.typing import NDArray

from .aux_functions import convex_hull_2d, min_bounding_rect, rodrigues_rot
from .fit_results import CuboidResult
from .fit_state import FitState


class Cuboid:
    """
    Implementation for box (Cuboid) RANSAC.

    A cuboid is defined as convex polyhedron bounded by six faces formed by three orthogonal normal vectors.
    This method uses 6 points to find 3 best plane equations orthogonal to eachother.

    Once the three faces are found, the bounded box which contains them is measured. The face with
    most inliers gives the reference axis, and the remaining rotation around it is the one which
    makes the smallest footprint when the inliers are projected on the plane orthogonal to that
    axis. This way `fit(.)` also gives the `center`, the `extents` and the `axes` of a real box,
    and not only the equations of three infinite planes.

    ![Cuboid](https://raw.githubusercontent.com/leomariga/pyRANSAC-3D/master/doc/cuboid.gif "Cuboid")

    Attributes:
        center: Center of the cuboid, `np.array (3,)`.
        extents: Size of the cuboid along each one of its axes, `np.array (3,)`.
        axes: Orthonormal axes of the cuboid, one per row, where the last one is the normal of the
            face with most inliers, `np.array (3, 3)`.
        inliers: Index of the points of `pts` which fit the cuboid.
        equation: Equation of the 3 orthogonal planes used to build the box, one per row, using
            Ax+By+Cz+D, `np.array (3, 4)`.
        plane_distances: Distance from each point of `pts` to each one of the 3 planes,
            `np.array (3, N)`, in the same order of `pts`.
        distances: Distance from each point of `pts` to the closest one of the 3 planes,
            `np.array (N,)`, in the same order of `pts`.
        face_normals: Normal of each one of the 3 planes, one per row, `np.array (3, 3)`.
        face_inlier_count: How many inliers are closest to each plane, `np.array (3,)`.
        ref_face_index: Index of the plane with most inliers, whose normal is the reference axis
            `axes[2]`.
        z_bounds: Smallest and biggest coordinate of the inliers along the reference axis,
            `np.array (2,)`.
        hull_angle: Rotation around the reference axis, in radians, which gives the smallest
            footprint of the inliers.

    ---
    """

    def __init__(self) -> None:
        self.inliers: NDArray[np.intp] | list[int] = []
        self.equation: NDArray[np.float64] | list[float] = []
        self.center: NDArray[np.float64] | list[float] = []
        self.extents: NDArray[np.float64] | list[float] = []
        self.axes: NDArray[np.float64] | list[float] = []
        self.distances: NDArray[np.float64] | list[float] = []
        self.plane_distances: NDArray[np.float64] | list[float] = []
        self.face_normals: NDArray[np.float64] | list[float] = []
        self.face_inlier_count: NDArray[np.intp] | list[int] = []
        self.ref_face_index: int | np.intp = 0
        self.z_bounds: NDArray[np.float64] | list[float] = []
        self.hull_angle: float = 0

    def fit(
        self,
        pts: NDArray[np.float64],
        thresh: float = 0.05,
        maxIteration: int = 5000,
        callback: Callable[[FitState], bool | None] | None = None,
    ) -> CuboidResult:
        """
        Find the cuboid which best fits the point cloud, from the 3 orthogonal planes of its faces.

        :param pts: 3D point cloud as a `np.array (N,3)`.
        :param thresh: Threshold distance from the cylinder radius which is considered inlier.
        :param maxIteration: Number of maximum iteration which RANSAC will loop over.
        :param callback: Optional callable which receives a `FitState` after every iteration.
            Return a truthy value from it to stop the fit early. Passing a callback measures the box
            of every new best candidate, so its `best_model` also carries the `center`, the
            `extents` and the `axes` of the box, at the cost of a slower fit.
        :returns: `CuboidResult` with the `center`, the `extents` and the `axes` of the cuboid, and
            its `inliers`.

        Everything else measured while fitting is kept on the object, described in the attributes of
        this class. Call `get_corners(.)` to get the 8 vertices of the box and `get_transform(.)` to
        get its rotation and translation as a single matrix.

        ---
        """
        n_points = pts.shape[0]
        best_eq = []
        best_inliers = []
        best_plane_distances = []
        best_distances = []

        if n_points < 6:
            raise ValueError("Point cloud must contain at least 6 points!")

        for it in range(maxIteration):
            plane_eq = []

            # Samples 6 random points
            id_samples = random.sample(range(0, n_points), 6)
            pt_samples = pts[id_samples]

            # We have to find the plane equation described by those 3 points
            # We find first 2 vectors that are part of this plane
            # A = pt2 - pt1
            # B = pt3 - pt1

            vecA = pt_samples[1, :] - pt_samples[0, :]
            vecB = pt_samples[2, :] - pt_samples[0, :]

            # Now we compute the cross product of vecA and vecB to get vecC which is normal to the plane
            vecC = np.cross(vecA, vecB)

            # Avoid parallel vectors (division by zero error)
            if np.any(vecC) == False:
                continue

            # The plane equation will be vecC[0]*x + vecC[1]*y + vecC[0]*z = -k
            # We have to use a point to find k
            vecC = vecC / np.linalg.norm(vecC)  # Normal

            k = -np.sum(np.multiply(vecC, pt_samples[1, :]))
            plane_eq.append([vecC[0], vecC[1], vecC[2], k])

            # Now we use another point to find a orthogonal plane 2
            # Calculate distance from the point to the first plane
            # vecC is already unitary, so no division by its norm is needed
            dist_p4_plane = np.dot(vecC, pt_samples[3, :]) + k

            # vecC is already normal (module 1) so we only have to discount from the point, the distance*unity = distance*normal
            # A simple way of understanding this is we move our point along the normal until it reaches the plane
            p4_proj_plane = pt_samples[3, :] - dist_p4_plane * vecC

            # Now, with help of our point p5 we can find another plane P2 which contains p4, p4_proj, p5 and
            vecD = p4_proj_plane - pt_samples[3, :]
            vecE = pt_samples[4, :] - pt_samples[3, :]
            vecF = np.cross(vecD, vecE)
            # Avoid parallel vectors (division by zero error)
            if np.any(vecF) == False:
                continue
            vecF = vecF / np.linalg.norm(vecF)  # Normal
            k = -np.sum(np.multiply(vecF, pt_samples[4, :]))
            plane_eq.append([vecF[0], vecF[1], vecF[2], k])

            # The last plane will be orthogonal to the first and sacond plane (and its normals will be orthogonal to first and second planes' normal)
            vecG = np.cross(vecC, vecF)
            vecG = vecG / np.linalg.norm(vecG)  # Normal

            k = -np.sum(np.multiply(vecG, pt_samples[5, :]))
            plane_eq.append([vecG[0], vecG[1], vecG[2], k])
            plane_eq = np.asarray(plane_eq)
            # We have to find the value D for the last plane.

            # Distance from a point to a plane
            # https://mathworld.wolfram.com/Point-PlaneDistance.html
            # The 3 normals are already unitary, so no division by their norms is needed
            pt_id_inliers = []  # list of inliers ids
            dist_pt = np.abs(pts.dot(plane_eq[:, 0:3].T) + plane_eq[:, 3]).T

            # Select indexes where distance is biggers than the threshold
            min_dist_pt = np.amin(dist_pt, axis=0)
            pt_id_inliers = np.where(np.abs(min_dist_pt) <= thresh)[0]

            is_best = len(pt_id_inliers) > len(best_inliers)
            if is_best:
                best_eq = plane_eq
                best_inliers = pt_id_inliers
                best_plane_distances = dist_pt
                best_distances = min_dist_pt
            self.inliers = best_inliers
            self.equation = best_eq
            self.plane_distances = best_plane_distances
            self.distances = best_distances

            if callback is not None:
                if is_best:
                    self._measure_box(pts[best_inliers], best_eq)
                stop = callback(
                    {
                        "iteration": it,
                        "sample_indices": id_samples,
                        "sample_points": pt_samples,
                        "model": {"equation": plane_eq},
                        "inliers": pt_id_inliers,
                        "best_model": {
                            "equation": best_eq,
                            "center": self.center,
                            "extents": self.extents,
                            "axes": self.axes,
                        },
                        "best_inliers": best_inliers,
                        "is_best": is_best,
                    }
                )
                if stop:
                    break

        # Every iteration was degenerate, so there is no box to measure
        if len(best_inliers) > 0:
            self._measure_box(pts[best_inliers], best_eq)

        return CuboidResult(self.center, self.extents, self.axes, self.inliers)

    def _measure_box(self, pts_inliers: NDArray[np.float64], plane_eq: NDArray[np.float64]) -> None:
        """
        Measure the box which contains the inliers of 3 orthogonal planes.

        It sets `center`, `extents` and `axes`, plus the measurements used to build them, which are
        all described in the attributes of this class.

        :param pts_inliers: Inlier points of the 3 planes as a `np.array (M,3)`.
        :param plane_eq: Equation of the 3 orthogonal planes `np.array (3, 4)`.

        ---
        """

        # The 3 normals are already unitary and orthogonal to each other, so the distance from a
        # point to a plane is just the dot product with the normal plus the plane's D
        normals = plane_eq[:, 0:3]
        dist_pt = np.abs(pts_inliers.dot(normals.T) + plane_eq[:, 3])

        # Each inlier belongs to the plane it is closest to. The face with more points is the one
        # we trust the most, because the 6 sampled points give a coarse orientation
        n_pts_face = np.bincount(np.argmin(dist_pt, axis=1), minlength=3)
        ref_face = np.argmax(n_pts_face)
        ref_normal = normals[ref_face]

        # Align the reference normal with Z. The extent along Z is then a dimension of the box and
        # deleting Z projects every inlier on the face which is orthogonal to the reference normal
        pts_ref = rodrigues_rot(pts_inliers, ref_normal, [0, 0, 1])
        z_min = np.amin(pts_ref[:, 2])
        z_max = np.amax(pts_ref[:, 2])

        # The projection of a box on that plane is a rectangle, so the remaining rotation and the
        # other 2 dimensions are the ones which enclose the projected points with the smallest area
        hull = convex_hull_2d(pts_ref[:, 0:2])
        angle, _, width, depth, center_2d, _ = min_bounding_rect(hull)

        # The rectangle axes are the rows of its rotation matrix, so we only have to rotate them
        # back together with the center to describe the box in the original space
        axes = np.asarray(
            [
                [np.cos(angle), np.sin(angle), 0],
                [-np.sin(angle), np.cos(angle), 0],
                [0, 0, 1],
            ]
        )
        self.axes = rodrigues_rot(axes, [0, 0, 1], ref_normal)
        self.center = rodrigues_rot([center_2d[0], center_2d[1], (z_min + z_max) / 2], [0, 0, 1], ref_normal)[0]
        self.extents = np.asarray([width, depth, z_max - z_min])
        self.face_normals = normals
        self.face_inlier_count = n_pts_face
        self.ref_face_index = ref_face
        self.z_bounds = np.asarray([z_min, z_max])
        self.hull_angle = angle

    def get_corners(self) -> NDArray[np.float64]:
        """
        Get the vertices of the fitted cuboid.

        :returns: The 8 vertices of the cuboid `np.array (8, 3)`, the first 4 on one face and the
        last 4 on the opposite one, both in the same order.

        ---
        """

        if len(self.center) == 0:
            raise ValueError("The cuboid has no corners because it was not fitted yet. Call fit() first!")

        signs = np.asarray(
            [
                [-1, -1, -1],
                [1, -1, -1],
                [1, 1, -1],
                [-1, 1, -1],
                [-1, -1, 1],
                [1, -1, 1],
                [1, 1, 1],
                [-1, 1, 1],
            ]
        )

        # Walk from the center along each axis by half of its extent
        return self.center + (signs * (self.extents / 2)).dot(self.axes)

    def get_transform(self) -> NDArray[np.float64]:
        """
        Get the rotation and the translation of the fitted cuboid as a single matrix.

        The matrix takes a point from the cuboid's own frame, where the box is centered on the
        origin and aligned with the coordinate axes with the size given by `extents`, to the
        frame of the point cloud. The rotation is `transform[0:3, 0:3]` and the translation is
        `transform[0:3, 3]`, which is the same as `center`.

        Keep in mind a box looks the same after being rotated by 90 degrees around its own axes, so
        this is one of the 24 rotations which describe the same cuboid, and not a unique answer.

        :returns: Homogeneous transformation matrix from the cuboid frame to the point cloud frame
        `np.array (4, 4)`

        ---
        """

        if len(self.center) == 0:
            raise ValueError("The cuboid has no transform because it was not fitted yet. Call fit() first!")

        # The rows of self.axes are the axes of the box written in the point cloud frame, so they
        # are the columns of the rotation matrix which takes the box frame to the point cloud frame
        transform = np.eye(4)
        transform[0:3, 0:3] = np.asarray(self.axes).T
        transform[0:3, 3] = self.center

        return transform
