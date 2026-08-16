import random

import numpy as np

from .aux_functions import convex_hull_2d, min_bounding_rect, rodrigues_rot


class Cuboid:
    """
    Implementation for box (Cuboid) RANSAC.

    A cuboid is defined as convex polyhedron bounded by six faces formed by three orthogonal normal vectors. Cats love to play with this kind of geometry.
    This method uses 6 points to find 3 best plane equations orthogonal to eachother.

    We could use a recursive planar RANSAC, but it would use 9 points instead. Orthogonality makes this algorithm more efficient.

    Once the three faces are found, the bounded box which contains them is measured. The face with
    most inliers gives the reference axis, and the remaining rotation around it is the one which
    makes the smallest footprint when the inliers are projected on the plane orthogonal to that
    axis. This way `fit(.)` also gives the `center`, the `extents` and the `axes` of a real box,
    and not only the equations of three infinite planes.

    ![Cuboid](https://raw.githubusercontent.com/leomariga/pyRANSAC-3D/master/doc/cuboid.gif "Cuboid")

    ---
    """

    def __init__(self):
        self.inliers = []
        self.equation = []
        self.center = []
        self.extents = []
        self.axes = []
        self.distances = []
        self.plane_distances = []
        self.face_normals = []
        self.face_inlier_count = []
        self.ref_face_index = 0
        self.z_bounds = []
        self.hull_angle = 0

    def fit(self, pts, thresh=0.05, maxIteration=5000, callback=None):
        """
        Find the cuboid which best fits the point cloud, from the 3 orthogonal planes of its faces.

        :param pts: 3D point cloud as a `np.array (N,3)`.
        :param thresh: Threshold distance from the cylinder radius which is considered inlier.
        :param maxIteration: Number of maximum iteration which RANSAC will loop over.
        :param callback: Optional callable invoked after every non-degenerate iteration
            with a state `dict`. Useful to plot the fitting progress, inspect
            intermediate results, or implement a custom early-stopping criterion. If it
            returns a truthy value, fitting stops early and the current best result is
            returned. Treat the arrays in the state `dict` as read-only. State keys:
            - `iteration`: current iteration index (0-based)
            - `sample_indices`: indices of the points sampled this iteration
            - `sample_points`: the sampled points, `np.array (6, 3)`
            - `model`: `dict` with this iteration's candidate `equation` (3 planes)
            - `inliers`: inlier indices found for this iteration's candidate
            - `best_model`: `dict` with the best `equation` (3 planes) found so far and the
            `center`, `extents` and `axes` of its box. Measuring the box is much more expensive
            than testing a candidate, so it only happens when the best candidate changes
            - `best_inliers`: best inlier indices found so far
            - `is_best`: `True` if this iteration became the new best candidate
        :returns:
        - `center`: Center of the cuboid `np.array (3,)`
        - `extents`: Size of the cuboid along each one of its axes `np.array (3,)`
        - `axes`: Orthonormal axes of the cuboid, one per row, where the last one is the normal of
        the face with most inliers `np.array (3, 3)`
        - `inliers`: Inlier's index from the original point cloud. `np.array (1, M)`

        The equations of the 3 orthogonal planes used to build the box are not returned, but they
        are kept in the object and can be read from `self.equation` as a `np.array (3, 4)`.

        The distances measured to select the inliers are not returned either, but they are kept in
        the object, in the same order of `pts`:
        - `self.plane_distances`: distance from each point to each one of the 3 planes,
        `np.array (3, N)`
        - `self.distances`: distance from each point to the closest one of the 3 planes,
        `np.array (N,)`

        The measurements used to build the box from the 3 planes are kept in the object too:
        - `self.face_normals`: normal of each one of the 3 planes `np.array (3, 3)`
        - `self.face_inlier_count`: how many inliers are closest to each plane `np.array (3,)`
        - `self.ref_face_index`: index of the plane with most inliers, whose normal is the
        reference axis `self.axes[2]`
        - `self.z_bounds`: smallest and biggest coordinate of the inliers along the reference
        axis `np.array (2,)`
        - `self.hull_angle`: rotation around the reference axis, in radians, which gives the
        smallest footprint of the inliers `float`

        Call `get_corners(.)` to get the 8 vertices of the box and `get_transform(.)` to get its
        rotation and translation as a single matrix.

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
            dist_p4_plane = (
                plane_eq[0][0] * pt_samples[3, 0]
                + plane_eq[0][1] * pt_samples[3, 1]
                + plane_eq[0][2] * pt_samples[3, 2]
                + plane_eq[0][3]
            ) / np.sqrt(plane_eq[0][0] ** 2 + plane_eq[0][1] ** 2 + plane_eq[0][2] ** 2)

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

            k = -np.sum(np.multiply(vecG, pt_samples[5, :]))
            plane_eq.append([vecG[0], vecG[1], vecG[2], k])
            plane_eq = np.asarray(plane_eq)
            # We have to find the value D for the last plane.

            # Distance from a point to a plane
            # https://mathworld.wolfram.com/Point-PlaneDistance.html
            pt_id_inliers = []  # list of inliers ids
            dist_pt = []
            for id_plane in range(plane_eq.shape[0]):
                dist_pt.append(
                    np.abs(
                        (
                            plane_eq[id_plane, 0] * pts[:, 0]
                            + plane_eq[id_plane, 1] * pts[:, 1]
                            + plane_eq[id_plane, 2] * pts[:, 2]
                            + plane_eq[id_plane, 3]
                        )
                        / np.sqrt(plane_eq[id_plane, 0] ** 2 + plane_eq[id_plane, 1] ** 2 + plane_eq[id_plane, 2] ** 2)
                    )
                )

            # Select indexes where distance is biggers than the threshold
            dist_pt = np.asarray(dist_pt)
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

        return self.center, self.extents, self.axes, self.inliers

    def _measure_box(self, pts_inliers, plane_eq):
        """
        Measure the box which contains the inliers of 3 orthogonal planes.

        It sets `self.center`, `self.extents` and `self.axes`, plus the measurements used to build
        them, which are all documented in `fit(.)`.

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

    def get_corners(self):
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

    def get_transform(self):
        """
        Get the rotation and the translation of the fitted cuboid as a single matrix.

        The matrix takes a point from the cuboid's own frame, where the box is centered on the
        origin and aligned with the coordinate axes with the size given by `self.extents`, to the
        frame of the point cloud. The rotation is `transform[0:3, 0:3]` and the translation is
        `transform[0:3, 3]`, which is the same as `self.center`.

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
