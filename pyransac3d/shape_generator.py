import numpy as np

# How much bigger than the shape itself is the box where the outliers are scattered, when the
# caller does not give explicit bounds
OUTLIER_MARGIN = 1.5


class ShapeGenerator:
    """
    Generator of synthetic point clouds for every primitive shape of the library.

    Each method samples points on the surface of one shape from the parameters which describe it,
    so the same parameters can be compared against the ones found by the RANSAC fitters. The clouds
    are not perfect on purpose: `noise` scatters the points around the ideal surface and
    `n_outliers` adds points which do not belong to the shape at all, which is what RANSAC is
    supposed to handle.

    The points are always shuffled, so the outliers are mixed with the rest of the cloud, and the
    only dependency is NumPy, which makes the clouds cheap to build and easy to use on tests.

    All the randomness comes from a single `np.random.Generator` created on `__init__(.)`, so
    giving it a `seed` makes every cloud reproducible.

    ---
    """

    def __init__(self, seed=None):
        """
        Create a generator of point clouds.

        :param seed: Seed of the internal `np.random.Generator`. Use an integer to get the same
            clouds on every run, or `None` to get a different cloud every time.

        ---
        """

        self.rng = np.random.default_rng(seed)

    def point(self, center, n_points=200, noise=0.05, n_outliers=0, outlier_bounds=None):
        """
        Generate a cloud of points gathered around a single 3D coordinate.

        :param center: Coordinate where the points are gathered `np.array (3,)`.
        :param n_points: Number of points of the cluster, without counting the outliers.
        :param noise: Standard deviation of the gaussian noise added to every coordinate.
        :param n_outliers: Number of points scattered around which do not belong to the shape.
        :param outlier_bounds: Half size of the box where the outliers are scattered, as a scalar
            or a `np.array (3,)`. When `None`, the box is the bounding box of the shape enlarged
            by `OUTLIER_MARGIN`.

        :returns: Point cloud as a `np.array (N, 3)`, where `N` is `n_points + n_outliers`

        ---
        """

        center = self._as_point(center, "center")
        n_points = self._as_count(n_points, "n_points", minimum=1)

        pts = np.tile(center, (n_points, 1))

        return self._finish(pts, noise, n_outliers, outlier_bounds)

    def line(self, anchor, direction, length=10.0, n_points=200, noise=0.05, n_outliers=0, outlier_bounds=None):
        """
        Generate a cloud of points along a 3D line.

        :param anchor: Coordinate of the middle of the segment `np.array (3,)`.
        :param direction: Direction of the line `np.array (3,)`, which does not need to be unitary.
        :param length: Length of the segment where the points are sampled.
        :param n_points: Number of points on the line, without counting the outliers.
        :param noise: Standard deviation of the gaussian noise added to every coordinate.
        :param n_outliers: Number of points scattered around which do not belong to the shape.
        :param outlier_bounds: Half size of the box where the outliers are scattered, as a scalar
            or a `np.array (3,)`. When `None`, the box is the bounding box of the shape enlarged
            by `OUTLIER_MARGIN`.

        :returns: Point cloud as a `np.array (N, 3)`, where `N` is `n_points + n_outliers`

        ---
        """

        anchor = self._as_point(anchor, "anchor")
        direction = self._as_direction(direction, "direction")
        length = self._as_sizes(length, 1, "length")[0]
        n_points = self._as_count(n_points, "n_points", minimum=2)

        steps = self.rng.uniform(-length / 2, length / 2, n_points)
        pts = anchor + steps[:, np.newaxis] * direction

        return self._finish(pts, noise, n_outliers, outlier_bounds)

    def plane(self, center, normal, size=10.0, n_points=400, noise=0.02, n_outliers=0, outlier_bounds=None):
        """
        Generate a cloud of points on a rectangular patch of a plane.

        :param center: Center of the patch `np.array (3,)`.
        :param normal: Normal of the plane `np.array (3,)`, which does not need to be unitary.
        :param size: Size of the patch along its two directions, as a scalar or a `np.array (2,)`.
        :param n_points: Number of points on the plane, without counting the outliers.
        :param noise: Standard deviation of the gaussian noise added to every coordinate.
        :param n_outliers: Number of points scattered around which do not belong to the shape.
        :param outlier_bounds: Half size of the box where the outliers are scattered, as a scalar
            or a `np.array (3,)`. When `None`, the box is the bounding box of the shape enlarged
            by `OUTLIER_MARGIN`.

        :returns: Point cloud as a `np.array (N, 3)`, where `N` is `n_points + n_outliers`

        ---
        """

        center = self._as_point(center, "center")
        normal = self._as_direction(normal, "normal")
        size = self._as_sizes(size, 2, "size")
        n_points = self._as_count(n_points, "n_points", minimum=3)

        first_axis, second_axis = self._basis_from_axis(normal)
        first_steps = self.rng.uniform(-size[0] / 2, size[0] / 2, n_points)
        second_steps = self.rng.uniform(-size[1] / 2, size[1] / 2, n_points)
        pts = center + first_steps[:, np.newaxis] * first_axis + second_steps[:, np.newaxis] * second_axis

        return self._finish(pts, noise, n_outliers, outlier_bounds)

    def circle(self, center, axis, radius, n_points=300, noise=0.02, n_outliers=0, outlier_bounds=None):
        """
        Generate a cloud of points on the hull of a circle.

        :param center: Center of the circle `np.array (3,)`.
        :param axis: Normal of the plane which contains the circle `np.array (3,)`, which does not
            need to be unitary.
        :param radius: Radius of the circle.
        :param n_points: Number of points on the circle, without counting the outliers.
        :param noise: Standard deviation of the gaussian noise added to every coordinate.
        :param n_outliers: Number of points scattered around which do not belong to the shape.
        :param outlier_bounds: Half size of the box where the outliers are scattered, as a scalar
            or a `np.array (3,)`. When `None`, the box is the bounding box of the shape enlarged
            by `OUTLIER_MARGIN`.

        :returns: Point cloud as a `np.array (N, 3)`, where `N` is `n_points + n_outliers`

        ---
        """

        center = self._as_point(center, "center")
        axis = self._as_direction(axis, "axis")
        radius = self._as_sizes(radius, 1, "radius")[0]
        n_points = self._as_count(n_points, "n_points", minimum=3)

        first_axis, second_axis = self._basis_from_axis(axis)
        angles = self.rng.uniform(0, 2 * np.pi, n_points)
        pts = center + radius * (
            np.cos(angles)[:, np.newaxis] * first_axis + np.sin(angles)[:, np.newaxis] * second_axis
        )

        return self._finish(pts, noise, n_outliers, outlier_bounds)

    def sphere(self, center, radius, n_points=500, noise=0.02, n_outliers=0, outlier_bounds=None):
        """
        Generate a cloud of points on the hull of a sphere.

        :param center: Center of the sphere `np.array (3,)`.
        :param radius: Radius of the sphere.
        :param n_points: Number of points on the sphere, without counting the outliers.
        :param noise: Standard deviation of the gaussian noise added to every coordinate.
        :param n_outliers: Number of points scattered around which do not belong to the shape.
        :param outlier_bounds: Half size of the box where the outliers are scattered, as a scalar
            or a `np.array (3,)`. When `None`, the box is the bounding box of the shape enlarged
            by `OUTLIER_MARGIN`.

        :returns: Point cloud as a `np.array (N, 3)`, where `N` is `n_points + n_outliers`

        ---
        """

        center = self._as_point(center, "center")
        radius = self._as_sizes(radius, 1, "radius")[0]
        n_points = self._as_count(n_points, "n_points", minimum=4)

        # Normalizing gaussian vectors spreads the directions uniformly over the sphere, which
        # sampling the two angles of the spherical coordinates would not do
        directions = self.rng.normal(size=(n_points, 3))
        directions = directions / np.linalg.norm(directions, axis=1)[:, np.newaxis]
        pts = center + radius * directions

        return self._finish(pts, noise, n_outliers, outlier_bounds)

    def cylinder(self, center, axis, radius, height=10.0, n_points=500, noise=0.02, n_outliers=0, outlier_bounds=None):
        """
        Generate a cloud of points on the lateral surface of a cylinder.

        The caps are not sampled, because the fitter looks for a cylinder of infinite height and
        the points on the caps would be outliers.

        :param center: Center of the cylinder `np.array (3,)`, which its axis passes through.
        :param axis: Axis of the cylinder `np.array (3,)`, which does not need to be unitary.
        :param radius: Radius of the cylinder.
        :param height: Height of the piece of cylinder where the points are sampled.
        :param n_points: Number of points on the cylinder, without counting the outliers.
        :param noise: Standard deviation of the gaussian noise added to every coordinate.
        :param n_outliers: Number of points scattered around which do not belong to the shape.
        :param outlier_bounds: Half size of the box where the outliers are scattered, as a scalar
            or a `np.array (3,)`. When `None`, the box is the bounding box of the shape enlarged
            by `OUTLIER_MARGIN`.

        :returns: Point cloud as a `np.array (N, 3)`, where `N` is `n_points + n_outliers`

        ---
        """

        center = self._as_point(center, "center")
        axis = self._as_direction(axis, "axis")
        radius = self._as_sizes(radius, 1, "radius")[0]
        height = self._as_sizes(height, 1, "height")[0]
        n_points = self._as_count(n_points, "n_points", minimum=3)

        first_axis, second_axis = self._basis_from_axis(axis)
        angles = self.rng.uniform(0, 2 * np.pi, n_points)
        heights = self.rng.uniform(-height / 2, height / 2, n_points)
        pts = (
            center
            + radius * (np.cos(angles)[:, np.newaxis] * first_axis + np.sin(angles)[:, np.newaxis] * second_axis)
            + heights[:, np.newaxis] * axis
        )

        return self._finish(pts, noise, n_outliers, outlier_bounds)

    def cuboid(self, center, extents, axes=None, n_points=900, noise=0.01, n_outliers=0, outlier_bounds=None):
        """
        Generate a cloud of points on the 6 faces of a cuboid.

        :param center: Center of the cuboid `np.array (3,)`.
        :param extents: Size of the cuboid along each one of its axes, as a scalar or a
            `np.array (3,)`.
        :param axes: Orthonormal axes of the cuboid, one per row, `np.array (3, 3)`. When `None`,
            the cuboid is aligned with the coordinate axes.
        :param n_points: Number of points on the cuboid, without counting the outliers. They are
            spread over the 6 faces proportionally to their area.
        :param noise: Standard deviation of the gaussian noise added to every coordinate.
        :param n_outliers: Number of points scattered around which do not belong to the shape.
        :param outlier_bounds: Half size of the box where the outliers are scattered, as a scalar
            or a `np.array (3,)`. When `None`, the box is the bounding box of the shape enlarged
            by `OUTLIER_MARGIN`.

        :returns: Point cloud as a `np.array (N, 3)`, where `N` is `n_points + n_outliers`

        ---
        """

        center = self._as_point(center, "center")
        extents = self._as_sizes(extents, 3, "extents")
        axes = self._as_axes(axes)
        n_points = self._as_count(n_points, "n_points", minimum=6)

        # A face is as likely to be sampled as its share of the total area of the cuboid, so the
        # points are spread evenly over the surface instead of piling up on the smallest faces
        areas = np.asarray([extents[1] * extents[2], extents[0] * extents[2], extents[0] * extents[1]])
        face_axis = self.rng.choice(3, size=n_points, p=areas / np.sum(areas))
        face_side = self.rng.choice([-1.0, 1.0], size=n_points)

        # Two coordinates are free inside the face and the third one is pinned on one of the two
        # faces orthogonal to it
        local = self.rng.uniform(-0.5, 0.5, size=(n_points, 3)) * extents
        local[np.arange(n_points), face_axis] = face_side * extents[face_axis] / 2

        # The rows of axes are the axes of the cuboid written in the frame of the point cloud
        pts = center + local.dot(axes)

        return self._finish(pts, noise, n_outliers, outlier_bounds)

    def _finish(self, pts, noise, n_outliers, outlier_bounds):
        """
        Turn the points of an ideal shape into a realistic point cloud.

        :param pts: Points sampled on the surface of the shape `np.array (M,3)`.
        :param noise: Standard deviation of the gaussian noise added to every coordinate.
        :param n_outliers: Number of points scattered around which do not belong to the shape.
        :param outlier_bounds: Half size of the box where the outliers are scattered.

        :returns: Noisy and shuffled point cloud with the outliers included `np.array (N, 3)`

        ---
        """

        pts = self._add_noise(pts, noise)
        pts = self._add_outliers(pts, n_outliers, outlier_bounds)

        # Shuffling mixes the outliers with the rest of the cloud, so the order of the points
        # never tells which ones belong to the shape
        return self.rng.permutation(pts)

    def _add_noise(self, pts, noise):
        """
        Scatter the points around the surface of the shape.

        :param pts: Points sampled on the surface of the shape `np.array (M,3)`.
        :param noise: Standard deviation of the gaussian noise added to every coordinate.

        :returns: Points moved by the noise `np.array (M, 3)`

        ---
        """

        noise = float(noise)
        if noise < 0:
            raise ValueError("noise must not be negative!")
        if noise == 0:
            return pts

        return pts + self.rng.normal(0.0, noise, size=pts.shape)

    def _add_outliers(self, pts, n_outliers, outlier_bounds):
        """
        Scatter points which do not belong to the shape around it.

        :param pts: Points of the shape `np.array (M,3)`.
        :param n_outliers: Number of outliers to add.
        :param outlier_bounds: Half size of the box where the outliers are scattered, as a scalar
            or a `np.array (3,)`. When `None`, the box is the bounding box of the shape enlarged
            by `OUTLIER_MARGIN`.

        :returns: Points of the shape with the outliers appended `np.array (M + n_outliers, 3)`

        ---
        """

        n_outliers = self._as_count(n_outliers, "n_outliers", minimum=0)
        if n_outliers == 0:
            return pts

        lower = np.amin(pts, axis=0)
        upper = np.amax(pts, axis=0)
        box_center = (lower + upper) / 2

        if outlier_bounds is None:
            # The biggest side of the bounding box gives a single half size for the 3 directions,
            # which keeps the outliers away from flat shapes like a plane or a circle too
            half_size = OUTLIER_MARGIN * np.amax(upper - lower) / 2
            half_size = np.full(3, half_size if half_size > 0 else 1.0)
        else:
            half_size = self._as_sizes(outlier_bounds, 3, "outlier_bounds")

        outliers = self.rng.uniform(box_center - half_size, box_center + half_size, size=(n_outliers, 3))

        return np.vstack((pts, outliers))

    @staticmethod
    def _basis_from_axis(axis):
        """
        Build the two directions which are orthogonal to an axis and to each other.

        :param axis: Unitary axis `np.array (3,)`.

        :returns: The two unitary directions which complete the basis, both `np.array (3,)`

        ---
        """

        # Crossing the axis with the coordinate direction it is the least aligned with is what
        # keeps the result far from a zero vector
        helper = np.zeros(3)
        helper[np.argmin(np.abs(axis))] = 1.0

        first_axis = np.cross(axis, helper)
        first_axis = first_axis / np.linalg.norm(first_axis)
        second_axis = np.cross(axis, first_axis)

        return first_axis, second_axis

    @staticmethod
    def _as_point(value, name):
        """
        Validate a 3D coordinate given by the caller.

        :param value: Coordinate to validate.
        :param name: Name of the parameter, used on the error message.

        :returns: The coordinate as a `np.array (3,)` of floats

        ---
        """

        point = np.asarray(value, dtype=float).reshape(-1)
        if point.shape != (3,):
            raise ValueError(f"{name} must be a 3D coordinate as a np.array (3,)!")
        if not np.all(np.isfinite(point)):
            raise ValueError(f"{name} must be finite!")

        return point

    @staticmethod
    def _as_direction(value, name):
        """
        Validate a direction given by the caller and make it unitary.

        :param value: Direction to validate, which does not need to be unitary.
        :param name: Name of the parameter, used on the error message.

        :returns: The direction as a unitary `np.array (3,)`

        ---
        """

        direction = ShapeGenerator._as_point(value, name)
        norm = np.linalg.norm(direction)
        if norm == 0:
            raise ValueError(f"{name} must not be a zero vector!")

        return direction / norm

    @staticmethod
    def _as_sizes(value, n_sizes, name):
        """
        Validate the dimensions of a shape given by the caller.

        :param value: Dimensions to validate, as a scalar which is used for all of them or as an
            array with one value per dimension.
        :param n_sizes: How many dimensions the shape has.
        :param name: Name of the parameter, used on the error message.

        :returns: The dimensions as a `np.array (n_sizes,)` of floats

        ---
        """

        sizes = np.asarray(value, dtype=float).reshape(-1)
        if sizes.shape == (1,):
            sizes = np.repeat(sizes, n_sizes)
        if sizes.shape != (n_sizes,):
            raise ValueError(f"{name} must be a scalar or a np.array ({n_sizes},)!")
        if not np.all(np.isfinite(sizes)) or np.any(sizes <= 0):
            raise ValueError(f"{name} must be finite and bigger than zero!")

        return sizes

    @staticmethod
    def _as_count(value, name, minimum):
        """
        Validate a number of points given by the caller.

        :param value: Number of points to validate.
        :param name: Name of the parameter, used on the error message.
        :param minimum: Smallest number of points which is accepted.

        :returns: The number of points as an `int`

        ---
        """

        count = int(value)
        if count != value or count < minimum:
            raise ValueError(f"{name} must be an integer of at least {minimum}!")

        return count

    @staticmethod
    def _as_axes(value):
        """
        Validate the axes of a cuboid given by the caller.

        :param value: Orthonormal axes to validate, one per row, or `None` to use the coordinate
            axes.

        :returns: The axes as a `np.array (3, 3)` of floats

        ---
        """

        if value is None:
            return np.eye(3)

        axes = np.asarray(value, dtype=float)
        if axes.shape != (3, 3):
            raise ValueError("axes must be a np.array (3, 3)!")
        if not np.allclose(axes.dot(axes.T), np.eye(3), atol=1e-6):
            raise ValueError("axes must be orthonormal!")

        return axes
