import numpy as np
from numpy.typing import ArrayLike, NDArray


def get_rotationMatrix_from_vectors(u: ArrayLike, v: ArrayLike) -> NDArray[np.float64]:
    """
    Create a rotation matrix that rotates the space from a 3D vector `u` to a 3D vector `v`

    :param u: Orign vector `np.array (1,3)`.
    :param v: Destiny vector `np.array (1,3)`.

    :returns: Rotation matrix `np.array (3, 3)`

    ---
    """

    # Lets find a vector which is ortogonal to both u and v
    w = np.cross(u, v)

    # This orthogonal vector w has some interesting proprieties
    # |w| = sin of the required rotation
    # dot product of w and goal_normal_plane is the cos of the angle
    c = np.dot(u, v)
    s = np.linalg.norm(w)

    # Now, we compute rotation matrix from rodrigues formula
    # https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
    # https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d

    # We calculate the skew symetric matrix of the ort_vec
    Sx = np.asarray([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    R = np.eye(3) + Sx + Sx.dot(Sx) * ((1 - c) / (s ** 2))
    return R


def convex_hull_2d(points: ArrayLike) -> NDArray[np.float64]:
    """
    Find the convex hull of a set of 2D points using Andrew's monotone chain algorithm.

    :param points: Set of 2D points `np.array (N,2)`.

    :returns: Hull vertices in counter-clockwise order as a closed polygon, which means the
        first vertex is repeated at the end `np.array (M+1, 2)`

    ---
    """

    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError("Points must be a np.array (N,2)!")

    # np.unique sorts the rows lexicographically (by x, then by y) and drops repeated points,
    # which is exactly the ordering the monotone chain needs
    pts = np.unique(points, axis=0)
    if pts.shape[0] < 3:
        raise ValueError("Convex hull needs at least 3 distinct points!")

    def turn(o: NDArray[np.float64], a: NDArray[np.float64], b: NDArray[np.float64]) -> np.float64:
        # Z component of the cross product (a-o) x (b-o). Positive means counter-clockwise
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    def build_chain(ordered_pts: NDArray[np.float64]) -> list[NDArray[np.float64]]:
        chain: list[NDArray[np.float64]] = []
        for p in ordered_pts:
            # Drop the last vertex while the turn it makes is not counter-clockwise.
            # Using <= also discards collinear vertices, keeping only real corners
            while len(chain) >= 2 and turn(chain[-2], chain[-1], p) <= 0:
                chain.pop()
            chain.append(p)
        return chain

    # The lower chain goes left to right and the upper chain comes back right to left.
    # Each one ends on the other's first vertex, so we drop both last vertices
    lower = build_chain(pts)
    upper = build_chain(pts[::-1])
    hull = np.asarray(lower[:-1] + upper[:-1])

    return np.vstack((hull, hull[0]))


def min_bounding_rect(
    hull_points: ArrayLike,
) -> tuple[np.float64, np.float64, np.float64, np.float64, NDArray[np.float64], NDArray[np.float64]]:
    """
    Find the minimum-area rectangle which encloses a 2D convex hull.

    The rectangle is aligned with one of the hull edges, so we only have to test the
    orientation of each edge and keep the one which gives the smallest area.

    :param hull_points: Convex hull as a closed polygon `np.array (N,2)`, as returned by
        `convex_hull_2d(.)`.

    :returns:
    - `angle`: Rotation of the rectangle in radians, inside the first quadrant `float`
    - `area`: Area of the rectangle `float`
    - `width`: Size of the rectangle along its first axis `float`
    - `height`: Size of the rectangle along its second axis `float`
    - `center`: Center of the rectangle `np.array (2,)`
    - `corners`: Corners of the rectangle `np.array (4, 2)`

    ---
    """

    hull_points = np.asarray(hull_points, dtype=float)
    if hull_points.ndim != 2 or hull_points.shape[1] != 2:
        raise ValueError("Hull points must be a np.array (N,2)!")
    if hull_points.shape[0] < 3:
        raise ValueError("Minimum bounding rectangle needs at least 3 hull points!")

    # A rectangle repeats itself every 90 degrees, so we fold the edge angles into the
    # first quadrant and keep only the unique ones to avoid testing the same rotation twice
    edges = np.diff(hull_points, axis=0)
    angles = np.unique(np.arctan2(edges[:, 1], edges[:, 0]) % (np.pi / 2))

    # Stack one rotation matrix per candidate angle as a (K, 2, 2) array
    cos_a = np.cos(angles)
    sin_a = np.sin(angles)
    rotations = np.empty((angles.shape[0], 2, 2))
    rotations[:, 0, 0] = cos_a
    rotations[:, 0, 1] = sin_a
    rotations[:, 1, 0] = -sin_a
    rotations[:, 1, 1] = cos_a

    # Rotate the hull by every candidate angle at once: (K, 2, 2) x (2, N) = (K, 2, N)
    rot_points = rotations @ hull_points.T
    min_xy = np.amin(rot_points, axis=2)
    max_xy = np.amax(rot_points, axis=2)
    sizes = max_xy - min_xy
    areas = sizes[:, 0] * sizes[:, 1]

    best = int(np.argmin(areas))
    rot_best = rotations[best]
    min_x, min_y = min_xy[best]
    max_x, max_y = max_xy[best]

    # Multiplying a row vector by the rotation matrix applies its transpose, which brings
    # the rectangle from the rotated frame back to the original one
    center = ((min_xy[best] + max_xy[best]) / 2).dot(rot_best)
    corners = np.asarray([[max_x, min_y], [min_x, min_y], [min_x, max_y], [max_x, max_y]]).dot(rot_best)

    return angles[best], areas[best], sizes[best, 0], sizes[best, 1], center, corners


def rodrigues_rot(P: ArrayLike, n0: ArrayLike, n1: ArrayLike) -> NDArray[np.float64]:
    """
    Rotate a set of point between two normal vectors using Rodrigues' formula.

    :param P: Set of points `np.array (N,3)`.
    :param n0: Orign vector `np.array (1,3)`.
    :param n1: Destiny vector `np.array (1,3)`.

    :returns: Set of points P, but rotated `np.array (N, 3)`

    ---
    """

    # If P is only 1d array (coords of single point), fix it to be matrix
    P = np.asarray(P)
    if P.ndim == 1:
        P = P[np.newaxis, :]

    # Get vector of rotation k and angle theta
    n0 = n0 / np.linalg.norm(n0)
    n1 = n1 / np.linalg.norm(n1)
    k = np.cross(n0, n1)
    P_rot = np.zeros((len(P), 3))
    if np.linalg.norm(k) != 0:
        k = k / np.linalg.norm(k)
        theta = np.arccos(np.dot(n0, n1))

        # Compute rotated points
        for i in range(len(P)):
            P_rot[i] = (
                P[i] * np.cos(theta) + np.cross(k, P[i]) * np.sin(theta) + k * np.dot(k, P[i]) * (1 - np.cos(theta))
            )
    else:
        P_rot = P
    return P_rot


def estimate_normals(pts: ArrayLike, k: int = 16) -> NDArray[np.float64]:
    """
    Estimate the normal of the surface on every point of a cloud.

    The normal of a point is taken from its `k` nearest neighbors: the direction in which they
    spread the least is the one leaving the surface, which is the eigenvector of the smallest
    eigenvalue of their covariance.

    The normals are not oriented, which means a normal may point inwards while the next one
    points outwards. That is enough for the fitters, which only use the direction of the normal,
    but it is not enough to render them.

    :param pts: 3D point cloud as a numpy array (N,3).
    :param k: Number of neighbors used on each point. It is clamped to the size of the cloud.

    :returns: Unit normal of the surface on each point `np.array (N, 3)`

    ---
    """

    pts = np.asarray(pts, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError("Points must be a np.array (N,3)!")

    n_points = pts.shape[0]
    if n_points < 3:
        raise ValueError("Estimating normals needs at least 3 points!")

    # A point is its own nearest neighbor, so k+1 of them are taken and k are left after it
    k = int(min(max(k, 2), n_points - 1))

    normals = np.empty((n_points, 3))
    squared = np.einsum("ij,ij->i", pts, pts)

    # The distances are computed in blocks, so the matrix of every point against every other one
    # is never held whole and the memory does not grow with the square of the cloud
    block = max(1, int(4e6 // n_points))
    for start in range(0, n_points, block):
        stop = min(start + block, n_points)
        chunk = pts[start:stop]

        # |a-b|^2 = |a|^2 + |b|^2 - 2ab, which avoids building the (block, N, 3) differences
        sq_dist = squared[np.newaxis, :] + squared[start:stop, np.newaxis] - 2.0 * (chunk @ pts.T)

        neighbor_ids = np.argpartition(sq_dist, k, axis=1)[:, : k + 1]
        neighbors = pts[neighbor_ids]

        centered = neighbors - neighbors.mean(axis=1, keepdims=True)
        covariances = np.einsum("bij,bik->bjk", centered, centered)

        # eigh sorts the eigenvalues in ascending order, so the first eigenvector is the one of
        # the smallest eigenvalue, which is the normal
        _, eigenvectors = np.linalg.eigh(covariances)
        normals[start:stop] = eigenvectors[:, :, 0]

    return normals
