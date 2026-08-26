import numpy as np
import pytest
from numpy.typing import NDArray

import pyransac3d as pyrsc


def _signed_area(polygon: NDArray[np.float64]) -> np.float64:
    """Area of a closed polygon, which is positive when its vertices are counter-clockwise."""

    x = polygon[:, 0]
    y = polygon[:, 1]

    return 0.5 * np.sum(x[:-1] * y[1:] - x[1:] * y[:-1])


def _rectangle(width: float, height: float, angle: float, center: NDArray[np.float64]) -> NDArray[np.float64]:
    """Corners of a rectangle, rotated by an angle in radians and moved to a center."""

    corners = np.asarray(
        [
            [-width / 2, -height / 2],
            [width / 2, -height / 2],
            [width / 2, height / 2],
            [-width / 2, height / 2],
        ]
    )
    rotation = np.asarray([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])

    return corners.dot(rotation.T) + center


def test_rotation_matrix_takes_the_first_vector_to_the_second() -> None:
    u = np.asarray([1.0, 0.0, 0.0])
    v = np.asarray([0.0, 1.0, 1.0]) / np.sqrt(2)

    rotation = pyrsc.get_rotationMatrix_from_vectors(u, v)

    np.testing.assert_allclose(rotation.dot(u), v, atol=1e-12)

    # A rotation keeps the lengths and the angles, so it is orthonormal and does not mirror space
    np.testing.assert_allclose(rotation.dot(rotation.T), np.eye(3), atol=1e-12)
    np.testing.assert_allclose(np.linalg.det(rotation), 1.0)


def test_rotation_matrix_turns_around_the_axis_shared_by_both_vectors() -> None:
    u = np.asarray([1.0, 2.0, 3.0]) / np.linalg.norm([1.0, 2.0, 3.0])
    v = np.asarray([-2.0, 1.0, 0.5]) / np.linalg.norm([-2.0, 1.0, 0.5])

    rotation = pyrsc.get_rotationMatrix_from_vectors(u, v)

    np.testing.assert_allclose(rotation.dot(u), v, atol=1e-12)

    # The rotation happens around the vector which is orthogonal to both, so that one stays put
    axis = np.cross(u, v)
    axis = axis / np.linalg.norm(axis)
    np.testing.assert_allclose(rotation.dot(axis), axis, atol=1e-12)


def test_convex_hull_keeps_only_the_corners_of_the_cloud() -> None:
    # A square with a point in the middle of each edge and one inside, so only the 4 corners of
    # the square are real corners of the hull
    points = np.asarray(
        [
            [-1.0, -1.0],
            [1.0, -1.0],
            [1.0, 1.0],
            [-1.0, 1.0],
            [0.0, -1.0],
            [1.0, 0.0],
            [0.0, 1.0],
            [-1.0, 0.0],
            [0.2, 0.3],
        ]
    )

    hull = pyrsc.convex_hull_2d(points)

    # The polygon is closed, so the first vertex is repeated at the end
    assert hull.shape == (5, 2)
    np.testing.assert_array_equal(hull[0], hull[-1])

    np.testing.assert_array_equal(np.unique(hull, axis=0), np.unique(points[0:4], axis=0))

    # The vertices come counter-clockwise, and they enclose the whole square
    assert _signed_area(hull) == pytest.approx(4.0)


def test_convex_hull_ignores_repeated_points() -> None:
    points = np.asarray([[0.0, 0.0], [2.0, 0.0], [0.0, 2.0], [2.0, 0.0], [0.0, 0.0]])

    hull = pyrsc.convex_hull_2d(points)

    assert hull.shape == (4, 2)
    assert _signed_area(hull) == pytest.approx(2.0)


def test_convex_hull_rejects_points_which_are_not_2d() -> None:
    with pytest.raises(ValueError, match=r"np.array \(N,2\)"):
        pyrsc.convex_hull_2d(np.zeros((5, 3)))


def test_convex_hull_rejects_a_cloud_without_enough_distinct_points() -> None:
    with pytest.raises(ValueError, match="at least 3 distinct points"):
        pyrsc.convex_hull_2d(np.asarray([[0.0, 0.0], [1.0, 1.0], [1.0, 1.0]]))


def test_min_bounding_rect_of_an_axis_aligned_square() -> None:
    hull = pyrsc.convex_hull_2d(np.asarray([[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]]))

    angle, area, width, height, center, corners = pyrsc.min_bounding_rect(hull)

    assert angle == pytest.approx(0.0)
    assert area == pytest.approx(4.0)
    assert width == pytest.approx(2.0)
    assert height == pytest.approx(2.0)
    np.testing.assert_allclose(center, [0.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(np.unique(corners, axis=0), np.unique(hull[0:4], axis=0), atol=1e-12)


def test_min_bounding_rect_follows_the_edges_of_a_rotated_rectangle() -> None:
    angle = np.radians(30.0)
    center = np.asarray([2.0, -1.0])
    rectangle = _rectangle(4.0, 2.0, angle, center)

    # Points inside the rectangle do not change the smallest rectangle which encloses them all
    inside = center + np.asarray([[0.1, 0.2], [-0.3, 0.1], [0.5, -0.4]])
    hull = pyrsc.convex_hull_2d(np.vstack((rectangle, inside)))

    fit_angle, area, width, height, fit_center, corners = pyrsc.min_bounding_rect(hull)

    # The rectangle is aligned with one of its own edges, and it repeats itself every 90 degrees
    assert fit_angle == pytest.approx(angle % (np.pi / 2))
    assert area == pytest.approx(8.0)
    np.testing.assert_allclose(np.sort([width, height]), [2.0, 4.0])
    np.testing.assert_allclose(fit_center, center, atol=1e-12)

    # The corners are the corners of the rectangle, in any order
    assert corners.shape == (4, 2)
    distance_to_corners = np.linalg.norm(corners[:, np.newaxis, :] - rectangle[np.newaxis, :, :], axis=2)
    np.testing.assert_allclose(np.amin(distance_to_corners, axis=1), np.zeros(4), atol=1e-12)


def test_min_bounding_rect_rejects_hulls_which_are_not_2d() -> None:
    with pytest.raises(ValueError, match=r"np.array \(N,2\)"):
        pyrsc.min_bounding_rect(np.zeros((5, 3)))


def test_min_bounding_rect_rejects_a_hull_without_enough_points() -> None:
    with pytest.raises(ValueError, match="at least 3 hull points"):
        pyrsc.min_bounding_rect(np.asarray([[0.0, 0.0], [1.0, 1.0]]))


def test_rodrigues_rot_takes_the_points_from_one_normal_to_the_other() -> None:
    # Points on the plane z = 0, whose normal is Z
    points = np.asarray([[1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [-1.0, -1.0, 0.0]])

    rotated = pyrsc.rodrigues_rot(points, [0.0, 0.0, 1.0], [0.0, 1.0, 0.0])

    # Taking the normal to Y puts every point on the plane y = 0
    assert rotated.shape == points.shape
    np.testing.assert_allclose(rotated[:, 1], np.zeros(3), atol=1e-12)

    # A rotation keeps the distance from each point to the origin and between the points
    np.testing.assert_allclose(np.linalg.norm(rotated, axis=1), np.linalg.norm(points, axis=1))
    np.testing.assert_allclose(
        np.linalg.norm(rotated[1] - rotated[0]),
        np.linalg.norm(points[1] - points[0]),
    )


def test_rodrigues_rot_agrees_with_the_rotation_matrix() -> None:
    points = np.asarray([[1.0, 2.0, 3.0], [-2.0, 0.5, 1.0]])
    n0 = np.asarray([0.0, 0.0, 1.0])
    n1 = np.asarray([1.0, 1.0, 1.0]) / np.sqrt(3)

    rotated = pyrsc.rodrigues_rot(points, n0, n1)

    np.testing.assert_allclose(rotated, points.dot(pyrsc.get_rotationMatrix_from_vectors(n0, n1).T), atol=1e-12)


def test_rodrigues_rot_accepts_a_single_point_and_vectors_which_are_not_unitary() -> None:
    rotated = pyrsc.rodrigues_rot([1.0, 0.0, 0.0], [0.0, 0.0, 5.0], [0.0, 3.0, 0.0])

    # A single point comes back as a cloud with one point
    assert rotated.shape == (1, 3)
    np.testing.assert_allclose(rotated[0], [1.0, 0.0, 0.0], atol=1e-12)


def test_rodrigues_rot_does_nothing_when_both_normals_are_the_same() -> None:
    points = np.asarray([[1.0, 2.0, 3.0], [-2.0, 0.5, 1.0]])

    rotated = pyrsc.rodrigues_rot(points, [0.0, 0.0, 1.0], [0.0, 0.0, 1.0])

    np.testing.assert_array_equal(rotated, points)


def test_estimate_normals_finds_the_normal_of_a_plane() -> None:
    # A cloud this size is split in more than one block by the neighbor search, so the blocks
    # have to agree with each other
    rng = np.random.default_rng(0)
    normal = np.asarray([1.0, 2.0, -1.0]) / np.linalg.norm([1.0, 2.0, -1.0])
    first, second = pyrsc.ShapeGenerator._basis_from_axis(normal)
    uv = rng.uniform(-5, 5, (2500, 2))
    points = uv[:, 0:1] * first + uv[:, 1:2] * second

    normals = pyrsc.estimate_normals(points, k=12)

    assert normals.shape == points.shape
    np.testing.assert_allclose(np.linalg.norm(normals, axis=1), 1.0)

    # The normals are not oriented, so half of them may point to the other side of the plane
    np.testing.assert_allclose(np.abs(normals.dot(normal)), 1.0, atol=1e-6)


def test_estimate_normals_follows_the_surface_of_a_sphere() -> None:
    generator = pyrsc.ShapeGenerator(seed=0)
    center = np.asarray([1.0, -1.0, 2.0])
    points = generator.sphere(center, 4.0, n_points=800, noise=0.0)

    normals = pyrsc.estimate_normals(points, k=20)

    # The normal of a sphere is radial, so it is aligned with the direction to its center
    radial = points - center
    radial = radial / np.linalg.norm(radial, axis=1)[:, np.newaxis]
    alignment = np.abs(np.einsum("ij,ij->i", normals, radial))

    assert np.median(alignment) > 0.99


def test_estimate_normals_clamps_k_to_the_size_of_the_cloud() -> None:
    points = np.asarray([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 1.0, 0.0]])

    normals = pyrsc.estimate_normals(points, k=50)

    assert normals.shape == (4, 3)
    np.testing.assert_allclose(np.abs(normals.dot([0.0, 0.0, 1.0])), 1.0, atol=1e-9)


def test_estimate_normals_rejects_points_which_are_not_3d() -> None:
    with pytest.raises(ValueError):
        pyrsc.estimate_normals(np.zeros((10, 2)))


def test_estimate_normals_rejects_a_cloud_without_enough_points() -> None:
    with pytest.raises(ValueError):
        pyrsc.estimate_normals(np.zeros((2, 3)))
