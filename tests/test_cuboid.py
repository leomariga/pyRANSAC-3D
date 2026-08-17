import random

import numpy as np
import pytest

import pyransac3d as pyrsc

# The 8 corners of a box in its own frame, as the sign of half of each one of its extents
CORNER_SIGNS = np.asarray(
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


def _fit_generated_cuboid():
    """Fit a cuboid on a cloud generated from known parameters, the same way of the first test."""

    random.seed(0)
    generator = pyrsc.ShapeGenerator(seed=0)

    angle = np.radians(25.0)
    axes = np.asarray(
        [
            [np.cos(angle), np.sin(angle), 0.0],
            [-np.sin(angle), np.cos(angle), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    points = generator.cuboid([1.0, -1.0, 2.0], [4.0, 3.0, 2.0], axes=axes, n_points=900, noise=0.01)

    cuboid = pyrsc.Cuboid()
    cuboid.fit(points, thresh=0.05, maxIteration=2000)

    return points, cuboid


def test_cuboid_finds_the_generated_cuboid():
    # Seeding both generators makes the cloud and the samples taken by RANSAC the same on every run
    random.seed(0)
    generator = pyrsc.ShapeGenerator(seed=0)

    center = np.asarray([1.0, -1.0, 2.0])
    extents = np.asarray([4.0, 3.0, 2.0])
    angle = np.radians(25.0)
    axes = np.asarray(
        [
            [np.cos(angle), np.sin(angle), 0.0],
            [-np.sin(angle), np.cos(angle), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )

    # The box is measured from the bounding box of the inliers, and the 3 planes of a candidate are
    # infinite, so outliers far from the box but close to one of the planes would stretch it
    n_points = 900
    points = generator.cuboid(center, extents, axes=axes, n_points=n_points, noise=0.01)

    cuboid = pyrsc.Cuboid()
    fit_center, fit_extents, fit_axes, inliers = cuboid.fit(points, thresh=0.05, maxIteration=2000)

    assert np.linalg.norm(fit_center - center) < 0.25

    # A box looks the same after being rotated by 90 degrees around its own axes, so its axes come
    # in any order and pointing either way, and so do the extents measured along them
    np.testing.assert_allclose(np.sort(fit_extents), np.sort(extents), atol=0.3)
    alignment = np.abs(fit_axes.dot(axes.T))
    assert np.all(np.amax(alignment, axis=1) > 0.99)

    # The 3 planes of the best candidate are 3 faces which touch each other, so they hold about
    # half of the points of the cloud
    assert len(inliers) >= 0.3 * n_points

    corners = cuboid.get_corners()
    assert corners.shape == (8, 3)

    # Every corner of the box is half of each extent away from the center, along its own axes
    np.testing.assert_allclose(
        np.linalg.norm(corners - fit_center, axis=1),
        np.full(8, np.linalg.norm(extents) / 2),
        atol=0.3,
    )


def test_cuboid_has_no_box_before_being_fitted():
    cuboid = pyrsc.Cuboid()

    with pytest.raises(ValueError, match="not fitted yet"):
        cuboid.get_corners()

    with pytest.raises(ValueError, match="not fitted yet"):
        cuboid.get_transform()


def test_cuboid_transform_takes_the_box_frame_to_the_point_cloud_frame():
    _, cuboid = _fit_generated_cuboid()

    transform = cuboid.get_transform()
    assert transform.shape == (4, 4)
    np.testing.assert_array_equal(transform[3], [0, 0, 0, 1])

    # The rows of the axes are the axes of the box written in the frame of the point cloud, so
    # they are the columns of the rotation, which is a rotation and not a reflection
    rotation = transform[0:3, 0:3]
    np.testing.assert_allclose(rotation, np.asarray(cuboid.axes).T)
    np.testing.assert_allclose(rotation.dot(rotation.T), np.eye(3), atol=1e-9)
    assert np.linalg.det(rotation) > 0

    np.testing.assert_array_equal(transform[0:3, 3], cuboid.center)

    # Taking the corners of the box from its own frame with the transform gives the same corners
    corners = CORNER_SIGNS * (cuboid.extents / 2)
    corners = corners.dot(rotation.T) + transform[0:3, 3]
    np.testing.assert_allclose(corners, cuboid.get_corners())


def test_cuboid_keeps_the_distances_of_the_best_candidate():
    points, cuboid = _fit_generated_cuboid()

    equation = np.asarray(cuboid.equation)
    assert equation.shape == (3, 4)

    # The 3 planes of a cuboid are orthogonal to each other and their normals are unitary
    normals = equation[:, 0:3]
    np.testing.assert_allclose(normals.dot(normals.T), np.eye(3), atol=1e-9)

    # There is one distance per plane and per point, and the one which is kept is the distance to
    # the closest plane
    expected = np.abs(points.dot(normals.T) + equation[:, 3]).T
    assert cuboid.plane_distances.shape == (3, points.shape[0])
    np.testing.assert_allclose(cuboid.plane_distances, expected, atol=1e-12)
    np.testing.assert_allclose(cuboid.distances, np.amin(expected, axis=0), atol=1e-12)

    np.testing.assert_array_equal(cuboid.inliers, np.where(cuboid.distances <= 0.05)[0])


def test_cuboid_keeps_the_measurements_used_to_build_the_box():
    _, cuboid = _fit_generated_cuboid()

    np.testing.assert_allclose(cuboid.face_normals, np.asarray(cuboid.equation)[:, 0:3])

    # Each inlier belongs to the plane it is closest to, and the face with more inliers is the one
    # which gives the reference axis of the box
    assert cuboid.face_inlier_count.shape == (3,)
    assert np.sum(cuboid.face_inlier_count) == len(cuboid.inliers)
    assert cuboid.ref_face_index == np.argmax(cuboid.face_inlier_count)
    np.testing.assert_allclose(cuboid.axes[2], cuboid.face_normals[cuboid.ref_face_index])

    # The extent along the reference axis is how much the inliers span on it
    assert cuboid.z_bounds.shape == (2,)
    assert cuboid.z_bounds[0] < cuboid.z_bounds[1]
    np.testing.assert_allclose(cuboid.extents[2], cuboid.z_bounds[1] - cuboid.z_bounds[0])

    # The rectangle which encloses the projected inliers repeats itself every 90 degrees
    assert 0 <= cuboid.hull_angle < np.pi / 2
