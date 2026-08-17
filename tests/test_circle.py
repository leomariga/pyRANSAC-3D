import random

import numpy as np

import pyransac3d as pyrsc


def test_circle_finds_the_generated_circle() -> None:
    # Seeding both generators makes the cloud and the samples taken by RANSAC the same on every run
    random.seed(0)
    generator = pyrsc.ShapeGenerator(seed=0)

    center = np.asarray([1.0, -1.0, 2.0])
    axis = np.asarray([0.0, 1.0, 1.0]) / np.linalg.norm([0.0, 1.0, 1.0])
    radius = 3.0
    n_points = 300
    points = generator.circle(center, axis, radius, n_points=n_points, noise=0.02, n_outliers=150)

    circle = pyrsc.Circle()
    fit_center, fit_axis, fit_radius, inliers = circle.fit(points, thresh=0.15, maxIteration=500)

    assert np.linalg.norm(np.asarray(fit_center) - center) < 0.25
    assert abs(fit_radius - radius) < 0.15

    # The circle lies on the same plane when its normal points the other way around
    assert abs(np.dot(fit_axis, axis)) > 0.99

    assert len(inliers) >= 0.95 * n_points


def test_circle_keeps_the_distances_of_the_best_candidate() -> None:
    random.seed(0)
    generator = pyrsc.ShapeGenerator(seed=0)

    points = generator.circle([1.0, -1.0, 2.0], [0.0, 1.0, 1.0], 3.0, n_points=200, noise=0.02, n_outliers=100)

    circle = pyrsc.Circle()
    fit_center, fit_axis, fit_radius, inliers = circle.fit(points, thresh=0.15, maxIteration=200)

    # The object keeps what fit(.) returned
    np.testing.assert_array_equal(circle.center, fit_center)
    np.testing.assert_array_equal(circle.axis, fit_axis)
    assert circle.radius == fit_radius
    np.testing.assert_array_equal(circle.inliers, inliers)

    # The first 3 terms of the plane equation are the axis of the circle
    plane_equation = np.asarray(circle.plane_equation)
    np.testing.assert_allclose(plane_equation[0:3], fit_axis)

    # The axis is unitary, so putting each point on the plane equation gives its signed distance
    # to the plane, and the cross product with the axis gives its distance to the center axis
    expected_plane = points.dot(plane_equation[0:3]) + plane_equation[3]
    expected_radial = np.linalg.norm(np.cross(fit_axis, fit_center - points), axis=1) - fit_radius
    np.testing.assert_allclose(circle.plane_distances, expected_plane)
    np.testing.assert_allclose(circle.radial_distances, expected_radial)

    # The distance to the hull of the circle is the hypotenuse of the other two
    np.testing.assert_allclose(circle.distances, np.hypot(expected_radial, expected_plane))

    np.testing.assert_array_equal(inliers, np.where(circle.distances <= 0.15)[0])
