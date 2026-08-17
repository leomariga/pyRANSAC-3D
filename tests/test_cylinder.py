import random

import numpy as np

import pyransac3d as pyrsc


def test_cylinder_finds_the_generated_cylinder():
    # Seeding both generators makes the cloud and the samples taken by RANSAC the same on every run
    random.seed(0)
    generator = pyrsc.ShapeGenerator(seed=0)

    center = np.asarray([1.0, 1.0, 0.0])
    axis = np.asarray([0.2, 0.0, 1.0]) / np.linalg.norm([0.2, 0.0, 1.0])
    radius = 3.0
    n_points = 500
    points = generator.cylinder(center, axis, radius, height=4.0, n_points=n_points, noise=0.02, n_outliers=200)

    cylinder = pyrsc.Cylinder()
    fit_center, fit_axis, fit_radius, inliers = cylinder.fit(points, thresh=0.15, maxIteration=1500)

    # The cylinder points the same way when its axis points the other way around
    assert abs(np.dot(fit_axis, axis)) > 0.99
    assert abs(fit_radius - radius) < 0.15

    # The center can be anywhere along the axis, so it is only wrong when it is off the axis
    to_center = np.asarray(fit_center) - center
    assert np.linalg.norm(to_center - np.dot(to_center, axis) * axis) < 0.3

    assert len(inliers) >= 0.8 * n_points


def test_cylinder_keeps_the_distances_of_the_best_candidate():
    random.seed(0)
    generator = pyrsc.ShapeGenerator(seed=0)

    points = generator.cylinder([1.0, 1.0, 0.0], [0.2, 0.0, 1.0], 3.0, height=4.0, n_points=250, noise=0.02)

    cylinder = pyrsc.Cylinder()
    fit_center, fit_axis, fit_radius, inliers = cylinder.fit(points, thresh=0.15, maxIteration=500)

    # The object keeps what fit(.) returned
    np.testing.assert_array_equal(cylinder.center, fit_center)
    np.testing.assert_array_equal(cylinder.axis, fit_axis)
    assert cylinder.radius == fit_radius
    np.testing.assert_array_equal(cylinder.inliers, inliers)

    # The axis is unitary, so the norm of the cross product is the distance from each point to the
    # axis, and the distance to the hull is how much it is away from the radius
    expected_radial = np.linalg.norm(np.cross(fit_axis, fit_center - points), axis=1)
    assert cylinder.radial_distances.shape == (points.shape[0],)
    np.testing.assert_allclose(cylinder.radial_distances, expected_radial)
    np.testing.assert_allclose(cylinder.distances, np.abs(expected_radial - fit_radius))

    np.testing.assert_array_equal(inliers, np.where(cylinder.distances <= 0.15)[0])
