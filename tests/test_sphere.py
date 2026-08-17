import random

import numpy as np

import pyransac3d as pyrsc


def test_sphere_finds_the_generated_sphere() -> None:
    # Seeding both generators makes the cloud and the samples taken by RANSAC the same on every run
    random.seed(0)
    generator = pyrsc.ShapeGenerator(seed=0)

    center = np.asarray([2.0, 1.0, -1.0])
    radius = 3.0
    n_points = 500
    points = generator.sphere(center, radius, n_points=n_points, noise=0.02, n_outliers=250)

    sphere = pyrsc.Sphere()
    fit_center, fit_radius, inliers = sphere.fit(points, thresh=0.15, maxIteration=300)

    assert np.linalg.norm(np.asarray(fit_center) - center) < 0.25
    assert abs(fit_radius - radius) < 0.15
    assert len(inliers) >= 0.95 * n_points


def test_sphere_keeps_the_distances_of_the_best_candidate() -> None:
    random.seed(0)
    generator = pyrsc.ShapeGenerator(seed=0)

    points = generator.sphere([2.0, 1.0, -1.0], 3.0, n_points=250, noise=0.02, n_outliers=120)

    sphere = pyrsc.Sphere()
    fit_center, fit_radius, inliers = sphere.fit(points, thresh=0.15, maxIteration=100)

    # The object keeps what fit(.) returned
    np.testing.assert_array_equal(sphere.center, fit_center)
    assert sphere.radius == fit_radius
    np.testing.assert_array_equal(sphere.inliers, inliers)

    # The radial distance is measured to the center and the other one to the hull, which is how
    # much the point is away from the radius
    expected_radial = np.linalg.norm(fit_center - points, axis=1)
    assert sphere.radial_distances.shape == (points.shape[0],)
    np.testing.assert_allclose(sphere.radial_distances, expected_radial)
    np.testing.assert_allclose(sphere.distances, np.abs(expected_radial - fit_radius))

    np.testing.assert_array_equal(inliers, np.where(sphere.distances <= 0.15)[0])
