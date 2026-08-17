import random

import numpy as np

import pyransac3d as pyrsc


def test_point_finds_the_generated_center() -> None:
    # Seeding both generators makes the cloud and the samples taken by RANSAC the same on every run
    random.seed(0)
    generator = pyrsc.ShapeGenerator(seed=0)

    center = np.asarray([1.0, -2.0, 3.0])
    n_points = 300
    points = generator.point(center, n_points=n_points, noise=0.05, n_outliers=200, outlier_bounds=4.0)

    point = pyrsc.Point()
    fit_center, inliers = point.fit(points, thresh=0.25, maxIteration=200)

    # The center is one of the points of the cluster, so it is as far from the real center as the
    # noise took it
    assert np.linalg.norm(np.asarray(fit_center) - center) < 0.25
    assert len(inliers) >= n_points


def test_point_keeps_the_distances_of_the_best_candidate() -> None:
    random.seed(0)
    generator = pyrsc.ShapeGenerator(seed=0)

    points = generator.point([1.0, -2.0, 3.0], n_points=150, noise=0.05, n_outliers=100, outlier_bounds=4.0)

    point = pyrsc.Point()
    fit_center, inliers = point.fit(points, thresh=0.25, maxIteration=100)

    # The object keeps what fit(.) returned
    np.testing.assert_array_equal(point.center, fit_center)
    np.testing.assert_array_equal(point.inliers, inliers)

    # The center is one of the points of the cloud, so its own distance is zero
    expected = np.linalg.norm(fit_center - points, axis=1)
    assert point.distances.shape == (points.shape[0],)
    np.testing.assert_allclose(point.distances, expected)
    assert np.amin(point.distances) == 0

    np.testing.assert_array_equal(inliers, np.where(point.distances <= 0.25)[0])
