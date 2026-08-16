import random

import numpy as np

import pyransac3d as pyrsc


def test_point_finds_the_generated_center():
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
