import random

import numpy as np

import pyransac3d as pyrsc


def test_circle_finds_the_generated_circle():
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
