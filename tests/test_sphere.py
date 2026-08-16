import random

import numpy as np

import pyransac3d as pyrsc


def test_sphere_finds_the_generated_sphere():
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
