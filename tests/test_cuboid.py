import random

import numpy as np

import pyransac3d as pyrsc


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
