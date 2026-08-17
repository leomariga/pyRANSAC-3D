import random

import numpy as np

import pyransac3d as pyrsc


def test_plane_finds_the_generated_plane() -> None:
    # Seeding both generators makes the cloud and the samples taken by RANSAC the same on every run
    random.seed(0)
    generator = pyrsc.ShapeGenerator(seed=0)

    center = np.asarray([0.5, 1.0, -2.0])
    normal = np.asarray([1.0, 2.0, 3.0]) / np.linalg.norm([1.0, 2.0, 3.0])
    n_points = 400
    points = generator.plane(center, normal, size=8.0, n_points=n_points, noise=0.02, n_outliers=200)

    plane = pyrsc.Plane()
    equation, inliers = plane.fit(points, thresh=0.1, maxIteration=300)

    equation = np.asarray(equation)

    # A plane is the same plane when its equation is multiplied by -1
    assert abs(np.dot(equation[0:3], normal)) > 0.999

    # The center of the patch is on the plane, so putting it on the equation gives its distance to
    # the fitted plane, which is zero when both planes are the same
    assert abs(np.dot(equation[0:3], center) + equation[3]) < 0.1

    assert len(inliers) >= 0.95 * n_points


def test_plane_keeps_the_distances_of_the_best_candidate() -> None:
    random.seed(0)
    generator = pyrsc.ShapeGenerator(seed=0)

    points = generator.plane([0.5, 1.0, -2.0], [1.0, 2.0, 3.0], size=8.0, n_points=200, noise=0.02, n_outliers=100)

    plane = pyrsc.Plane()
    equation, inliers = plane.fit(points, thresh=0.1, maxIteration=100)

    # The object keeps what fit(.) returned
    np.testing.assert_array_equal(plane.equation, equation)
    np.testing.assert_array_equal(plane.inliers, inliers)

    # The normal of the equation is unitary, so the signed distance to the plane is what putting
    # each point on the equation gives
    equation = np.asarray(equation)
    assert plane.distances.shape == (points.shape[0],)
    np.testing.assert_allclose(plane.distances, points.dot(equation[0:3]) + equation[3])

    # The inliers are exactly the points which are closer to the plane than the threshold
    np.testing.assert_array_equal(inliers, np.where(np.abs(plane.distances) <= 0.1)[0])
