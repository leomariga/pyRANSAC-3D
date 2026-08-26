import random

import numpy as np
import pytest

import pyransac3d as pyrsc


def cone_normals(pts: np.ndarray, apex: np.ndarray, axis: np.ndarray, angle: float) -> np.ndarray:
    """
    Normal of the surface of a cone on each point, which is what a sensor would measure.

    The cone is the surface where `dot(x - apex, axis) = |x - apex| * cos(angle)`, so its
    gradient, and with it the normal, is `cos(angle) * u - axis`, where `u` points from the apex
    to the point.
    """

    directions = pts - apex
    directions = directions / np.linalg.norm(directions, axis=1)[:, np.newaxis]
    normals = np.cos(angle) * directions - axis
    return normals / np.linalg.norm(normals, axis=1)[:, np.newaxis]


def test_cone_finds_the_generated_cone() -> None:
    # Seeding both generators makes the cloud and the samples taken by RANSAC the same on every run
    random.seed(0)
    generator = pyrsc.ShapeGenerator(seed=0)

    apex = np.asarray([1.0, -2.0, 0.5])
    axis = np.asarray([0.3, 0.2, 1.0]) / np.linalg.norm([0.3, 0.2, 1.0])
    angle = np.deg2rad(25.0)
    n_points = 600
    points = generator.cone(apex, axis, angle, height=6.0, n_points=n_points, noise=0.01, n_outliers=200)

    cone = pyrsc.Cone()
    fit_apex, fit_axis, fit_angle, inliers = cone.fit(points, thresh=0.08, maxIteration=800)

    # Unlike the cylinder, the apex of a cone is a single point, so it is wrong wherever it lands
    assert np.linalg.norm(np.asarray(fit_apex) - apex) < 0.3

    # The axis of a cone points from the apex towards the opening, so it cannot be the other way around
    assert np.dot(fit_axis, axis) > 0.99
    assert abs(fit_angle - angle) < np.deg2rad(2.0)

    assert len(inliers) >= 0.8 * n_points


def test_cone_uses_the_normals_it_is_given() -> None:
    random.seed(0)
    generator = pyrsc.ShapeGenerator(seed=0)

    apex = np.asarray([0.0, 0.0, 0.0])
    axis = np.asarray([0.0, 0.0, 1.0])
    angle = np.deg2rad(30.0)
    points = generator.cone(apex, axis, angle, height=5.0, n_points=400, noise=0.01)

    cone = pyrsc.Cone()
    fit_apex, fit_axis, fit_angle, inliers = cone.fit(
        points, thresh=0.08, maxIteration=400, normals=cone_normals(points, apex, axis, angle)
    )

    assert np.linalg.norm(np.asarray(fit_apex) - apex) < 0.3
    assert np.dot(fit_axis, axis) > 0.99
    assert abs(fit_angle - angle) < np.deg2rad(2.0)
    assert len(inliers) >= 0.8 * 400


def test_cone_keeps_the_distances_of_the_best_candidate() -> None:
    random.seed(0)
    generator = pyrsc.ShapeGenerator(seed=0)

    points = generator.cone([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], np.deg2rad(30.0), height=5.0, n_points=300, noise=0.01)

    cone = pyrsc.Cone()
    fit_apex, fit_axis, fit_angle, inliers = cone.fit(points, thresh=0.08, maxIteration=400)

    # The object keeps what fit(.) returned
    np.testing.assert_array_equal(cone.apex, fit_apex)
    np.testing.assert_array_equal(cone.axis, fit_axis)
    assert cone.angle == fit_angle
    np.testing.assert_array_equal(cone.inliers, inliers)

    # A point seen from the apex under the angle alpha is d * sin(alpha - angle) away from the surface
    to_points = points - np.asarray(fit_apex)
    lengths = np.linalg.norm(to_points, axis=1)
    alpha = np.arccos(np.clip(np.dot(to_points, fit_axis) / lengths, -1.0, 1.0))
    expected = np.abs(lengths * np.sin(alpha - fit_angle))

    assert cone.distances.shape == (points.shape[0],)

    # A point which lands on the apex is on the surface whatever the angle is, and the fitter
    # says so instead of normalizing a zero vector, so the two only agree to the floating point
    np.testing.assert_allclose(cone.distances, expected, atol=1e-12)
    np.testing.assert_array_equal(inliers, np.where(cone.distances <= 0.08)[0])


def test_cone_stops_on_the_callback() -> None:
    random.seed(0)
    generator = pyrsc.ShapeGenerator(seed=0)
    points = generator.cone([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], np.deg2rad(30.0), height=5.0, n_points=200, noise=0.01)

    seen = []

    def stop_after_five(state: dict) -> bool:
        seen.append(state["iteration"])
        return len(seen) == 5

    pyrsc.Cone().fit(points, thresh=0.08, maxIteration=400, callback=stop_after_five)

    assert len(seen) == 5


def test_cone_needs_at_least_three_points() -> None:
    with pytest.raises(ValueError):
        pyrsc.Cone().fit(np.zeros((2, 3)))


def test_cone_rejects_normals_which_do_not_match_the_cloud() -> None:
    points = np.random.default_rng(0).uniform(size=(20, 3))

    with pytest.raises(ValueError):
        pyrsc.Cone().fit(points, normals=np.ones((10, 3)))


def test_cone_puts_the_apex_on_the_surface() -> None:
    # The apex belongs to the cone for any angle, and measuring it means normalizing a zero
    # vector, so it is the one point which has to be answered without dividing by its length
    cone = pyrsc.Cone()
    apex = np.asarray([1.0, 2.0, 3.0])
    distances = cone._distance_to_cone(np.asarray([apex]), apex, np.asarray([0.0, 0.0, 1.0]), np.deg2rad(30.0))

    assert distances.shape == (1,)
    assert distances[0] == 0.0
