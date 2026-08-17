import random

import numpy as np
import pytest

import pyransac3d as pyrsc

# Every state given to the callback describes the iteration, the points it sampled, the candidate
# it built from them and the best candidate found so far
STATE_KEYS = {
    "iteration",
    "sample_indices",
    "sample_points",
    "model",
    "inliers",
    "best_model",
    "best_inliers",
    "is_best",
}


def _point_cloud(generator):
    return generator.point([1.0, -2.0, 3.0], n_points=80, noise=0.05, n_outliers=40, outlier_bounds=4.0)


def _line_cloud(generator):
    return generator.line([1.0, 2.0, -1.0], [1.0, 1.0, 0.5], length=10.0, n_points=80, noise=0.05, n_outliers=40)


def _plane_cloud(generator):
    return generator.plane([0.5, 1.0, -2.0], [1.0, 2.0, 3.0], size=8.0, n_points=80, noise=0.02, n_outliers=40)


def _circle_cloud(generator):
    return generator.circle([1.0, -1.0, 2.0], [0.0, 1.0, 1.0], 3.0, n_points=80, noise=0.02, n_outliers=40)


def _cylinder_cloud(generator):
    return generator.cylinder([1.0, 1.0, 0.0], [0.2, 0.0, 1.0], 3.0, height=4.0, n_points=80, noise=0.02)


def _sphere_cloud(generator):
    return generator.sphere([2.0, 1.0, -1.0], 3.0, n_points=80, noise=0.02, n_outliers=40)


def _cuboid_cloud(generator):
    return generator.cuboid([1.0, -1.0, 2.0], [4.0, 3.0, 2.0], n_points=120, noise=0.01)


# How many points each fitter samples, the keys of the candidate it builds from them and the keys
# of the best candidate it reports, which on the cuboid also carry the box it measured
FITTERS = [
    (pyrsc.Point, _point_cloud, 1, {"center"}, {"center"}),
    (pyrsc.Line, _line_cloud, 2, {"A", "B"}, {"A", "B"}),
    (pyrsc.Plane, _plane_cloud, 3, {"equation"}, {"equation"}),
    (pyrsc.Circle, _circle_cloud, 3, {"center", "axis", "radius"}, {"center", "axis", "radius"}),
    (pyrsc.Cylinder, _cylinder_cloud, 3, {"center", "axis", "radius"}, {"center", "axis", "radius"}),
    (pyrsc.Sphere, _sphere_cloud, 4, {"center", "radius"}, {"center", "radius"}),
    (pyrsc.Cuboid, _cuboid_cloud, 6, {"equation"}, {"equation", "center", "extents", "axes"}),
]

FITTER_IDS = [fitter[0].__name__ for fitter in FITTERS]


@pytest.mark.parametrize("shape_class, build_cloud, n_samples, model_keys, best_model_keys", FITTERS, ids=FITTER_IDS)
def test_callback_receives_the_state_of_every_iteration(
    shape_class, build_cloud, n_samples, model_keys, best_model_keys
):
    # Seeding both generators makes the cloud and the samples taken by RANSAC the same on every run
    random.seed(0)
    points = build_cloud(pyrsc.ShapeGenerator(seed=0))

    states = []
    max_iteration = 5
    shape_class().fit(points, maxIteration=max_iteration, callback=states.append)

    assert len(states) == max_iteration

    # The first iteration has nothing to beat, because the fitter starts without any inlier
    previous_best_count = 0

    for iteration, state in enumerate(states):
        assert set(state) == STATE_KEYS
        assert state["iteration"] == iteration

        # The state carries both the indices of the sampled points and the points themselves
        assert len(state["sample_indices"]) == n_samples
        assert state["sample_points"].shape == (n_samples, 3)
        np.testing.assert_array_equal(state["sample_points"], points[state["sample_indices"]])

        assert set(state["model"]) == model_keys
        assert set(state["best_model"]) == best_model_keys

        # The best candidate is the one with more inliers, so no iteration beats it. A tie is not
        # enough to replace it, which is why the candidate has to be strictly better
        assert len(state["best_inliers"]) >= len(state["inliers"])
        assert state["is_best"] == (len(state["inliers"]) > previous_best_count)
        if state["is_best"]:
            np.testing.assert_array_equal(state["best_inliers"], state["inliers"])
        previous_best_count = len(state["best_inliers"])

    # The number of inliers of the best candidate only grows
    best_counts = [len(state["best_inliers"]) for state in states]
    assert best_counts == sorted(best_counts)


@pytest.mark.parametrize("shape_class, build_cloud, n_samples, model_keys, best_model_keys", FITTERS, ids=FITTER_IDS)
def test_callback_stops_the_fitting_when_it_returns_true(
    shape_class, build_cloud, n_samples, model_keys, best_model_keys
):
    random.seed(0)
    points = build_cloud(pyrsc.ShapeGenerator(seed=0))

    states = []

    def stop_on_the_second_iteration(state):
        states.append(state)
        return state["iteration"] == 1

    fitter = shape_class()
    fitter.fit(points, maxIteration=500, callback=stop_on_the_second_iteration)

    # The fitting stopped long before the last iteration, keeping the best candidate it had
    assert len(states) == 2
    np.testing.assert_array_equal(fitter.inliers, states[-1]["best_inliers"])
