import random

import numpy as np
import pytest

import pyransac3d as pyrsc

# Each fitter samples a fixed number of points on every iteration, so this is the smallest cloud
# it is able to work with
MINIMUM_POINTS = [
    (pyrsc.Point, 1),
    (pyrsc.Line, 2),
    (pyrsc.Plane, 3),
    (pyrsc.Circle, 3),
    (pyrsc.Cylinder, 3),
    (pyrsc.Sphere, 4),
    (pyrsc.Cuboid, 6),
]


@pytest.mark.parametrize("shape_class, minimum", MINIMUM_POINTS)
def test_fit_rejects_a_cloud_smaller_than_the_sample(shape_class, minimum):
    points = np.zeros((minimum - 1, 3))

    with pytest.raises(ValueError, match=f"at least {minimum} point"):
        shape_class().fit(points, maxIteration=1)


@pytest.mark.parametrize("shape_class, minimum", MINIMUM_POINTS)
def test_fit_accepts_a_cloud_with_exactly_the_sample_size(shape_class, minimum):
    # Seeding both generators makes the cloud and the samples taken by RANSAC the same on every run
    random.seed(0)

    # Points taken from a gaussian are never collinear or coplanar in practice, so every fitter
    # gets a cloud it can sample without falling on a degenerate candidate
    points = np.random.default_rng(0).normal(size=(minimum, 3))

    shape_class().fit(points, maxIteration=1)
