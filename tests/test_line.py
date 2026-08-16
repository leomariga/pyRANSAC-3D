import random

import numpy as np

import pyransac3d as pyrsc


def test_line_finds_the_generated_line():
    # Seeding both generators makes the cloud and the samples taken by RANSAC the same on every run
    random.seed(0)
    generator = pyrsc.ShapeGenerator(seed=0)

    anchor = np.asarray([1.0, 2.0, -1.0])
    direction = np.asarray([1.0, 1.0, 0.5]) / np.linalg.norm([1.0, 1.0, 0.5])
    n_points = 300
    points = generator.line(anchor, direction, length=10.0, n_points=n_points, noise=0.05, n_outliers=150)

    line = pyrsc.Line()
    fit_direction, fit_point, inliers = line.fit(points, thresh=0.25, maxIteration=300)

    # A line is the same line when its direction points the other way around
    assert abs(np.dot(fit_direction, direction)) > 0.999

    # Any point of the line is a valid answer, so the only thing to check is that it really is on
    # the line, which is the part of the vector to the anchor which is not along the direction
    to_anchor = np.asarray(fit_point) - anchor
    assert np.linalg.norm(to_anchor - np.dot(to_anchor, direction) * direction) < 0.25

    assert len(inliers) >= 0.95 * n_points
