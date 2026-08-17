import numpy as np
import pytest

import pyransac3d as pyrsc
from pyransac3d.shape_generator import OUTLIER_MARGIN

# One builder per shape, with the smallest number of points it can be sampled with, which is how
# many points the fitter of that shape needs
SHAPES = [
    ("point", lambda generator, **kwargs: generator.point([1.0, -2.0, 3.0], **kwargs), 1),
    ("line", lambda generator, **kwargs: generator.line([1.0, 2.0, -1.0], [1.0, 1.0, 0.5], **kwargs), 2),
    ("plane", lambda generator, **kwargs: generator.plane([0.5, 1.0, -2.0], [1.0, 2.0, 3.0], **kwargs), 3),
    ("circle", lambda generator, **kwargs: generator.circle([1.0, -1.0, 2.0], [0.0, 1.0, 1.0], 3.0, **kwargs), 3),
    ("cylinder", lambda generator, **kwargs: generator.cylinder([1.0, 1.0, 0.0], [0.2, 0.0, 1.0], 3.0, **kwargs), 3),
    ("sphere", lambda generator, **kwargs: generator.sphere([2.0, 1.0, -1.0], 3.0, **kwargs), 4),
    ("cuboid", lambda generator, **kwargs: generator.cuboid([1.0, -1.0, 2.0], [4.0, 3.0, 2.0], **kwargs), 6),
]

SHAPE_IDS = [shape[0] for shape in SHAPES]


@pytest.mark.parametrize("name, build_cloud, minimum", SHAPES, ids=SHAPE_IDS)
def test_generated_cloud_has_the_shape_points_plus_the_outliers(name, build_cloud, minimum):
    points = build_cloud(pyrsc.ShapeGenerator(seed=0), n_points=50, noise=0.0, n_outliers=20)

    assert points.shape == (70, 3)
    assert np.all(np.isfinite(points))


@pytest.mark.parametrize("name, build_cloud, minimum", SHAPES, ids=SHAPE_IDS)
def test_the_same_seed_generates_the_same_cloud(name, build_cloud, minimum):
    first = build_cloud(pyrsc.ShapeGenerator(seed=7), n_points=40, noise=0.05, n_outliers=10)
    same = build_cloud(pyrsc.ShapeGenerator(seed=7), n_points=40, noise=0.05, n_outliers=10)
    other = build_cloud(pyrsc.ShapeGenerator(seed=8), n_points=40, noise=0.05, n_outliers=10)

    np.testing.assert_array_equal(first, same)
    assert not np.array_equal(first, other)


@pytest.mark.parametrize("name, build_cloud, minimum", SHAPES, ids=SHAPE_IDS)
def test_generator_rejects_a_cloud_smaller_than_the_shape_needs(name, build_cloud, minimum):
    with pytest.raises(ValueError, match=f"n_points must be an integer of at least {minimum}"):
        build_cloud(pyrsc.ShapeGenerator(seed=0), n_points=minimum - 1)


@pytest.mark.parametrize("name, build_cloud, minimum", SHAPES, ids=SHAPE_IDS)
def test_generator_rejects_a_number_of_points_which_is_not_an_integer(name, build_cloud, minimum):
    with pytest.raises(ValueError, match="n_points must be an integer"):
        build_cloud(pyrsc.ShapeGenerator(seed=0), n_points=10.5)


def test_point_cloud_is_gathered_around_the_center():
    center = np.asarray([1.0, -2.0, 3.0])

    points = pyrsc.ShapeGenerator(seed=0).point(center, n_points=100, noise=0.0)

    np.testing.assert_array_equal(points, np.tile(center, (100, 1)))


def test_line_cloud_lies_on_the_line():
    anchor = np.asarray([1.0, 2.0, -1.0])
    direction = np.asarray([1.0, 1.0, 0.5]) / np.linalg.norm([1.0, 1.0, 0.5])
    length = 10.0

    points = pyrsc.ShapeGenerator(seed=0).line(anchor, [1.0, 1.0, 0.5], length=length, n_points=200, noise=0.0)

    # The part of the vector to the anchor which is not along the direction is the distance from
    # the point to the line
    to_anchor = points - anchor
    along = to_anchor.dot(direction)
    off_the_line = np.linalg.norm(to_anchor - along[:, np.newaxis] * direction, axis=1)
    np.testing.assert_allclose(off_the_line, np.zeros(200), atol=1e-12)

    # The anchor is the middle of the segment where the points are sampled
    assert np.all(np.abs(along) <= length / 2)
    assert np.amax(np.abs(along)) > 0.9 * length / 2


def test_plane_cloud_lies_on_the_plane():
    center = np.asarray([0.5, 1.0, -2.0])
    normal = np.asarray([1.0, 2.0, 3.0]) / np.linalg.norm([1.0, 2.0, 3.0])

    points = pyrsc.ShapeGenerator(seed=0).plane(center, [1.0, 2.0, 3.0], size=8.0, n_points=200, noise=0.0)

    np.testing.assert_allclose((points - center).dot(normal), np.zeros(200), atol=1e-12)

    # The patch is a square of the given size, so no point is farther than its diagonal
    assert np.amax(np.linalg.norm(points - center, axis=1)) <= 8.0 * np.sqrt(2) / 2


def test_plane_cloud_accepts_a_different_size_on_each_direction():
    center = np.asarray([0.0, 0.0, 0.0])

    points = pyrsc.ShapeGenerator(seed=0).plane(center, [0.0, 0.0, 1.0], size=[10.0, 2.0], n_points=500, noise=0.0)

    # The normal is Z, so the patch spreads over X and Y, one size on each one of them
    np.testing.assert_allclose(np.amax(points, axis=0), [1.0, 5.0, 0.0], atol=0.1)
    np.testing.assert_allclose(np.amin(points, axis=0), [-1.0, -5.0, 0.0], atol=0.1)


def test_circle_cloud_lies_on_the_hull_of_the_circle():
    center = np.asarray([1.0, -1.0, 2.0])
    axis = np.asarray([0.0, 1.0, 1.0]) / np.linalg.norm([0.0, 1.0, 1.0])
    radius = 3.0

    points = pyrsc.ShapeGenerator(seed=0).circle(center, [0.0, 1.0, 1.0], radius, n_points=200, noise=0.0)

    # Every point is on the plane of the circle and as far from the center as the radius
    np.testing.assert_allclose((points - center).dot(axis), np.zeros(200), atol=1e-12)
    np.testing.assert_allclose(np.linalg.norm(points - center, axis=1), np.full(200, radius))


def test_sphere_cloud_lies_on_the_hull_of_the_sphere():
    center = np.asarray([2.0, 1.0, -1.0])
    radius = 3.0

    points = pyrsc.ShapeGenerator(seed=0).sphere(center, radius, n_points=300, noise=0.0)

    np.testing.assert_allclose(np.linalg.norm(points - center, axis=1), np.full(300, radius))

    # The directions are spread over the whole sphere, and not piled up on one side of it
    np.testing.assert_allclose(np.mean(points - center, axis=0), np.zeros(3), atol=0.5)


def test_cylinder_cloud_lies_on_the_lateral_surface():
    center = np.asarray([1.0, 1.0, 0.0])
    axis = np.asarray([0.2, 0.0, 1.0]) / np.linalg.norm([0.2, 0.0, 1.0])
    radius = 3.0
    height = 4.0

    generator = pyrsc.ShapeGenerator(seed=0)
    points = generator.cylinder(center, [0.2, 0.0, 1.0], radius, height=height, n_points=300, noise=0.0)

    # The distance from each point to the axis is the radius, because the caps are not sampled
    np.testing.assert_allclose(np.linalg.norm(np.cross(axis, center - points), axis=1), np.full(300, radius))

    along = (points - center).dot(axis)
    assert np.all(np.abs(along) <= height / 2 + 1e-12)


def test_cuboid_cloud_lies_on_the_faces_of_the_box():
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

    points = pyrsc.ShapeGenerator(seed=0).cuboid(center, extents, axes=axes, n_points=600, noise=0.0)

    # The rows of the axes are the axes of the box, so this brings each point to the frame of the
    # box, where one of its coordinates is pinned on a face and the other two are inside of it
    local = (points - center).dot(axes.T)
    ratios = np.abs(local) / (extents / 2)
    np.testing.assert_allclose(np.amax(ratios, axis=1), np.ones(600))
    assert np.all(ratios <= 1 + 1e-12)


def test_cuboid_cloud_uses_the_coordinate_axes_and_a_scalar_size_by_default():
    points = pyrsc.ShapeGenerator(seed=0).cuboid([0.0, 0.0, 0.0], 2.0, n_points=600, noise=0.0)

    # A scalar gives a cube, which is aligned with the coordinate axes when no axes are given
    np.testing.assert_allclose(np.amax(points, axis=0), np.ones(3))
    np.testing.assert_allclose(np.amin(points, axis=0), -np.ones(3))


def test_noise_scatters_the_points_around_the_surface():
    center = np.asarray([2.0, 1.0, -1.0])
    radius = 3.0
    noise = 0.05

    points = pyrsc.ShapeGenerator(seed=0).sphere(center, radius, n_points=2000, noise=noise)

    # The noise is added to every coordinate, so the points are no longer exactly on the hull, but
    # they are still gathered around it
    deviation = np.linalg.norm(points - center, axis=1) - radius
    assert np.all(deviation != 0)
    assert 0.5 * noise < np.std(deviation) < 1.5 * noise


def test_noise_must_not_be_negative():
    with pytest.raises(ValueError, match="noise must not be negative"):
        pyrsc.ShapeGenerator(seed=0).sphere([0.0, 0.0, 0.0], 1.0, noise=-0.1)


def test_outliers_are_scattered_inside_the_default_box():
    center = np.asarray([2.0, 1.0, -1.0])
    radius = 3.0

    points = pyrsc.ShapeGenerator(seed=0).sphere(center, radius, n_points=200, noise=0.0, n_outliers=100)

    # The box where the outliers are scattered is the bounding box of the shape enlarged by the
    # margin, and the sphere is centered on its own bounding box
    half_size = OUTLIER_MARGIN * radius
    assert np.all(np.abs(points - center) <= half_size + 1e-12)

    # The outliers do not belong to the sphere, so they are not on its hull
    distances = np.linalg.norm(points - center, axis=1)
    assert np.count_nonzero(np.abs(distances - radius) > 1e-9) == 100


def test_outlier_bounds_replace_the_default_box():
    center = np.asarray([1.0, -2.0, 3.0])
    bounds = 4.0

    points = pyrsc.ShapeGenerator(seed=0).point(center, n_points=100, noise=0.0, n_outliers=100, outlier_bounds=bounds)

    # Every point of the shape is on the center, so the box is centered on it
    assert np.all(np.abs(points - center) <= bounds)
    assert np.amax(np.abs(points - center)) > 0.5 * bounds


def test_outliers_are_mixed_with_the_points_of_the_shape():
    center = np.asarray([1.0, -2.0, 3.0])
    n_points = 100

    points = pyrsc.ShapeGenerator(seed=0).point(center, n_points=n_points, noise=0.0, n_outliers=100, outlier_bounds=4.0)

    # The cloud is shuffled, so the order of the points does not tell which ones are outliers
    is_outlier = np.linalg.norm(points - center, axis=1) > 1e-9
    assert np.count_nonzero(is_outlier[0:n_points]) > 0
    assert np.count_nonzero(is_outlier[n_points:]) < 100


def test_number_of_outliers_must_not_be_negative():
    with pytest.raises(ValueError, match="n_outliers must be an integer of at least 0"):
        pyrsc.ShapeGenerator(seed=0).sphere([0.0, 0.0, 0.0], 1.0, n_outliers=-1)


def test_generator_rejects_a_center_which_is_not_a_3d_coordinate():
    with pytest.raises(ValueError, match=r"center must be a 3D coordinate as a np.array \(3,\)"):
        pyrsc.ShapeGenerator(seed=0).point([1.0, 2.0])


def test_generator_rejects_a_center_which_is_not_finite():
    with pytest.raises(ValueError, match="center must be finite"):
        pyrsc.ShapeGenerator(seed=0).point([1.0, 2.0, np.inf])


def test_generator_rejects_a_direction_without_a_direction():
    with pytest.raises(ValueError, match="direction must not be a zero vector"):
        pyrsc.ShapeGenerator(seed=0).line([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])


@pytest.mark.parametrize("radius", [0.0, -1.0, np.inf])
def test_generator_rejects_a_radius_which_is_not_a_real_size(radius):
    with pytest.raises(ValueError, match="radius must be finite and bigger than zero"):
        pyrsc.ShapeGenerator(seed=0).sphere([0.0, 0.0, 0.0], radius)


def test_generator_rejects_a_size_with_the_wrong_number_of_dimensions():
    with pytest.raises(ValueError, match=r"size must be a scalar or a np.array \(2,\)"):
        pyrsc.ShapeGenerator(seed=0).plane([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], size=[1.0, 2.0, 3.0])


def test_generator_rejects_axes_which_are_not_a_3x3_matrix():
    with pytest.raises(ValueError, match=r"axes must be a np.array \(3, 3\)"):
        pyrsc.ShapeGenerator(seed=0).cuboid([0.0, 0.0, 0.0], 1.0, axes=np.eye(2))


def test_generator_rejects_axes_which_are_not_orthonormal():
    axes = np.asarray([[1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 0.0, 1.0]])

    with pytest.raises(ValueError, match="axes must be orthonormal"):
        pyrsc.ShapeGenerator(seed=0).cuboid([0.0, 0.0, 0.0], 1.0, axes=axes)
