# pyransac3d.shape_generator

## ShapeGenerator Objects

```
class ShapeGenerator()
```

Generator of synthetic point clouds for every primitive shape of the library.

Each method samples points on the surface of one shape from the parameters which describe it, so the same parameters can be compared against the ones found by the RANSAC fitters. The clouds are not perfect on purpose: `noise` scatters the points around the ideal surface and `n_outliers` adds points which do not belong to the shape at all, which is what RANSAC is supposed to handle.

The points are always shuffled, so the outliers are mixed with the rest of the cloud, and the only dependency is NumPy, which makes the clouds cheap to build and easy to use on tests.

All the randomness comes from a single `np.random.Generator` created on `__init__(.)`, so giving it a `seed` makes every cloud reproducible.

______________________________________________________________________

#### __init__

```
def __init__(seed: int | None = None) -> None
```

Create a generator of point clouds.

**Arguments**:

- `seed`: Seed of the internal `np.random.Generator`. Use an integer to get the same clouds on every run, or `None` to get a different cloud every time.

______________________________________________________________________

#### point

```
def point(center: ArrayLike,
          n_points: int = 200,
          noise: float = 0.05,
          n_outliers: int = 0,
          outlier_bounds: ArrayLike | None = None) -> NDArray[np.float64]
```

Generate a cloud of points gathered around a single 3D coordinate.

**Arguments**:

- `center`: Coordinate where the points are gathered `np.array (3,)`.
- `n_points`: Number of points of the cluster, without counting the outliers.
- `noise`: Standard deviation of the gaussian noise added to every coordinate.
- `n_outliers`: Number of points scattered around which do not belong to the shape.
- `outlier_bounds`: Half size of the box where the outliers are scattered, as a scalar or a `np.array (3,)`. When `None`, the box is the bounding box of the shape enlarged by `OUTLIER_MARGIN`.

**Returns**:

## Point cloud as a `np.array (N, 3)`, where `N` is `n_points + n_outliers`

#### line

```
def line(anchor: ArrayLike,
         direction: ArrayLike,
         length: float = 10.0,
         n_points: int = 200,
         noise: float = 0.05,
         n_outliers: int = 0,
         outlier_bounds: ArrayLike | None = None) -> NDArray[np.float64]
```

Generate a cloud of points along a 3D line.

**Arguments**:

- `anchor`: Coordinate of the middle of the segment `np.array (3,)`.
- `direction`: Direction of the line `np.array (3,)`, which does not need to be unitary.
- `length`: Length of the segment where the points are sampled.
- `n_points`: Number of points on the line, without counting the outliers.
- `noise`: Standard deviation of the gaussian noise added to every coordinate.
- `n_outliers`: Number of points scattered around which do not belong to the shape.
- `outlier_bounds`: Half size of the box where the outliers are scattered, as a scalar or a `np.array (3,)`. When `None`, the box is the bounding box of the shape enlarged by `OUTLIER_MARGIN`.

**Returns**:

## Point cloud as a `np.array (N, 3)`, where `N` is `n_points + n_outliers`

#### plane

```
def plane(center: ArrayLike,
          normal: ArrayLike,
          size: ArrayLike = 10.0,
          n_points: int = 400,
          noise: float = 0.02,
          n_outliers: int = 0,
          outlier_bounds: ArrayLike | None = None) -> NDArray[np.float64]
```

Generate a cloud of points on a rectangular patch of a plane.

**Arguments**:

- `center`: Center of the patch `np.array (3,)`.
- `normal`: Normal of the plane `np.array (3,)`, which does not need to be unitary.
- `size`: Size of the patch along its two directions, as a scalar or a `np.array (2,)`.
- `n_points`: Number of points on the plane, without counting the outliers.
- `noise`: Standard deviation of the gaussian noise added to every coordinate.
- `n_outliers`: Number of points scattered around which do not belong to the shape.
- `outlier_bounds`: Half size of the box where the outliers are scattered, as a scalar or a `np.array (3,)`. When `None`, the box is the bounding box of the shape enlarged by `OUTLIER_MARGIN`.

**Returns**:

## Point cloud as a `np.array (N, 3)`, where `N` is `n_points + n_outliers`

#### circle

```
def circle(center: ArrayLike,
           axis: ArrayLike,
           radius: float,
           n_points: int = 300,
           noise: float = 0.02,
           n_outliers: int = 0,
           outlier_bounds: ArrayLike | None = None) -> NDArray[np.float64]
```

Generate a cloud of points on the hull of a circle.

**Arguments**:

- `center`: Center of the circle `np.array (3,)`.
- `axis`: Normal of the plane which contains the circle `np.array (3,)`, which does not need to be unitary.
- `radius`: Radius of the circle.
- `n_points`: Number of points on the circle, without counting the outliers.
- `noise`: Standard deviation of the gaussian noise added to every coordinate.
- `n_outliers`: Number of points scattered around which do not belong to the shape.
- `outlier_bounds`: Half size of the box where the outliers are scattered, as a scalar or a `np.array (3,)`. When `None`, the box is the bounding box of the shape enlarged by `OUTLIER_MARGIN`.

**Returns**:

## Point cloud as a `np.array (N, 3)`, where `N` is `n_points + n_outliers`

#### sphere

```
def sphere(center: ArrayLike,
           radius: float,
           n_points: int = 500,
           noise: float = 0.02,
           n_outliers: int = 0,
           outlier_bounds: ArrayLike | None = None) -> NDArray[np.float64]
```

Generate a cloud of points on the hull of a sphere.

**Arguments**:

- `center`: Center of the sphere `np.array (3,)`.
- `radius`: Radius of the sphere.
- `n_points`: Number of points on the sphere, without counting the outliers.
- `noise`: Standard deviation of the gaussian noise added to every coordinate.
- `n_outliers`: Number of points scattered around which do not belong to the shape.
- `outlier_bounds`: Half size of the box where the outliers are scattered, as a scalar or a `np.array (3,)`. When `None`, the box is the bounding box of the shape enlarged by `OUTLIER_MARGIN`.

**Returns**:

## Point cloud as a `np.array (N, 3)`, where `N` is `n_points + n_outliers`

#### cylinder

```
def cylinder(center: ArrayLike,
             axis: ArrayLike,
             radius: float,
             height: float = 10.0,
             n_points: int = 500,
             noise: float = 0.02,
             n_outliers: int = 0,
             outlier_bounds: ArrayLike | None = None) -> NDArray[np.float64]
```

Generate a cloud of points on the lateral surface of a cylinder.

The caps are not sampled, because the fitter looks for a cylinder of infinite height and the points on the caps would be outliers.

**Arguments**:

- `center`: Center of the cylinder `np.array (3,)`, which its axis passes through.
- `axis`: Axis of the cylinder `np.array (3,)`, which does not need to be unitary.
- `radius`: Radius of the cylinder.
- `height`: Height of the piece of cylinder where the points are sampled.
- `n_points`: Number of points on the cylinder, without counting the outliers.
- `noise`: Standard deviation of the gaussian noise added to every coordinate.
- `n_outliers`: Number of points scattered around which do not belong to the shape.
- `outlier_bounds`: Half size of the box where the outliers are scattered, as a scalar or a `np.array (3,)`. When `None`, the box is the bounding box of the shape enlarged by `OUTLIER_MARGIN`.

**Returns**:

## Point cloud as a `np.array (N, 3)`, where `N` is `n_points + n_outliers`

#### cuboid

```
def cuboid(center: ArrayLike,
           extents: ArrayLike,
           axes: ArrayLike | None = None,
           n_points: int = 900,
           noise: float = 0.01,
           n_outliers: int = 0,
           outlier_bounds: ArrayLike | None = None) -> NDArray[np.float64]
```

Generate a cloud of points on the 6 faces of a cuboid.

**Arguments**:

- `center`: Center of the cuboid `np.array (3,)`.
- `extents`: Size of the cuboid along each one of its axes, as a scalar or a `np.array (3,)`.
- `axes`: Orthonormal axes of the cuboid, one per row, `np.array (3, 3)`. When `None`, the cuboid is aligned with the coordinate axes.
- `n_points`: Number of points on the cuboid, without counting the outliers. They are spread over the 6 faces proportionally to their area.
- `noise`: Standard deviation of the gaussian noise added to every coordinate.
- `n_outliers`: Number of points scattered around which do not belong to the shape.
- `outlier_bounds`: Half size of the box where the outliers are scattered, as a scalar or a `np.array (3,)`. When `None`, the box is the bounding box of the shape enlarged by `OUTLIER_MARGIN`.

**Returns**:

## Point cloud as a `np.array (N, 3)`, where `N` is `n_points + n_outliers`
