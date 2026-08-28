# pyransac3d.aux_functions

#### get_rotationMatrix_from_vectors

```
def get_rotationMatrix_from_vectors(u: ArrayLike,
                                    v: ArrayLike) -> NDArray[np.float64]
```

Create a rotation matrix that rotates the space from a 3D vector `u` to a 3D vector `v`

**Arguments**:

- `u`: Orign vector `np.array (1,3)`.
- `v`: Destiny vector `np.array (1,3)`.

**Returns**:

## Rotation matrix `np.array (3, 3)`

#### convex_hull_2d

```
def convex_hull_2d(points: ArrayLike) -> NDArray[np.float64]
```

Find the convex hull of a set of 2D points using Andrew's monotone chain algorithm.

**Arguments**:

- `points`: Set of 2D points `np.array (N,2)`.

**Returns**:

Hull vertices in counter-clockwise order as a closed polygon, which means the first vertex is repeated at the end `np.array (M+1, 2)`

______________________________________________________________________

#### min_bounding_rect

```
def min_bounding_rect(
    hull_points: ArrayLike
) -> tuple[np.float64, np.float64, np.float64, np.float64, NDArray[np.float64],
           NDArray[np.float64]]
```

Find the minimum-area rectangle which encloses a 2D convex hull.

The rectangle is aligned with one of the hull edges, so we only have to test the orientation of each edge and keep the one which gives the smallest area.

**Arguments**:

- `hull_points`: Convex hull as a closed polygon `np.array (N,2)`, as returned by `convex_hull_2d(.)`.

**Returns**:

- `angle`: Rotation of the rectangle in radians, inside the first quadrant `float`
- `area`: Area of the rectangle `float`
- `width`: Size of the rectangle along its first axis `float`
- `height`: Size of the rectangle along its second axis `float`
- `center`: Center of the rectangle `np.array (2,)`
- `corners`: Corners of the rectangle `np.array (4, 2)`

______________________________________________________________________

#### rodrigues_rot

```
def rodrigues_rot(P: ArrayLike, n0: ArrayLike,
                  n1: ArrayLike) -> NDArray[np.float64]
```

Rotate a set of point between two normal vectors using Rodrigues' formula.

**Arguments**:

- `P`: Set of points `np.array (N,3)`.
- `n0`: Orign vector `np.array (1,3)`.
- `n1`: Destiny vector `np.array (1,3)`.

**Returns**:

## Set of points P, but rotated `np.array (N, 3)`

#### estimate_normals

```
def estimate_normals(pts: ArrayLike, k: int = 16) -> NDArray[np.float64]
```

Estimate the normal of the surface on every point of a cloud.

The normal of a point is taken from its `k` nearest neighbors: the direction in which they spread the least is the one leaving the surface, which is the eigenvector of the smallest eigenvalue of their covariance.

The normals are not oriented, which means a normal may point inwards while the next one points outwards. That is enough for the fitters, which only use the direction of the normal, but it is not enough to render them.

Performance

This is a brute-force nearest-neighbor search: runtime grows quadratically with the cloud size. Pass normals you already have on large scans instead of estimating them here.

**Arguments**:

- `pts`: 3D point cloud as a numpy array (N,3).
- `k`: Number of neighbors used on each point. It is clamped to the size of the cloud.

**Returns**:

## Unit normal of the surface on each point `np.array (N, 3)`
