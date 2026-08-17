# pyransac3d.cuboid

## Cuboid Objects

```
class Cuboid()
```

Implementation for box (Cuboid) RANSAC.

A cuboid is defined as convex polyhedron bounded by six faces formed by three orthogonal normal vectors. Cats love to play with this kind of geometry. This method uses 6 points to find 3 best plane equations orthogonal to eachother.

We could use a recursive planar RANSAC, but it would use 9 points instead. Orthogonality makes this algorithm more efficient.

Once the three faces are found, the bounded box which contains them is measured. The face with most inliers gives the reference axis, and the remaining rotation around it is the one which makes the smallest footprint when the inliers are projected on the plane orthogonal to that axis. This way `fit(.)` also gives the `center`, the `extents` and the `axes` of a real box, and not only the equations of three infinite planes.

______________________________________________________________________

#### fit

```
def fit(
    pts: NDArray[np.float64],
    thresh: float = 0.05,
    maxIteration: int = 5000,
    callback: Callable[[dict[str, Any]], bool | None] | None = None
) -> tuple[
        NDArray[np.float64] | list[float],
        NDArray[np.float64] | list[float],
        NDArray[np.float64] | list[float],
        NDArray[np.intp] | list[int],
]
```

Find the cuboid which best fits the point cloud, from the 3 orthogonal planes of its faces.

**Arguments**:

- `pts`: 3D point cloud as a `np.array (N,3)`.
- `thresh`: Threshold distance from the cylinder radius which is considered inlier.
- `maxIteration`: Number of maximum iteration which RANSAC will loop over.
- `callback`: Optional callable invoked after every non-degenerate iteration with a state `dict`. Useful to plot the fitting progress, inspect intermediate results, or implement a custom early-stopping criterion. If it returns a truthy value, fitting stops early and the current best result is returned. Treat the arrays in the state `dict` as read-only. State keys:
- `iteration`: current iteration index (0-based)
- `sample_indices`: indices of the points sampled this iteration
- `sample_points`: the sampled points, `np.array (6, 3)`
- `model`: `dict` with this iteration's candidate `equation` (3 planes)
- `inliers`: inlier indices found for this iteration's candidate
- `best_model`: `dict` with the best `equation` (3 planes) found so far and the `center`, `extents` and `axes` of its box. Measuring the box is much more expensive than testing a candidate, so it only happens when the best candidate changes
- `best_inliers`: best inlier indices found so far
- `is_best`: `True` if this iteration became the new best candidate

**Returns**:

- `center`: Center of the cuboid `np.array (3,)`
- `extents`: Size of the cuboid along each one of its axes `np.array (3,)`
- `axes`: Orthonormal axes of the cuboid, one per row, where the last one is the normal of the face with most inliers `np.array (3, 3)`
- `inliers`: Inlier's index from the original point cloud. `np.array (1, M)`

The equations of the 3 orthogonal planes used to build the box are not returned, but they are kept in the object and can be read from `self.equation` as a `np.array (3, 4)`.

The distances measured to select the inliers are not returned either, but they are kept in the object, in the same order of `pts`:

- `self.plane_distances`: distance from each point to each one of the 3 planes, `np.array (3, N)`
- `self.distances`: distance from each point to the closest one of the 3 planes, `np.array (N,)`

The measurements used to build the box from the 3 planes are kept in the object too:

- `self.face_normals`: normal of each one of the 3 planes `np.array (3, 3)`
- `self.face_inlier_count`: how many inliers are closest to each plane `np.array (3,)`
- `self.ref_face_index`: index of the plane with most inliers, whose normal is the reference axis `self.axes[2]`
- `self.z_bounds`: smallest and biggest coordinate of the inliers along the reference axis `np.array (2,)`
- `self.hull_angle`: rotation around the reference axis, in radians, which gives the smallest footprint of the inliers `float`

Call `get_corners(.)` to get the 8 vertices of the box and `get_transform(.)` to get its rotation and translation as a single matrix.

______________________________________________________________________

#### get_corners

```
def get_corners() -> NDArray[np.float64]
```

Get the vertices of the fitted cuboid.

**Returns**:

The 8 vertices of the cuboid `np.array (8, 3)`, the first 4 on one face and the last 4 on the opposite one, both in the same order.

______________________________________________________________________

#### get_transform

```
def get_transform() -> NDArray[np.float64]
```

Get the rotation and the translation of the fitted cuboid as a single matrix.

The matrix takes a point from the cuboid's own frame, where the box is centered on the origin and aligned with the coordinate axes with the size given by `self.extents`, to the frame of the point cloud. The rotation is `transform[0:3, 0:3]` and the translation is `transform[0:3, 3]`, which is the same as `self.center`.

Keep in mind a box looks the same after being rotated by 90 degrees around its own axes, so this is one of the 24 rotations which describe the same cuboid, and not a unique answer.

**Returns**:

Homogeneous transformation matrix from the cuboid frame to the point cloud frame `np.array (4, 4)`

______________________________________________________________________
