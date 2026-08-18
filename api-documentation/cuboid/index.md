# pyransac3d.cuboid

## Cuboid Objects

```
class Cuboid()
```

Implementation for box (Cuboid) RANSAC.

A cuboid is defined as convex polyhedron bounded by six faces formed by three orthogonal normal vectors. This method uses 6 points to find 3 best plane equations orthogonal to eachother.

Once the three faces are found, the bounded box which contains them is measured. The face with most inliers gives the reference axis, and the remaining rotation around it is the one which makes the smallest footprint when the inliers are projected on the plane orthogonal to that axis. This way `fit(.)` also gives the `center`, the `extents` and the `axes` of a real box, and not only the equations of three infinite planes.

**Attributes**:

- `center` - Center of the cuboid, `np.array (3,)`.
- `extents` - Size of the cuboid along each one of its axes, `np.array (3,)`.
- `axes` - Orthonormal axes of the cuboid, one per row, where the last one is the normal of the face with most inliers, `np.array (3, 3)`.
- `inliers` - Index of the points of `pts` which fit the cuboid.
- `equation` - Equation of the 3 orthogonal planes used to build the box, one per row, using Ax+By+Cz+D, `np.array (3, 4)`.
- `plane_distances` - Distance from each point of `pts` to each one of the 3 planes, `np.array (3, N)`, in the same order of `pts`.
- `distances` - Distance from each point of `pts` to the closest one of the 3 planes, `np.array (N,)`, in the same order of `pts`.
- `face_normals` - Normal of each one of the 3 planes, one per row, `np.array (3, 3)`.
- `face_inlier_count` - How many inliers are closest to each plane, `np.array (3,)`.
- `ref_face_index` - Index of the plane with most inliers, whose normal is the reference axis `axes[2]`.
- `z_bounds` - Smallest and biggest coordinate of the inliers along the reference axis, `np.array (2,)`.
- `hull_angle` - Rotation around the reference axis, in radians, which gives the smallest footprint of the inliers.

______________________________________________________________________

#### fit

```
def fit(pts: NDArray[np.float64],
        thresh: float = 0.05,
        maxIteration: int = 5000,
        callback: Callable[[FitState], bool | None] | None = None
        ) -> CuboidResult
```

Find the cuboid which best fits the point cloud, from the 3 orthogonal planes of its faces.

**Arguments**:

- `pts`: 3D point cloud as a `np.array (N,3)`.
- `thresh`: Threshold distance from the cylinder radius which is considered inlier.
- `maxIteration`: Number of maximum iteration which RANSAC will loop over.
- `callback`: Optional callable which receives a `FitState` after every iteration. Return a truthy value from it to stop the fit early. Passing a callback measures the box of every new best candidate, so its `best_model` also carries the `center`, the `extents` and the `axes` of the box, at the cost of a slower fit.

**Returns**:

`CuboidResult` with the `center`, the `extents` and the `axes` of the cuboid, and its `inliers`.

Everything else measured while fitting is kept on the object, described in the attributes of this class. Call `get_corners(.)` to get the 8 vertices of the box and `get_transform(.)` to get its rotation and translation as a single matrix.

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

The matrix takes a point from the cuboid's own frame, where the box is centered on the origin and aligned with the coordinate axes with the size given by `extents`, to the frame of the point cloud. The rotation is `transform[0:3, 0:3]` and the translation is `transform[0:3, 3]`, which is the same as `center`.

Keep in mind a box looks the same after being rotated by 90 degrees around its own axes, so this is one of the 24 rotations which describe the same cuboid, and not a unique answer.

**Returns**:

Homogeneous transformation matrix from the cuboid frame to the point cloud frame `np.array (4, 4)`

______________________________________________________________________
