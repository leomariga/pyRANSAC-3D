# pyransac3d.cone

## Cone Objects

```
class Cone()
```

Implementation for cone RANSAC.

This class finds an infinite right circular cone, and returns its apex, the direction of its axis and its half-angle, which is the angle between the axis and the surface of the cone.

A cone cannot be found from points alone: three points are not enough to define the six parameters of a cone, but three points **and their normals** are, because every plane tangent to a cone passes through its apex. So `fit(.)` works on the normals of the surface, which it takes from `normals` when they are given, and estimates from the cloud itself when they are not.

**Attributes**:

- `apex` - Point where the surface of the cone converges, `np.array (3,)`.
- `axis` - Unit vector describing the direction of the cone's axis, pointing from the apex towards the opening, `np.array (3,)`.
- `angle` - Half-angle of the cone in radians, between its axis and its surface.
- `inliers` - Index of the points of `pts` which fit the cone.
- `distances` - Distance from each point of `pts` to the surface of the cone, `np.array (N,)`, in the same order of `pts`.

______________________________________________________________________

#### fit

```
def fit(pts: NDArray[np.float64],
        thresh: float = 0.2,
        maxIteration: int = 1000,
        normals: NDArray[np.float64] | None = None,
        k_neighbors: int = 16,
        callback: Callable[[FitState], bool | None] | None = None
        ) -> ConeResult
```

Find the parameters (apex, axis and half-angle) defining a cone.

**Arguments**:

- `pts`: 3D point cloud as a numpy array (N,3).
- `thresh`: Threshold distance from the cone surface which is considered inlier.
- `maxIteration`: Number of maximum iteration which RANSAC will loop over.
- `normals`: Optional unit normals of the surface at each point of `pts`, `np.array (N,3)`. When `None`, they are estimated from `pts` with `estimate_normals(.)`, which is quadratic in the cloud size. Passing the normals you already have, such as the ones of an Open3D cloud, skips it. Their orientation does not matter, only their direction.
- `k_neighbors`: Number of neighbors used to estimate the normals, ignored when `normals` is given.
- `callback`: Optional callable which receives a `FitState` after every iteration. Return a truthy value from it to stop the fit early.

**Returns**:

`ConeResult` with the `apex`, the `axis` and the `angle` of the cone, and its `inliers`.

Everything else measured while fitting is kept on the object, described in the attributes of this class.

______________________________________________________________________
