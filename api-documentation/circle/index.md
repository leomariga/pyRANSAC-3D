# pyransac3d.circle

## Circle Objects

```
class Circle()
```

Implementation for Circle RANSAC.

This class finds the circle's parameters based on 3 sampled points. This method uses 3 points to find the circle's plane, center and radius.

**Attributes**:

- `center` - Center of the circle, `np.array (3,)`.
- `axis` - Unit normal vector of the plane which contains the circle, `np.array (3,)`.
- `radius` - Radius of the circle.
- `inliers` - Index of the points of `pts` which fit the circle.
- `plane_equation` - Equation of the plane which contains the circle, using Ax+By+Cz+D, `np.array (4,)`, where A, B and C are the same as `axis`.
- `plane_distances` - Signed distance from each point of `pts` to the plane of the circle, `np.array (N,)`, in the same order of `pts`.
- `radial_distances` - Distance from each point of `pts` to the hull of the circle if it was extruded along its axis, `np.array (N,)`, in the same order of `pts`, which is negative for points inside of it.
- `distances` - Distance from each point of `pts` to the hull of the circle, `np.array (N,)`, in the same order of `pts`.

______________________________________________________________________

#### fit

```
def fit(pts: NDArray[np.float64],
        thresh: float = 0.2,
        maxIteration: int = 1000,
        callback: Callable[[FitState], bool | None] | None = None
        ) -> CircleResult
```

Find the parameters (axis and radius and center) to define a circle.

**Arguments**:

- `pts`: 3D point cloud as a numpy array (N,3).
- `thresh`: Threshold distance from the cylinder hull which is considered inlier.
- `maxIteration`: Number of maximum iteration which RANSAC will loop over.
- `callback`: Optional callable which receives a `FitState` after every iteration. Return a truthy value from it to stop the fit early.

**Returns**:

`CircleResult` with the `center`, the `axis` and the `radius` of the circle, and its `inliers`.

Everything else measured while fitting is kept on the object, described in the attributes of this class.

______________________________________________________________________
