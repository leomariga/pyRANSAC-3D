# pyransac3d.plane

## Plane Objects

```
class Plane()
```

Implementation of planar RANSAC.

Class for Plane object, which finds the equation of a infinite plane using RANSAC algorithim.

Call `fit(.)` to randomly take 3 points of pointcloud to verify inliers based on a threshold.

**Attributes**:

- `equation` - Parameters of the plane using Ax+By+Cz+D, `np.array (4,)`.
- `inliers` - Index of the points of `pts` which fit the plane.
- `distances` - Signed distance from each point of `pts` to the plane, `np.array (N,)`, in the same order of `pts`. The sign tells on which side of the plane the point is.

______________________________________________________________________

#### fit

```
def fit(pts: NDArray[np.float64],
        thresh: float = 0.05,
        maxIteration: int = 1000,
        callback: Callable[[FitState], bool | None] | None = None
        ) -> PlaneResult
```

Find the best equation for a plane.

**Arguments**:

- `pts`: 3D point cloud as a `np.array (N,3)`.
- `thresh`: Threshold distance from the plane which is considered inlier.
- `maxIteration`: Number of maximum iteration which RANSAC will loop over.
- `callback`: Optional callable which receives a `FitState` after every iteration. Return a truthy value from it to stop the fit early.

**Returns**:

`PlaneResult` with the `equation` of the plane and its `inliers`. Everything else measured while fitting is kept on the object, described in the attributes of this class.

______________________________________________________________________
