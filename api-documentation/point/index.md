# pyransac3d.point

## Point Objects

```
class Point()
```

Implementation for Point RANSAC.

This object finds the coordinate of a point in 3D space using RANSAC method. The point with more neighbors in a determined radius (`thresh`) will be selected as the best candidate.

**Attributes**:

- `center` - Point selected as best candidate, `np.array (3,)`.
- `inliers` - Index of the points of `pts` which are neighbors of `center`.
- `distances` - Distance from each point of `pts` to `center`, `np.array (N,)`, in the same order of `pts`.

______________________________________________________________________

#### fit

```
def fit(pts: NDArray[np.float64],
        thresh: float = 0.2,
        maxIteration: int = 10000,
        callback: Callable[[FitState], bool | None] | None = None
        ) -> PointResult
```

Find the best point for the 3D Point representaiton. The Point in a 3d enviroment is defined as a X, Y Z coordinate with more neighbors around.

**Arguments**:

- `pts`: 3D point cloud as a `np.array (N,3)`.
- `thresh`: Threshold radius from the point which is considered inlier.
- `maxIteration`: Number of maximum iteration which RANSAC will loop over.
- `callback`: Optional callable which receives a `FitState` after every iteration. Return a truthy value from it to stop the fit early.

**Returns**:

`PointResult` with the `center` selected as best candidate and its `inliers`. Everything else measured while fitting is kept on the object, described in the attributes of this class.

______________________________________________________________________
