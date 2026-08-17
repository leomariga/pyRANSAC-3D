# pyransac3d.plane

## Plane Objects

```
class Plane()
```

Implementation of planar RANSAC.

Class for Plane object, which finds the equation of a infinite plane using RANSAC algorithim.

Call `fit(.)` to randomly take 3 points of pointcloud to verify inliers based on a threshold.

______________________________________________________________________

#### fit

```
def fit(
    pts: NDArray[np.float64],
    thresh: float = 0.05,
    maxIteration: int = 1000,
    callback: Callable[[dict[str, Any]], bool | None] | None = None
) -> tuple[list[float], NDArray[np.intp] | list[int]]
```

Find the best equation for a plane.

**Arguments**:

- `pts`: 3D point cloud as a `np.array (N,3)`.
- `thresh`: Threshold distance from the plane which is considered inlier.
- `maxIteration`: Number of maximum iteration which RANSAC will loop over.
- `callback`: Optional callable invoked after every non-degenerate iteration with a state `dict`. If it returns a truthy value, fitting stops early.

**Returns**:

- `self.equation`: Parameters of the plane using Ax+By+Cy+D `np.array (1, 4)`
- `self.inliers`: points from the dataset considered inliers

The distances used to select the inliers are not returned, but they are kept in the object and can be read from `self.distances` as a `np.array (N,)`, in the same order of `pts`. They are signed, so the sign tells on which side of the plane the point is.

The optional `callback` is invoked after every non-degenerate iteration with a state `dict`. Useful to plot the fitting progress, inspect intermediate results, or implement a custom early-stopping criterion. If it returns a truthy value, fitting stops early and the current best result is returned. Treat the arrays in the state `dict` as read-only. State keys:

- `iteration`: current iteration index (0-based)
- `sample_indices`: indices of the points sampled this iteration
- `sample_points`: the sampled points, `np.array (3, 3)`
- `model`: `dict` with this iteration's candidate `equation`
- `inliers`: inlier indices found for this iteration's candidate
- `best_model`: `dict` with the best `equation` found so far
- `best_inliers`: best inlier indices found so far
- `is_best`: `True` if this iteration became the new best candidate

______________________________________________________________________
