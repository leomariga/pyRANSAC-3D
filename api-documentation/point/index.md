# pyransac3d.point

## Point Objects

```
class Point()
```

Implementation for Point RANSAC.

This object finds the coordinate of a point in 3D space using RANSAC method. The point with more neighbors in a determined radius (`thresh`) will be selected as the best candidate.

______________________________________________________________________

#### fit

```
def fit(
    pts: NDArray[np.float64],
    thresh: float = 0.2,
    maxIteration: int = 10000,
    callback: Callable[[dict[str, Any]], bool | None] | None = None
) -> tuple[NDArray[np.float64] | list[float], NDArray[np.intp] | list[int]]
```

Find the best point for the 3D Point representaiton. The Point in a 3d enviroment is defined as a X, Y Z coordinate with more neighbors around.

The optional `callback` is invoked after every iteration with a state `dict`. Useful to plot the fitting progress, inspect intermediate results, or implement a custom early-stopping criterion. If it returns a truthy value, fitting stops early and the current best result is returned. Treat the arrays in the state `dict` as read-only. State keys:

- `iteration`: current iteration index (0-based)
- `sample_indices`: indices of the points sampled this iteration
- `sample_points`: the sampled points, `np.array (1, 3)`
- `model`: `dict` with this iteration's candidate `center`
- `inliers`: inlier indices found for this iteration's candidate
- `best_model`: `dict` with the best `center` found so far
- `best_inliers`: best inlier indices found so far
- `is_best`: `True` if this iteration became the new best candidate

**Arguments**:

- `pts`: 3D point cloud as a `np.array (N,3)`.
- `thresh`: Threshold radius from the point which is considered inlier.
- `maxIteration`: Number of maximum iteration which RANSAC will loop over.
- `callback`: Optional callable invoked after every iteration with a state `dict`. If it returns a truthy value, fitting stops early.

**Returns**:

- `center`: Point selected as best candidate `np.array (1, 3)`
- `inliers`: Inlier's index from the original point cloud. `np.array (1, M)`

The distances used to select the inliers are not returned, but they are kept in the object and can be read from `self.distances` as a `np.array (N,)`, in the same order of `pts`.

______________________________________________________________________
