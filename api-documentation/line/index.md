# pyransac3d.line

## Line Objects

```
class Line()
```

Implementation for 3D Line RANSAC.

This object finds the equation of a line in 3D space using RANSAC method. This method uses 2 points from 3D space and computes a line. The selected candidate will be the line with more inliers inside the radius theshold.

______________________________________________________________________

#### fit

```
def fit(
    pts: NDArray[np.float64],
    thresh: float = 0.2,
    maxIteration: int = 1000,
    callback: Callable[[dict[str, Any]], bool | None] | None = None
) -> tuple[
        NDArray[np.float64] | list[float],
        NDArray[np.float64] | list[float],
        NDArray[np.intp] | list[int],
]
```

Find the best equation for the 3D line. The line in a 3d enviroment is defined as y = Ax+B, but A and B are vectors intead of scalars.

**Arguments**:

- `pts`: 3D point cloud as a `np.array (N,3)`.
- `thresh`: Threshold distance from the line which is considered inlier.
- `maxIteration`: Number of maximum iteration which RANSAC will loop over.
- `callback`: Optional callable invoked after every iteration with a state `dict`. Useful to plot the fitting progress, inspect intermediate results, or implement a custom early-stopping criterion. If it returns a truthy value, fitting stops early and the current best result is returned. Treat the arrays in the state `dict` as read-only. State keys:
- `iteration`: current iteration index (0-based)
- `sample_indices`: indices of the points sampled this iteration
- `sample_points`: the sampled points, `np.array (2, 3)`
- `model`: `dict` with this iteration's candidate `A` and `B`
- `inliers`: inlier indices found for this iteration's candidate
- `best_model`: `dict` with the best `A` and `B` found so far
- `best_inliers`: best inlier indices found so far
- `is_best`: `True` if this iteration became the new best candidate

**Returns**:

- `A`: 3D slope of the line (angle) `np.array (1, 3)`
- `B`: Axis interception as `np.array (1, 3)`
- `inliers`: Inlier's index from the original point cloud. `np.array (1, M)`

The distances used to select the inliers are not returned, but they are kept in the object and can be read from `self.distances` as a `np.array (N,)`, in the same order of `pts`.

______________________________________________________________________
