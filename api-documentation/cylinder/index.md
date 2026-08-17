# pyransac3d.cylinder

## Cylinder Objects

```
class Cylinder()
```

Warning

The cylinder RANSAC does NOT present good results on real data on the current version. We are working to make a better algorithim using normals. If you want to contribute, please create a MR on github. Or give us ideas on [this issue](https://github.com/leomariga/pyRANSAC-3D/issues/13)

Implementation for cylinder RANSAC.

This class finds a infinite height cilinder and returns the cylinder axis, center and radius.

______________________________________________________________________

#### fit

```
def fit(
    pts: NDArray[np.float64],
    thresh: float = 0.2,
    maxIteration: int = 10000,
    callback: Callable[[dict[str, Any]], bool | None] | None = None
) -> tuple[
        NDArray[np.float64] | list[float],
        NDArray[np.float64] | list[float],
        float,
        NDArray[np.intp] | list[int],
]
```

Find the parameters (axis and radius) defining a cylinder.

**Arguments**:

- `pts`: 3D point cloud as a numpy array (N,3).
- `thresh`: Threshold distance from the cylinder hull which is considered inlier.
- `maxIteration`: Number of maximum iteration which RANSAC will loop over.
- `callback`: Optional callable invoked after every non-degenerate iteration with a state `dict`. If it returns a truthy value, fitting stops early.

**Returns**:

- `center`: Center of the cylinder np.array(1,3) which the cylinder axis is passing through.
- `axis`: Vector describing cylinder's axis np.array(1,3).
- `radius`: Radius of cylinder.
- `inliers`: Inlier's index from the original point cloud.

The distances measured to select the inliers are not returned, but they are kept in the object, both as a `np.array (N,)` in the same order of `pts`:

- `self.radial_distances`: distance from each point to the axis of the cylinder
- `self.distances`: distance from each point to the hull of the cylinder

The optional `callback` is invoked after every non-degenerate iteration with a state `dict`. Useful to plot the fitting progress, inspect intermediate results, or implement a custom early-stopping criterion. If it returns a truthy value, fitting stops early and the current best result is returned. Treat the arrays in the state `dict` as read-only. State keys:

- `iteration`: current iteration index (0-based)
- `sample_indices`: indices of the points sampled this iteration
- `sample_points`: the sampled points, `np.array (3, 3)`
- `model`: `dict` with this iteration's candidate `center`, `axis`, `radius`
- `inliers`: inlier indices found for this iteration's candidate
- `best_model`: `dict` with the best `center`, `axis`, `radius` found so far
- `best_inliers`: best inlier indices found so far
- `is_best`: `True` if this iteration became the new best candidate

______________________________________________________________________
