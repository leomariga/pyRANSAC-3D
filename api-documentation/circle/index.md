# pyransac3d.circle

## Circle Objects

```
class Circle()
```

Implementation for Circle RANSAC.

This class finds the circle's parameters based on 3 sampled points. This method uses 3 points to find the circle's plane, center and radius.

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
        float,
        NDArray[np.intp] | list[int],
]
```

Find the parameters (axis and radius and center) to define a circle.

The optional `callback` is invoked after every non-degenerate iteration with a state `dict`. Useful to plot the fitting progress, inspect intermediate results, or implement a custom early-stopping criterion. If it returns a truthy value, fitting stops early and the current best result is returned. Treat the arrays in the state `dict` as read-only. State keys:

- `iteration`: current iteration index (0-based)
- `sample_indices`: indices of the points sampled this iteration
- `sample_points`: the sampled points, `np.array (3, 3)`
- `model`: `dict` with this iteration's candidate `center`, `axis`, `radius`
- `inliers`: inlier indices found for this iteration's candidate
- `best_model`: `dict` with the best `center`, `axis`, `radius` found so far
- `best_inliers`: best inlier indices found so far
- `is_best`: `True` if this iteration became the new best candidate

**Arguments**:

- `pts`: 3D point cloud as a numpy array (N,3).
- `thresh`: Threshold distance from the cylinder hull which is considered inlier.
- `maxIteration`: Number of maximum iteration which RANSAC will loop over.
- `callback`: Optional callable invoked after every non-degenerate iteration with a state `dict`. If it returns a truthy value, fitting stops early.

**Returns**:

- `center`: Center of the circle np.array(1,3) which the circle center is passing through.
- `axis`: Vector describing circle's plane normal as np.array(1,3).
- `radius`: Radius of the circle.
- `inliers`: Inlier's index from the original point cloud.

The equation of the plane which contains the circle is not returned, but it is kept in the object and can be read from `self.plane_equation` as Ax+By+Cz+D `np.array (1, 4)`, where A, B and C are the same as `self.axis`.

The distances measured to select the inliers are not returned either, but they are kept in the object, all of them as a `np.array (N,)` in the same order of `pts`:

- `self.plane_distances`: signed distance from each point to the plane of the circle
- `self.radial_distances`: distance from each point to the hull of the circle if it was extruded along its axis, which is negative for points inside of it
- `self.distances`: distance from each point to the hull of the circle

______________________________________________________________________
