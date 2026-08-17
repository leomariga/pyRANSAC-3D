# pyransac3d.sphere

## Sphere Objects

```
class Sphere()
```

Implementation for Sphere RANSAC. A Sphere is defined as points spaced from the center by a constant radius.

This class finds the center and radius of a sphere. Base on article "PGP2X: Principal Geometric Primitives Parameters Extraction"

______________________________________________________________________

#### fit

```
def fit(
    pts: NDArray[np.float64],
    thresh: float = 0.2,
    maxIteration: int = 1000,
    callback: Callable[[dict[str, Any]], bool | None] | None = None
) -> tuple[list[float], float, NDArray[np.intp] | list[int]]
```

Find the parameters (center and radius) to define a Sphere.

**Arguments**:

- `pts`: 3D point cloud as a numpy array (N,3).
- `thresh`: Threshold distance from the Sphere hull which is considered inlier.
- `maxIteration`: Number of maximum iteration which RANSAC will loop over.
- `callback`: Optional callable invoked after every iteration with a state `dict`. If it returns a truthy value, fitting stops early.

**Returns**:

- `center`: Center of the cylinder np.array(1,3) which the cylinder axis is passing through.
- `radius`: Radius of cylinder.
- `inliers`: Inlier's index from the original point cloud.

The distances measured to select the inliers are not returned, but they are kept in the object, both as a `np.array (N,)` in the same order of `pts`:

- `self.radial_distances`: distance from each point to the center of the sphere
- `self.distances`: distance from each point to the hull of the sphere

The optional `callback` is invoked after every iteration with a state `dict`. Useful to plot the fitting progress, inspect intermediate results, or implement a custom early-stopping criterion. If it returns a truthy value, fitting stops early and the current best result is returned. Treat the arrays in the state `dict` as read-only. State keys:

- `iteration`: current iteration index (0-based)
- `sample_indices`: indices of the points sampled this iteration
- `sample_points`: the sampled points, `np.array (4, 3)`
- `model`: `dict` with this iteration's candidate `center` and `radius`
- `inliers`: inlier indices found for this iteration's candidate
- `best_model`: `dict` with the best `center` and `radius` found so far
- `best_inliers`: best inlier indices found so far
- `is_best`: `True` if this iteration became the new best candidate

______________________________________________________________________
