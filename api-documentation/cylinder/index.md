# pyransac3d.cylinder

## Cylinder Objects

```
class Cylinder()
```

The cylinder RANSAC does NOT present good results on real data on the current version

We are working to make a better algorithim using normals. If you want to contribute, please create a MR on github. Or give us ideas on [this issue](https://github.com/leomariga/pyRANSAC-3D/issues/13)

Implementation for cylinder RANSAC.

This class finds a infinite height cilinder and returns the cylinder axis, center and radius.

**Attributes**:

- `center` - A point the axis of the cylinder passes through, `np.array (3,)`.
- `axis` - Unit vector describing the direction of the cylinder's axis, `np.array (3,)`.
- `radius` - Radius of the cylinder.
- `inliers` - Index of the points of `pts` which fit the cylinder.
- `radial_distances` - Distance from each point of `pts` to the axis of the cylinder, `np.array (N,)`, in the same order of `pts`.
- `distances` - Distance from each point of `pts` to the hull of the cylinder, `np.array (N,)`, in the same order of `pts`.

______________________________________________________________________

#### fit

```
def fit(
    pts: NDArray[np.float64],
    thresh: float = 0.2,
    maxIteration: int = 10000,
    callback: Callable[[FitState], bool | None] | None = None
) -> CylinderResult
```

Find the parameters (axis and radius) defining a cylinder.

**Arguments**:

- `pts`: 3D point cloud as a numpy array (N,3).
- `thresh`: Threshold distance from the cylinder hull which is considered inlier.
- `maxIteration`: Number of maximum iteration which RANSAC will loop over.
- `callback`: Optional callable which receives a `FitState` after every iteration. Return a truthy value from it to stop the fit early.

**Returns**:

`CylinderResult` with the `center`, the `axis` and the `radius` of the cylinder, and its `inliers`.

Everything else measured while fitting is kept on the object, described in the attributes of this class.

______________________________________________________________________
