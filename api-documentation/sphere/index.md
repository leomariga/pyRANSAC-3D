# pyransac3d.sphere

## Sphere Objects

```
class Sphere()
```

Implementation for Sphere RANSAC. A Sphere is defined as points spaced from the center by a constant radius.

This class finds the center and radius of a sphere. Base on article "PGP2X: Principal Geometric Primitives Parameters Extraction"

**Attributes**:

- `center` - Center of the sphere, `np.array (3,)`.
- `radius` - Radius of the sphere.
- `inliers` - Index of the points of `pts` which fit the sphere.
- `radial_distances` - Distance from each point of `pts` to the center of the sphere, `np.array (N,)`, in the same order of `pts`.
- `distances` - Distance from each point of `pts` to the hull of the sphere, `np.array (N,)`, in the same order of `pts`.

______________________________________________________________________

#### fit

```
def fit(
    pts: NDArray[np.float64],
    thresh: float = 0.2,
    maxIteration: int = 1000,
    callback: Callable[[FitState], bool | None] | None = None
) -> tuple[list[float], float, NDArray[np.intp] | list[int]]
```

Find the parameters (center and radius) to define a Sphere.

**Arguments**:

- `pts`: 3D point cloud as a numpy array (N,3).
- `thresh`: Threshold distance from the Sphere hull which is considered inlier.
- `maxIteration`: Number of maximum iteration which RANSAC will loop over.
- `callback`: Optional callable which receives a `FitState` after every iteration. Return a truthy value from it to stop the fit early.

**Returns**:

- `center`: Center of the sphere `np.array (3,)`
- `radius`: Radius of the sphere
- `inliers`: Inlier's index from the original point cloud

Everything else measured while fitting is kept on the object, described in the attributes of this class.

______________________________________________________________________
