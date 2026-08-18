# pyransac3d.line

## Line Objects

```
class Line()
```

Implementation for 3D Line RANSAC.

This object finds the equation of a line in 3D space using RANSAC method. This method uses 2 points from 3D space and computes a line. The selected candidate will be the line with more inliers inside the radius theshold.

**Attributes**:

- `A` - 3D slope of the line, which is a unit vector along its direction, `np.array (3,)`.
- `B` - Axis interception, which is a point the line passes through, `np.array (3,)`.
- `inliers` - Index of the points of `pts` which fit the line.
- `distances` - Distance from each point of `pts` to the line, `np.array (N,)`, in the same order of `pts`.

______________________________________________________________________

#### fit

```
def fit(
    pts: NDArray[np.float64],
    thresh: float = 0.2,
    maxIteration: int = 1000,
    callback: Callable[[FitState], bool | None] | None = None
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
- `callback`: Optional callable which receives a `FitState` after every iteration. Return a truthy value from it to stop the fit early.

**Returns**:

- `A`: 3D slope of the line (angle) `np.array (3,)`
- `B`: Axis interception as `np.array (3,)`
- `inliers`: Inlier's index from the original point cloud

Everything else measured while fitting is kept on the object, described in the attributes of this class.

______________________________________________________________________
