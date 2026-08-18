# pyransac3d.fit_results

Result of `fit(.)`, one type per shape.

Every result is a `NamedTuple`, so it is still the plain tuple it has always been and can be unpacked or indexed, but its values can also be read by name, which is clearer when a shape has several parameters:

```
center, radius, inliers = pyrsc.Sphere().fit(points)

result = pyrsc.Sphere().fit(points)
print(result.center, result.radius)
```

A result only carries the parameters which define the fitted shape. Everything else measured while fitting is kept on the shape object, described in the attributes of each class.

______________________________________________________________________

## PlaneResult Objects

```
class PlaneResult(NamedTuple)
```

Result of `Plane.fit(.)`.

______________________________________________________________________

#### equation: `list[float]`

Parameters of the plane using Ax+By+Cz+D, `np.array (4,)`.

#### inliers: `NDArray[np.intp] | list[int]`

Index of the points of `pts` which fit the plane.

## CuboidResult Objects

```
class CuboidResult(NamedTuple)
```

Result of `Cuboid.fit(.)`.

______________________________________________________________________

#### center: `NDArray[np.float64] | list[float]`

Center of the cuboid, `np.array (3,)`.

#### extents: `NDArray[np.float64] | list[float]`

Size of the cuboid along each one of its axes, `np.array (3,)`.

#### axes: `NDArray[np.float64] | list[float]`

Orthonormal axes of the cuboid, one per row, where the last one is the normal of the face with most inliers, `np.array (3, 3)`.

#### inliers: `NDArray[np.intp] | list[int]`

Index of the points of `pts` which fit the cuboid.

## CylinderResult Objects

```
class CylinderResult(NamedTuple)
```

Result of `Cylinder.fit(.)`.

______________________________________________________________________

#### center: `NDArray[np.float64] | list[float]`

A point the axis of the cylinder passes through, `np.array (3,)`.

#### axis: `NDArray[np.float64] | list[float]`

Unit vector describing the direction of the cylinder's axis, `np.array (3,)`.

#### radius: `float`

Radius of the cylinder.

#### inliers: `NDArray[np.intp] | list[int]`

Index of the points of `pts` which fit the cylinder.

## SphereResult Objects

```
class SphereResult(NamedTuple)
```

Result of `Sphere.fit(.)`.

______________________________________________________________________

#### center: `list[float]`

Center of the sphere, `np.array (3,)`.

#### radius: `float`

Radius of the sphere.

#### inliers: `NDArray[np.intp] | list[int]`

Index of the points of `pts` which fit the sphere.

## LineResult Objects

```
class LineResult(NamedTuple)
```

Result of `Line.fit(.)`.

______________________________________________________________________

#### A: `NDArray[np.float64] | list[float]`

3D slope of the line, which is a unit vector along its direction, `np.array (3,)`.

#### B: `NDArray[np.float64] | list[float]`

Axis interception, which is a point the line passes through, `np.array (3,)`.

#### inliers: `NDArray[np.intp] | list[int]`

Index of the points of `pts` which fit the line.

## CircleResult Objects

```
class CircleResult(NamedTuple)
```

Result of `Circle.fit(.)`.

______________________________________________________________________

#### center: `NDArray[np.float64] | list[float]`

Center of the circle, `np.array (3,)`.

#### axis: `NDArray[np.float64] | list[float]`

Unit normal vector of the plane which contains the circle, `np.array (3,)`.

#### radius: `float`

Radius of the circle.

#### inliers: `NDArray[np.intp] | list[int]`

Index of the points of `pts` which fit the circle.

## PointResult Objects

```
class PointResult(NamedTuple)
```

Result of `Point.fit(.)`.

______________________________________________________________________

#### center: `NDArray[np.float64] | list[float]`

Point selected as best candidate, `np.array (3,)`.

#### inliers: `NDArray[np.intp] | list[int]`

Index of the points of `pts` which are neighbors of `center`.
