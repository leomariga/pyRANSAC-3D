# Getting Started

Welcome to **pyRANSAC-3D**.

The library fits simple 3D primitives to point clouds with RANSAC. Start here if you want a first working example before reading the API reference.

## On This Page

- [Install](#install)
- [Choose a Shape](#choose-a-shape)
- [First Fit: Plane](#first-fit-plane)
- [Reading the Result](#reading-the-result)
- [Fit a Sphere](#fit-a-sphere)
- [Fit a Cuboid](#fit-a-cuboid)
- [Callbacks](#callbacks)
- [Visualize With Open3D](#visualize-with-open3d)
- [Using Your Own Data](#using-your-own-data)
- [Where To Go Next](#where-to-go-next)

## Install

```
pip install pyransac3d
```

## Choose a Shape

pyRANSAC-3D exposes one fitter class per primitive:

| Shape      | Use it when you need                           | Main result                                     |
| ---------- | ---------------------------------------------- | ----------------------------------------------- |
| `Point`    | A dense cluster around one coordinate          | Center and inliers                              |
| `Line`     | A straight edge, rail, or scan line            | Direction, point on the line, and inliers       |
| `Plane`    | Floors, walls, tables, roofs, or flat surfaces | Plane equation and inliers                      |
| `Circle`   | Circular edges in 3D space                     | Center, axis, radius, and inliers               |
| `Sphere`   | Ball-like objects                              | Center, radius, and inliers                     |
| `Cylinder` | Pipes, trunks, poles, or columns               | Axis point, axis direction, radius, and inliers |
| `Cuboid`   | Box-like objects                               | Center, extents, axes, and inliers              |

The exact return values of each fitter are documented in [Fit Results](https://leomariga.github.io/pyRANSAC-3D/api-documentation/fit-results/index.md), but every one of them returns the indexes of the inlier points. Those indexes are useful for extracting the part of your point cloud that matched the fitted shape.

pyRANSAC-3D expects point clouds as NumPy-compatible arrays with shape `(N, 3)`

## First Fit: Plane

Planes are a good first example because many point clouds contain floors, walls, tables, or other flat surfaces.

```
import pyransac3d as pyrsc

# Replace this with your own NumPy array with shape (N, 3).
points = pyrsc.ShapeGenerator(seed=0).plane(
    [1, 2, 3],
    [0, 0, 1],
    size=6.0,
    n_points=500,
    noise=0.02,
)

plane = pyrsc.Plane()
equation, inliers = plane.fit(points, thresh=0.05)

print(f"Plane equation (Ax + By + Cz + D): {equation}")
print(f"Inliers: {len(inliers)} of {len(points)}")
```

The plane result is returned as `[A, B, C, D]` for:

```
Ax + By + Cz + D = 0
```

## Reading the Result

Every `fit` returns a named tuple, one type per shape, so you can either unpack it as above or keep it and read each value by name. Reading by name is clearer for shapes with several parameters, where the order is easy to forget:

```
import pyransac3d as pyrsc

points = pyrsc.ShapeGenerator(seed=0).cylinder([1, 2, 3], [0, 0, 1], radius=2.0, height=6.0, n_points=800, noise=0.02)

result = pyrsc.Cylinder().fit(points, thresh=0.15)

print(f"Center: {result.center}")
print(f"Axis: {result.axis}")
print(f"Radius: {result.radius}")
print(f"Inliers: {len(result.inliers)} of {len(points)}")
```

The values a fit measures but does not return, such as the distance from every point to the fitted shape, are kept on the fitter object instead. They are listed in the attributes of each shape in the API pages:

```
cylinder = pyrsc.Cylinder()
cylinder.fit(points, thresh=0.15)

print(f"Distance of the first point to the cylinder hull: {cylinder.distances[0]}")
```

## Fit a Sphere

Use a sphere when you expect points around a ball-like object.

```
import pyransac3d as pyrsc

points = pyrsc.ShapeGenerator(seed=0).sphere(
    [1, 2, 3],
    radius=3.0,
    n_points=500,
    noise=0.02,
)

sphere = pyrsc.Sphere()
center, radius, inliers = sphere.fit(points, thresh=0.1)

print(f"Center: {center}")
print(f"Radius: {radius}")
print(f"Inliers: {len(inliers)} of {len(points)}")
```

## Fit a Cuboid

Cuboids are useful for boxes, packages, furniture, and other objects with rectangular faces.

```
import pyransac3d as pyrsc

points = pyrsc.ShapeGenerator(seed=0).cuboid([1, 2, 3], [4, 3, 2], n_points=1000, noise=0.01)

cuboid = pyrsc.Cuboid()
center, extents, axes, inliers = cuboid.fit(points, thresh=0.05)

print(f"Center: {center}")
print(f"Extents: {extents}")
print(f"Axes: {axes}")
print(f"Inliers: {len(inliers)} of {len(points)}")
```

## Callbacks

Every fitter accepts an optional `callback`. The callback runs during RANSAC and receives a `state` dictionary for the current iteration, including values such as `iteration` and `best_inliers`.

Use a callback when you want to react while fitting is still running: stop early with your own criteria, log intermediate values, inspect candidate parameters, or update a visualization.

Every key of the `state` dictionary is documented in [Fit State](https://leomariga.github.io/pyRANSAC-3D/api-documentation/fit-state/index.md).

### Early Stop With Your Own Criteria

If the callback returns `True`, the fit stops early and returns the best result found so far. This is useful when you already know what a good enough result looks like:

```
import pyransac3d as pyrsc

points = pyrsc.ShapeGenerator(seed=0).plane(
    [1, 2, 3],
    [0, 0, 1],
    size=6.0,
    n_points=500,
    noise=0.02,
)
n_points = len(points)
target_inlier_ratio = 0.8


def stop_when_enough_points_fit(state):
    inlier_ratio = len(state["best_inliers"]) / n_points
    if state["is_best"]:
        print(
            f"iteration {state['iteration']:>4} | "
            f"best inliers: {len(state['best_inliers']):>4} ({inlier_ratio:.1%})"
        )
    return inlier_ratio >= target_inlier_ratio


plane = pyrsc.Plane()
equation, inliers = plane.fit(
    points,
    thresh=0.05,
    maxIteration=1000,
    callback=stop_when_enough_points_fit,
)

print(f"Stopped with {len(inliers)} inliers")
print(f"Plane equation: {equation}")
```

See the full runnable [early stop callback example](https://github.com/leomariga/pyRANSAC-3D/blob/master/examples/early_stop_callback.py).

### Inspect Progress or Build Visualizations

Callbacks can also be used without stopping the fit. Return `False` or `None` to keep RANSAC running, and use the `state` values to log progress, compare candidate parameters, or update a live plot.

```
import pyransac3d as pyrsc

points = pyrsc.ShapeGenerator(seed=0).plane(
    [1, 2, 3],
    [0, 0, 1],
    size=6.0,
    n_points=500,
    noise=0.02,
)


def print_progress(state):
    if state["is_best"]:
        print(
            f"iteration {state['iteration']:>4} | "
            f"best inliers: {len(state['best_inliers']):>4} | "
            f"equation: {state['best_model']['equation']}"
        )
    return False


plane = pyrsc.Plane()
equation, inliers = plane.fit(points, thresh=0.05, maxIteration=1000, callback=print_progress)
```

For a visual version of the same idea, see the [plane animation example](https://github.com/leomariga/pyRANSAC-3D/blob/master/examples/plane_animation.py), where the callback updates the Open3D point colors on every iteration.

## Visualize With Open3D

pyRANSAC-3D only needs NumPy to fit shapes, but Open3D is useful when you want to inspect the result visually.

Install Open3D if you are running a visualization example:

```
pip install open3d
```

This example fits a plane, paints the inliers red, keeps the remaining points in their original color, and opens an Open3D window:

```
import numpy as np
import open3d as o3d
import pyransac3d as pyrsc

points = pyrsc.ShapeGenerator(seed=0).plane(
    [1, 2, 3],
    [0, 0, 1],
    size=6.0,
    n_points=500,
    noise=0.02,
)

point_cloud = o3d.geometry.PointCloud()
point_cloud.points = o3d.utility.Vector3dVector(points)

plane = pyrsc.Plane()
equation, inliers = plane.fit(points, thresh=0.05)

inlier_cloud = point_cloud.select_by_index(inliers)
inlier_cloud.paint_uniform_color([1.0, 0.0, 0.0])

outlier_cloud = point_cloud.select_by_index(inliers, invert=True)
outlier_cloud.paint_uniform_color([0.6, 0.6, 0.6])

o3d.visualization.draw_geometries([outlier_cloud, inlier_cloud])
```

The repository has richer Open3D examples that also draw fitted meshes, wireframes, bounding boxes, and live RANSAC animations.

## Using Your Own Data

If your points are already in memory, pass them directly:

```
import numpy as np
import pyransac3d as pyrsc

points = np.asarray(my_points)

plane = pyrsc.Plane()
equation, inliers = plane.fit(points, thresh=0.05)
```

For best results:

- Make sure the array has shape `(N, 3)`.
- Choose `thresh` based on your point cloud units and expected noise.
- Inspect `len(inliers)` to see how much of the cloud matched the fitted shape.
- Try a few thresholds if the fit is too strict or too permissive.

## Where To Go Next

- Browse the [examples folder](https://github.com/leomariga/pyRANSAC-3D/tree/master/examples) for runnable scripts for every shape.
- Start with the simple examples: [Point](https://github.com/leomariga/pyRANSAC-3D/blob/master/examples/point_simple.py), [Line](https://github.com/leomariga/pyRANSAC-3D/blob/master/examples/line_simple.py), [Plane](https://github.com/leomariga/pyRANSAC-3D/blob/master/examples/plane_simple.py), [Circle](https://github.com/leomariga/pyRANSAC-3D/blob/master/examples/circle_simple.py), [Sphere](https://github.com/leomariga/pyRANSAC-3D/blob/master/examples/sphere_simple.py), [Cylinder](https://github.com/leomariga/pyRANSAC-3D/blob/master/examples/cylinder_simple.py), and [Cuboid](https://github.com/leomariga/pyRANSAC-3D/blob/master/examples/cuboid_simple.py).
- Try the Open3D visual examples: [Point](https://github.com/leomariga/pyRANSAC-3D/blob/master/examples/point_visual.py), [Line](https://github.com/leomariga/pyRANSAC-3D/blob/master/examples/line_visual.py), [Plane](https://github.com/leomariga/pyRANSAC-3D/blob/master/examples/plane_visual.py), [Circle](https://github.com/leomariga/pyRANSAC-3D/blob/master/examples/circle_visual.py), [Sphere](https://github.com/leomariga/pyRANSAC-3D/blob/master/examples/sphere_visual.py), [Cylinder](https://github.com/leomariga/pyRANSAC-3D/blob/master/examples/cylinder_visual.py), and [Cuboid](https://github.com/leomariga/pyRANSAC-3D/blob/master/examples/cuboid_visual.py).
- Use the animation examples to see callbacks in action: [Point animation](https://github.com/leomariga/pyRANSAC-3D/blob/master/examples/point_animation.py), [Line animation](https://github.com/leomariga/pyRANSAC-3D/blob/master/examples/line_animation.py), [Plane animation](https://github.com/leomariga/pyRANSAC-3D/blob/master/examples/plane_animation.py), [Circle animation](https://github.com/leomariga/pyRANSAC-3D/blob/master/examples/circle_animation.py), [Sphere animation](https://github.com/leomariga/pyRANSAC-3D/blob/master/examples/sphere_animation.py), [Cylinder animation](https://github.com/leomariga/pyRANSAC-3D/blob/master/examples/cylinder_animation.py), and [Cuboid animation](https://github.com/leomariga/pyRANSAC-3D/blob/master/examples/cuboid_animation.py).
- Open the API pages for each fitter: [Plane](https://leomariga.github.io/pyRANSAC-3D/api-documentation/plane/index.md), [Sphere](https://leomariga.github.io/pyRANSAC-3D/api-documentation/sphere/index.md), [Cylinder](https://leomariga.github.io/pyRANSAC-3D/api-documentation/cylinder/index.md), [Cuboid](https://leomariga.github.io/pyRANSAC-3D/api-documentation/cuboid/index.md), [Line](https://leomariga.github.io/pyRANSAC-3D/api-documentation/line/index.md), [Circle](https://leomariga.github.io/pyRANSAC-3D/api-documentation/circle/index.md), and [Point](https://leomariga.github.io/pyRANSAC-3D/api-documentation/point/index.md).
- Use [Shape Generator](https://leomariga.github.io/pyRANSAC-3D/api-documentation/shape-generator/index.md) to create synthetic point clouds while learning or testing.
