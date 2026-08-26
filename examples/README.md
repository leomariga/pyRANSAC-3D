# Examples

Every shape has 3 examples, and the name of the file says what kind of example it is:

- `_simple`: the shortest way to call the fitter. Only NumPy, no window
- `_visual`: fits the shape and plots the result with [Open3D](http://www.open3d.org/), which opens a window
- `_animation`: plots every candidate with Open3D while RANSAC is running, using the `callback` of the fitter

Run one at a time:

```sh
uv run python examples/plane_simple.py
```

## Simple

They build the point cloud with `pyransac3d.ShapeGenerator`, fit it and print the parameters found.

| Example | Fits |
| --- | --- |
| [point_simple.py](point_simple.py) | The coordinate with more neighbors around |
| [line_simple.py](line_simple.py) | The direction of a line and a point on it |
| [plane_simple.py](plane_simple.py) | The equation of a plane |
| [circle_simple.py](circle_simple.py) | The center, the axis and the radius of a circle |
| [sphere_simple.py](sphere_simple.py) | The center and the radius of a sphere |
| [cylinder_simple.py](cylinder_simple.py) | The center, the axis and the radius of a cylinder |
| [cone_simple.py](cone_simple.py) | The apex, the axis and the half-angle of a cone |
| [cuboid_simple.py](cuboid_simple.py) | The center, the extents and the axes of a cuboid |

## Visual

| Example | What it plots |
| --- | --- |
| [point_visual.py](point_visual.py) | The inliers of the point and a marker on it |
| [line_visual.py](line_visual.py) | The inliers of the line and a tube along it |
| [plane_visual.py](plane_visual.py) | The inliers of a plane of `dataset/caixa.ply` and their bounding boxes |
| [circle_visual.py](circle_visual.py) | The inliers of the circle and a torus on it |
| [sphere_visual.py](sphere_visual.py) | The inliers of the sphere and its wireframe |
| [cylinder_visual.py](cylinder_visual.py) | The inliers of the cylinder, its axes and a disk on it |
| [cone_visual.py](cone_visual.py) | The inliers of the cone and its wireframe, fitted with the normals Open3D estimated |
| [cuboid_visual.py](cuboid_visual.py) | The inliers of the box of `dataset/caixa.ply` and its 8 corners |

## Animation

The current inliers are red, the best inliers so far are green, the sampled points are blue, and the shape of the best candidate is drawn in blue while it changes.

| Example | Point cloud |
| --- | --- |
| [point_animation.py](point_animation.py) | Generated cluster, with the ball of neighbors of the best point |
| [line_animation.py](line_animation.py) | Generated line, with the segment of the best candidate |
| [plane_animation.py](plane_animation.py) | Real cloud `dataset/caixa.ply` |
| [circle_animation.py](circle_animation.py) | Generated circle, with the ring of the best candidate |
| [sphere_animation.py](sphere_animation.py) | Generated sphere, with the wireframe of the best candidate |
| [cylinder_animation.py](cylinder_animation.py) | Generated cylinder, with the wireframe of the best candidate |
| [cone_animation.py](cone_animation.py) | Generated cone, with the wireframe of the best candidate |
| [cuboid_animation.py](cuboid_animation.py) | Real cloud `dataset/caixa.ply`, with the box of the best candidate |

## Others

| Example | What it does |
| --- | --- |
| [early_stop_callback.py](early_stop_callback.py) | Uses the `callback` to stop the fitting as soon as enough points are explained, instead of plotting |

The `dataset` folder has the real point clouds used by the examples which do not generate their own.
