______________________________________________________________________

## What is pyRANSAC-3D?

***pyRANSAC-3D*** is an open source Python implementation of the Random Sample Consensus (RANSAC) method. It fits primitive shapes such as planes, cuboids, and cylinders to point clouds for applications including 3D SLAM, 3D reconstruction, object tracking, and more.

### Features

- [Plane](https://leomariga.github.io/pyRANSAC-3D/api-documentation/plane/)
- [Cylinder](https://leomariga.github.io/pyRANSAC-3D/api-documentation/cylinder/)
- [Cone](https://leomariga.github.io/pyRANSAC-3D/api-documentation/cone/)
- [Cuboid](https://leomariga.github.io/pyRANSAC-3D/api-documentation/cuboid/)
- [Sphere](https://leomariga.github.io/pyRANSAC-3D/api-documentation/sphere/)
- [Line](https://leomariga.github.io/pyRANSAC-3D/api-documentation/line/)
- [Circle](https://leomariga.github.io/pyRANSAC-3D/api-documentation/circle/)
- [Point](https://leomariga.github.io/pyRANSAC-3D/api-documentation/point/)

## Installation

Requirement: NumPy

Install from [PyPI](https://pypi.org/project/pyransac3d/):

```
pip install pyransac3d
```

For a complete introduction see the [Getting Started guide](https://leomariga.github.io/pyRANSAC-3D/getting-started/).

## Quick start

### Example 1 - Planar RANSAC

```
import pyransac3d as pyrsc

points = load_points(.) # Load your point cloud as a NumPy array with shape (N, 3)

plane1 = pyrsc.Plane()
best_eq, best_inliers = plane1.fit(points, 0.01)
```

The result is the plane equation `Ax + By + Cz + D = 0`: `[0.720, -0.253, 0.646, 1.100]`

### Example 2 - Spherical RANSAC

To fit a noisy sphere with `r = 5` centered at the origin, use:

```
import pyransac3d as pyrsc

points = load_points(.) # Load your point cloud as a NumPy array with shape (N, 3)

sph = pyrsc.Sphere()
center, radius, inliers = sph.fit(points, thresh=0.4)
```

Results:

```
center: [0.010462385575072288, -0.2855090643954039, 0.02867848979091283]
radius: 5.085218633039647
```

### More examples

Runnable examples of every shape are in the [examples folder](https://github.com/leomariga/pyRANSAC-3D/tree/master/examples).

## When should I use pyRANSAC-3D?

Use pyRANSAC-3D when you need to:

- Fit a plane to a 3D point cloud in Python, such as a floor, wall, roof, or tabletop.
- Detect cylinders in LiDAR or depth-sensor data, such as pipes, poles, trunks, or columns.
- Segment geometric shapes from noisy point clouds with RANSAC.
- Fit spheres, cuboids, circles, lines, or point clusters from NumPy-compatible `(N, 3)` arrays.
- Combine lightweight NumPy-based shape fitting with optional [Open3D](https://www.open3d.org/) visualization.

## Documentation and other links

- Read the [documentation](https://leomariga.github.io/pyRANSAC-3D/).
- Browse the source code in the [GitHub repository](https://github.com/leomariga/pyRANSAC-3D).
- Install the [PyPI package](https://pypi.org/project/pyransac3d/).
- Explore the [animation examples](https://github.com/leomariga/pyRANSAC-3D/tree/master/examples#animation) to watch each RANSAC fitter update through its callback.

## License

[Apache 2.0](https://github.com/leomariga/pyRANSAC-3D/blob/master/LICENSE)

## Citation

Was this repository useful for your work? Please cite it:

```
@software{Mariga_pyRANSAC-3D,
  author = {Mariga, Leonardo},
  doi = {10.5281/zenodo.7212567},
  month = {8},
  title = {{pyRANSAC-3D}},
  url = {https://github.com/leomariga/pyRANSAC-3D},
  version = {v0.7.0},
  year = {2026}
}
```

## Contributing is awesome!

See [CONTRIBUTING](https://github.com/leomariga/pyRANSAC-3D/blob/master/CONTRIBUTING.md)

## Contact

Developed with :heart: by the internet

Maintainer: [Leonardo Mariga](https://github.com/leomariga)

Did you like it? Remember to click the :star2: button.
