# pyRANSAC-3D: RANSAC shape fitting for Python point clouds

<div align="center">
  <img src="https://raw.githubusercontent.com/leomariga/pyRANSAC-3D/master/doc/logo.png"><br>
</div>

-----------------
[![Tests](https://github.com/leomariga/pyRANSAC-3D/actions/workflows/tests.yml/badge.svg?branch=master)](https://github.com/leomariga/pyRANSAC-3D/actions/workflows/tests.yml)
[![DOI](https://zenodo.org/badge/287829485.svg)](https://zenodo.org/badge/latestdoi/287829485)
[![PyPI Latest Release](https://img.shields.io/pypi/v/pyransac3d.svg)](https://pypi.org/project/pyransac3d/)
[![License](https://img.shields.io/pypi/l/pyransac3d.svg)](https://github.com/leomariga/pyransac3d/blob/master/LICENSE)


## What is pyRANSAC-3D?
**pyRANSAC-3D** is an open source Python library for detecting and fitting 3D geometric primitives in point clouds using the Random Sample Consensus (RANSAC) algorithm. It works with NumPy-compatible arrays and supports planes, lines, circles, spheres, cylinders, cuboids, and point clusters.

Use pyRANSAC-3D for point-cloud segmentation and shape detection in applications such as LiDAR processing, 3D SLAM, 3D reconstruction, computer vision, and object tracking.

<div align="center">
  <img src="https://raw.githubusercontent.com/leomariga/pyRANSAC-3D/master/doc/plano.gif"><br>
</div>

### Features
 - [Plane](https://leomariga.github.io/pyRANSAC-3D/api-documentation/plane/)
 - [Cylinder](https://leomariga.github.io/pyRANSAC-3D/api-documentation/cylinder/)
 - [Cuboid](https://leomariga.github.io/pyRANSAC-3D/api-documentation/cuboid/)
 - [Sphere](https://leomariga.github.io/pyRANSAC-3D/api-documentation/sphere/)
 - [Line](https://leomariga.github.io/pyRANSAC-3D/api-documentation/line/)
 - [Circle](https://leomariga.github.io/pyRANSAC-3D/api-documentation/circle/)
 - [Point](https://leomariga.github.io/pyRANSAC-3D/api-documentation/point/)

## Installation
Requirement: NumPy

Install from [PyPI](https://pypi.org/project/pyransac3d/):

```sh
pip install pyransac3d
```

For a complete introduction, shape-selection guide, and runnable examples, see the [Getting Started guide](https://leomariga.github.io/pyRANSAC-3D/getting-started/).

## Quick start

### Example 1 - Planar RANSAC

```python
import pyransac3d as pyrsc

points = load_points(.) # Load your point cloud as a NumPy array with shape (N, 3)

plane1 = pyrsc.Plane()
best_eq, best_inliers = plane1.fit(points, 0.01)
```

The result is the plane equation `Ax + By + Cz + D = 0`:
`[0.720, -0.253, 0.646, 1.100]`

### Example 2 - Spherical RANSAC

To fit a noisy sphere with `r = 5` centered at the origin, use:

```python
import pyransac3d as pyrsc

points = load_points(.) # Load your point cloud as a NumPy array with shape (N, 3)

sph = pyrsc.Sphere()
center, radius, inliers = sph.fit(points, thresh=0.4)
```

Results:
```text
center: [0.010462385575072288, -0.2855090643954039, 0.02867848979091283]
radius: 5.085218633039647
```

![3D Sphere](https://raw.githubusercontent.com/leomariga/pyRANSAC-3D/master/doc/sphere.gif "3D Sphere")

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
@software{Mariga_pyRANSAC-3D_2022,
  author = {Mariga, Leonardo},
  doi = {10.5281/zenodo.7212567},
  month = {10},
  title = {{pyRANSAC-3D}},
  url = {https://github.com/leomariga/pyRANSAC-3D},
  version = {v0.6.0},
  year = {2022}
}
```

## Contributing is awesome!

See [CONTRIBUTING](https://github.com/leomariga/pyRANSAC-3D/blob/master/CONTRIBUTING.md)




## Contact

Developed with :heart: by the internet


Maintainer: [Leonardo Mariga](https://github.com/leomariga)

Did you like it? Remember to click the :star2: button.
