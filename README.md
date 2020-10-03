

-----------------
[![PyPI Latest Release](https://img.shields.io/pypi/v/pyransac3d.svg?style=for-the-badge)](https://pypi.org/project/pyransac3d/)
[![License](https://img.shields.io/pypi/l/pyransac3d.svg?style=for-the-badge)](https://github.com/leomariga/pyransac3d/blob/master/LICENSE)

## What is pyRANSAC-3D?
**_pyRANSAC-3D_** is an open source implementation of Random sample consensus (RANSAC) method. It fits primitive shapes such as planes, cuboids and cylinder in a point cloud to many aplications: 3D slam, 3D reconstruction, object tracking and many others. 

<div align="center">
  <img src="https://raw.githubusercontent.com/leomariga/pyRANSAC-3D/master/doc/plano.gif"><br>
</div>

#### Features:
 - Cylinder
 - Plane
 - Cuboid
 - Line


## Installation
Requirements: Numpy

Install with [Pypi](https://pypi.org/project/pyransac3d/):

```sh
pip3 install pyransac3d
```

### Take a look: 

##### Example 1 - Planar RANSAC

``` python
import pyransac3d as pyrsc

points = load_points(.) # Load your point cloud as a numpy array (N, 3)

plane1 = pyrsc.Plane()
best_eq, best_inliers = plane1.fit(points, 0.01)

```

Results in the plane equation Ax+By+Cz+D:
`[1, 0.5, 2, 0]`


## Documentation & other links
 - The [documentation is this Ṕage](https://leomariga.github.io/pyRANSAC-3D/).
 - Source code in the [Github repository](https://github.com/leomariga/pyRANSAC-3D).
 - [Pypi pakage installer](https://pypi.org/project/pyransac3d/)


## License
[Apache 2.0](https://github.com/leomariga/pyRANSAC-3D/blob/master/LICENSE)

## Contributing is awesome!

See [CONTRIBUTING](https://github.com/leomariga/pyRANSAC-3D/blob/master/CONTRIBUTING.md)




## Contact

Developed with :heart: by the internet


Mainteiner: [Leonardo Mariga](https://github.com/leomariga) 

leomariga@gmail.com

Did you like it? Remember to click on :star2: button.
