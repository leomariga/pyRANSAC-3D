import numpy as np

import pyransac3d as pyrsc

# The generator builds a noisy cloud of a shape we already know, so there is no file to load here
points = pyrsc.ShapeGenerator(seed=0).cone(
    [1, 2, 3], [0, 0, 1], angle=np.deg2rad(25.0), height=6.0, n_points=500, noise=0.02
)

# A cone is found from the normals of the surface. They are estimated from the cloud when they are
# not given, so pass the ones you already have to skip the nearest neighbor search
cone = pyrsc.Cone()
apex, axis, angle, inliers = cone.fit(points, thresh=0.1, maxIteration=1000)

print(f"Apex: {apex}")
print(f"Axis: {axis}")
print(f"Angle (deg): {np.rad2deg(angle)}")
print(f"Inliers: {len(inliers)} of {len(points)}")
