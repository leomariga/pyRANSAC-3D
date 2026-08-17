import pyransac3d as pyrsc

# The generator builds a noisy cloud of a shape we already know, so there is no file to load here
points = pyrsc.ShapeGenerator(seed=0).cuboid([1, 2, 3], [4, 3, 2], n_points=1000, noise=0.01)

cuboid = pyrsc.Cuboid()
center, extents, axes, inliers = cuboid.fit(points, thresh=0.05)

print(f"Center: {center}")
print(f"Extents: {extents}")
print(f"Axes: {axes}")
print(f"Inliers: {len(inliers)} of {len(points)}")
