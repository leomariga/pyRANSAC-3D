import pyransac3d as pyrsc

# The generator builds a noisy cloud of a shape we already know, so there is no file to load here
points = pyrsc.ShapeGenerator(seed=0).point([1, 2, 3], n_points=500, noise=0.05)

point = pyrsc.Point()
center, inliers = point.fit(points, thresh=0.2)

print(f"Center: {center}")
print(f"Inliers: {len(inliers)} of {len(points)}")
