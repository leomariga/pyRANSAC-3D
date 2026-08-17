import pyransac3d as pyrsc

# The generator builds a noisy cloud of a shape we already know, so there is no file to load here
points = pyrsc.ShapeGenerator(seed=0).circle([1, 2, 3], [0, 0, 1], radius=3.0, n_points=500, noise=0.02)

circle = pyrsc.Circle()
center, axis, radius, inliers = circle.fit(points, thresh=0.1)

print(f"Center: {center}")
print(f"Axis: {axis}")
print(f"Radius: {radius}")
print(f"Inliers: {len(inliers)} of {len(points)}")
