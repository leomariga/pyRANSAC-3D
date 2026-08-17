import pyransac3d as pyrsc

# The generator builds a noisy cloud of a shape we already know, so there is no file to load here
points = pyrsc.ShapeGenerator(seed=0).plane([1, 2, 3], [0, 0, 1], size=6.0, n_points=500, noise=0.02)

plane = pyrsc.Plane()
equation, inliers = plane.fit(points, thresh=0.05)

print(f"Equation (Ax + By + Cz + D): {equation}")
print(f"Inliers: {len(inliers)} of {len(points)}")
