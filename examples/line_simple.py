import pyransac3d as pyrsc

# The generator builds a noisy cloud of a shape we already know, so there is no file to load here
points = pyrsc.ShapeGenerator(seed=0).line([1, 2, 3], [1, 1, 0], length=10.0, n_points=500, noise=0.05)

line = pyrsc.Line()
direction, anchor, inliers = line.fit(points, thresh=0.2)

print(f"Direction: {direction}")
print(f"Point of the line: {anchor}")
print(f"Inliers: {len(inliers)} of {len(points)}")
