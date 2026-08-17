from pathlib import Path
from typing import Any

import numpy as np
import open3d as o3d

import pyransac3d as pyrsc

DATASET_DIR = Path(__file__).resolve().parent / "dataset"

# Fit a plane but stop as soon as a candidate explains a large enough
# fraction of the point cloud, instead of always running maxIteration times.
pcd_load = o3d.io.read_point_cloud(str(DATASET_DIR / "caixa.ply"))
points = np.asarray(pcd_load.points)
n_points = points.shape[0]

target_inlier_ratio = 0.3


def early_stop_callback(state: dict[str, Any]) -> bool:
    inlier_ratio = len(state["best_inliers"]) / n_points
    if state["is_best"]:
        print(f"iteration {state['iteration']:>4} | best inliers: {len(state['best_inliers']):>6} ({inlier_ratio:.1%})")
    return inlier_ratio >= target_inlier_ratio


plane1 = pyrsc.Plane()
best_eq, best_inliers = plane1.fit(points, thresh=0.01, maxIteration=1000, callback=early_stop_callback)

print(f"Stopped early with {len(best_inliers)} inliers ({len(best_inliers) / n_points:.1%} of the cloud)")
print(f"Plane equation: {best_eq}")
