# Draft from github post
from concurrent.futures import ThreadPoolExecutor

import numpy as np
from random import Random
from typing import List, Tuple
from abc import ABC, abstractmethod
import time

class BaseRansac(ABC):
    def __init__(self, seed=None, n_workers=None):
        self.random = Random(seed)
        self.n_workers = n_workers

    def fit(self, points: np.ndarray, thresh: float = 0.05, max_iteration: int = 5000) -> Tuple[List[float], List[float]]:
        """
        :param points: A numpy array of points, of shape (# points, 3)
        :param thresh: The distance threshold to include points as inliers
        :param max_iteration: How many (parallel) Ransac iterations to run
        :returns:
            best_eq: A list of integers representing the best 'equation' for the primitive shape.
            best_inliers: A list of indices of points that fit the shape.
        """
        best_eq = []
        best_inliers = []

        jobs = ((points, float(thresh)) for _ in range(max_iteration))
        # We can use a with statement to ensure threads are cleaned up promptly
        with ThreadPoolExecutor(max_workers=self.n_workers) as executor:
            for eq, point_id_inliers in executor.map(self.iteration, *zip(*jobs)):
                if len(point_id_inliers) > len(best_inliers):
                    best_eq = eq
                    best_inliers = point_id_inliers
        return best_eq, best_inliers

    @abstractmethod
    def iteration(self, points: np.ndarray, thresh: float) -> Tuple[List[float], List[float]]:
        pass

class PrimitiveTest(BaseRansac):
    def iteration(self, points: np.ndarray, thresh: float)-> Tuple[List[float], List[float]]:
        time.sleep(0.1)
        return list(points), list(points*thresh)

prim = PrimitiveTest(3, 5000)
a1, a2 = prim.fit(np.asarray([0, 2, 6]), 5)
print(a1)
print(a2)