# Draft from github post
from concurrent.futures import ThreadPoolExecutor

import numpy as np
from random import Random
from typing import List, Tuple
from abc import ABC, abstractmethod


class BaseRansac(ABC):
    def __init__(self, seed=None, n_workers=None):
        self.executor = ThreadPoolExecutor(max_workers=None)
        self.random = Random(seed)

    def fit(self, points: np.ndarray, thresh: float = 0.05, max_iteration: int = 5000) -> Tuple[List[int], List[int]]:
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
        jobs = ((self.random, points, float(thresh)) for _ in range(max_iteration))
        for eq, point_id_inliers in self.executor.map(self.iteration, *zip(*jobs)):
            if len(point_id_inliers) > len(best_inliers):
                best_eq = eq
                best_inliers = point_id_inliers
        return best_eq, best_inliers

    @staticmethod
    @abstractmethod
    def iteration(self, random: Random, points: np.ndarray, thresh: float) -> Tuple[List[int], List[int]]:
        pass