import math

import numpy as np
from EasyGA import GA
import random
from sklearn import cluster as skc
from sklearn import metrics


class My_GA(GA):
    """Example GA with custom initialize_population method."""
    def __init__(self, num_of_shots: int, metric: int):
        GA.__init__(self)
        self.pcds_outliers = []
        self.pcds_inliers = []
        self.num_of_shots = num_of_shots
        self.metric_num = metric

    def fitting_function(self, chromosome):
        cloud_number = random.randint(0, len(self.pcds_outliers)-1)
        points = np.asarray(self.pcds_outliers[cloud_number].points)
        clustering = \
            skc.DBSCAN(eps=chromosome[0].value, min_samples=chromosome[1].value).fit(points)

        silhouette_score = float(metrics.silhouette_score(points, clustering.labels_))
        num_of_clusters = len(np.unique(np.asarray(clustering.labels_)))

        if silhouette_score < 0.1 or num_of_clusters < 7 or num_of_clusters > 150:
            fitness = 0
        else:
            fitness = silhouette_score
        return fitness

    def initialize_population(self):
        a_eps = 0.1
        b_eps = 5

        a_min_samples = 1
        b_min_samples = self.population_size

        h_eps = (b_eps - a_eps)/self.population_size
        h_min_samples = round((b_min_samples - a_min_samples)/self.population_size)
        if h_min_samples == 0:
            h_min_samples += 1

        self.population = self.make_population(
            [
                a_eps + i * h_eps,
                i + 1
            ]
            for i
            in range(self.population_size)
        )

