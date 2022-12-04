import math

import numpy as np
from EasyGA import GA
import random
from sklearn import cluster as skc
from sklearn import metrics


class My_GA(GA):
    """Example GA with custom initialize_population method."""
    def __init__(self, num_of_shots: int, fitness_function: int, num_generation: int, num_population: int):
        GA.__init__(self)
        self.pcds_outliers = []
        self.pcds_inliers = []
        self.pcds_projected_outliers = []
        self.generation_goal = num_generation
        self.population_size = num_population
        self.num_of_shots = num_of_shots
        self.fitness_funcion_num = fitness_function

    def fitting_function(self, chromosome):
        cloud_number = random.randint(0, self.num_of_shots - 1)
        points = np.asarray(self.pcds_projected_outliers[cloud_number].points)
        clustering = \
            skc.DBSCAN(eps=chromosome[0].value, min_samples=chromosome[1].value).fit(points)
        num_of_clusters = len(np.unique(np.asarray(clustering.labels_)))

        if self.fitness_funcion_num == 1:   # silhouette
            self.target_fitness_type = 'max'
            silhouette_score = float(metrics.silhouette_score(points, clustering.labels_))

            if silhouette_score < 0.1 or num_of_clusters < 20 or num_of_clusters > 50:
                fitness = -1
            else:
                fitness = silhouette_score
            # fitness = silhouette_score
            return fitness
        elif self.fitness_funcion_num == 2:  # davies-bouldin
            self.target_fitness_type = 'min'
            davies_bouldin_score = float(metrics.davies_bouldin_score(points, clustering.labels_))

            if num_of_clusters < 7 or num_of_clusters > 50:  # need to check
                fitness = 150
            else:
                fitness = davies_bouldin_score
            # fitness = davies_bouldin_score
            return fitness
        elif self.fitness_funcion_num == 3:  # calinski-harabasz
            self.target_fitness_type = 'max'
            calinski_harabasz_score = float(metrics.calinski_harabasz_score(points, clustering.labels_))

            if calinski_harabasz_score < 0.1 or num_of_clusters < 14 or num_of_clusters > 50:
                fitness = 0
            else:
                fitness = calinski_harabasz_score
            return fitness

    def initialize_population(self):
        a_eps = 0.1
        b_eps = 1.3

        a_min_samples = 1
        b_min_samples = self.population_size/2

        h_eps = (b_eps - a_eps)/self.population_size
        h_min_samples = round((b_min_samples - a_min_samples)/self.population_size)
        if h_min_samples == 0:
            h_min_samples += 1

        self.population = self.make_population(
            [
                a_eps + i * h_eps,
                (i % 10) + 1
            ]
            for i
            in range(self.population_size)
        )

