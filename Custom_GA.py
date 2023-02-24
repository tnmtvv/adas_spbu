from typing import List

import numpy as np
import open3d
from EasyGA import GA
from sklearn import cluster as skc
from sklearn import metrics
# from ClustersFeatures import *


class MyGA(GA):

    def __init__(self, num_of_shots: int, fitness_function: int, num_generation: int, num_population: int):
        GA.__init__(self)
        self.pcds_inliers: List[open3d.geometry.PointCloud] = []
        self.pcds_projected_outliers: List[open3d.geometry.PointCloud] = []
        self.pcds_cropped_outliers: List[open3d.geometry.PointCloud] = []
        self.generation_goal = num_generation
        self.population_size = num_population
        self.num_of_shots = num_of_shots
        self.fitness_funcion_num = fitness_function

    def fitting_function(self, chromosome):  # custom fitness function
        silhouette = 1
        davies_bouldin = 2
        calinski_harabasz = 3

        scores = []
        for cloud in self.pcds_cropped_outliers:
            points = np.asarray(cloud.points)
            clustering = \
                skc.DBSCAN(eps=chromosome[0].value, min_samples=chromosome[1].value).fit(points)
            num_of_clusters = len(np.unique(np.asarray(clustering.labels_)))

            if self.fitness_funcion_num == silhouette:
                self.target_fitness_type = 'max'

                if num_of_clusters < 20:
                    fitness = -1
                else:
                    fitness = float(metrics.silhouette_score(points, clustering.labels_))
                scores.append(fitness)

            elif self.fitness_funcion_num == davies_bouldin:
                self.target_fitness_type = 'min'
                davies_bouldin_score = float(metrics.davies_bouldin_score(points, clustering.labels_))

                if num_of_clusters < 7 or num_of_clusters > 50:  # need to check
                    fitness = 150
                else:
                    fitness = davies_bouldin_score
                scores.append(fitness)

            elif self.fitness_funcion_num == calinski_harabasz:
                self.target_fitness_type = 'max'
                calinski_harabasz_score = float(metrics.calinski_harabasz_score(points, clustering.labels_))

                if calinski_harabasz_score < 0.1 or num_of_clusters < 14:
                    fitness = 0
                else:
                    fitness = calinski_harabasz_score
                scores.append(fitness)
        return np.mean(scores)

    def initialize_population(self):
        a_eps = 0.1  # minimum valid distance
        b_eps = 1.3  # maximum valid distance

        a_min_samples = 1
        b_min_samples = self.population_size / 2

        h_eps = (b_eps - a_eps) / self.population_size
        h_min_samples = round((b_min_samples - a_min_samples) / self.population_size)
        if h_min_samples == 0:
            h_min_samples += 1

        self.population = self.make_population(
            [
                a_eps + i * h_eps,
                (i % 10) + 1
            ]
            for i in range(self.population_size)
        )

    def ga_run(self, verbose):
        while self.active():
            self.evolve(1)
            if verbose:
                print('------------------------------')
                self.print_population()
                self.print_best_chromosome()
                self.print_worst_chromosome()
                print('------------------------------')

        return self.population[0].gene_value_list
