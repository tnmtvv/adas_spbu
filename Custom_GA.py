import random
from typing import List

import numpy as np
import open3d
from EasyGA import GA
from cdbw import CDbw
from sklearn import cluster as skc
from sklearn import metrics
import open3d as o3d

import SemanticKitti_methods
from DBCV import DBCV

from LabeledPcd import LabeledPcd


# from ClustersFeatures import *


class MyGA(GA):
    def __init__(
        self,
        num_of_shots: int,
        fitness_function: int,
        num_generation: int,
        num_population: int,
    ):
        GA.__init__(self)
        self.pcds_inliers: List[open3d.geometry.PointCloud] = []
        self.pcds_cropped_outliers: List[open3d.geometry.PointCloud] = []
        self.generation_goal = num_generation
        self.population_size = num_population
        self.num_of_shots = num_of_shots
        self.fitness_function_num = fitness_function
        self.map_params_fitnes = {}
        self.necessary_labels = set()

    def fitting_function(self, chromosome):  # custom fitness function
        silhouette_metric = 1
        davies_bouldin = 2
        calinski_harabasz = 3
        DBCV_metric = 4
        CDbw_metric = 5
        gt_metric = 6
        crowd_wisdom = 7
        filter = 8

        chrom_str = str(chromosome)
        if chrom_str in self.map_params_fitnes:
            return self.map_params_fitnes[chrom_str]
        else:
            scores = []
            for cloud in self.pcds_cropped_outliers:
                points = np.asarray(cloud.points)
                clustering = skc.DBSCAN(
                    eps=chromosome[0].value, min_samples=chromosome[1].value
                ).fit(points)
                # clustering = skc.KMeans(
                #     n_clusters=chromosome[0].value, max_iter=chromosome[1].value, n_init=chromosome[2].value
                # ).fit(np.asarray(points))
                num_of_clusters = len(np.unique(np.asarray(clustering.labels_)))

                if self.fitness_function_num == silhouette_metric:
                    self.target_fitness_type = "max"

                    if num_of_clusters < 20 or num_of_clusters > 250:
                        fitness = -1
                    else:
                        fitness = float(
                            metrics.silhouette_score(points, clustering.labels_)
                        )
                    scores.append(fitness)
                elif self.fitness_function_num == davies_bouldin:
                    self.target_fitness_type = "min"
                    if num_of_clusters < 20 or num_of_clusters > 250 or num_of_clusters < 5:  # need to check
                        fitness = 150
                    else:
                        davies_bouldin_score = float(
                            metrics.davies_bouldin_score(points, clustering.labels_)
                        )
                        fitness = davies_bouldin_score
                    scores.append(fitness)
                elif self.fitness_function_num == calinski_harabasz:
                    self.target_fitness_type = "max"
                    if num_of_clusters > 250 or num_of_clusters < 5:
                         fitness = 0
                    else:
                        calinski_harabasz_score = float(
                            metrics.calinski_harabasz_score(points, clustering.labels_)
                        )
                        fitness = calinski_harabasz_score
                    scores.append(fitness)
                elif self.fitness_function_num == DBCV_metric:
                    self.target_fitness_type = "max"
                    if num_of_clusters > 100:
                        fitness = -1
                    else:
                        DBCV_score = float(
                            DBCV(points, clustering.labels_)
                        )
                        fitness = DBCV_score
                    scores.append(fitness)
                elif self.fitness_function_num == CDbw_metric:
                    self.target_fitness_type = "max"
                    if num_of_clusters > 100 or num_of_clusters < 5:
                        fitness = -1
                    else:
                        CDbw_score = float(
                            CDbw(np.asarray(points), clustering.labels_)
                        )
                        fitness = CDbw_score
                    scores.append(fitness)
                elif self.fitness_function_num == gt_metric:
                    self.target_fitness_type = "max"
                    cur_map = {}
                    cloud.raw_labels = clustering.labels_
                    flatten_indices_of_interest = SemanticKitti_methods.extract_necessary_indices(cloud, self.necessary_labels)
                    fitness, _ = SemanticKitti_methods.evaluate_IoU(cloud, cur_map, flatten_indices_of_interest)
                    scores.append(fitness)
                elif self.fitness_function_num == crowd_wisdom:
                    self.target_fitness_type = "max"
                    if num_of_clusters > 250 or num_of_clusters < 15:
                        fitness = -1
                    else:
                        calinski = -1 / metrics.calinski_harabasz_score(points, clustering.labels_)  # originally maximisation function
                        true_devies_metric = metrics.davies_bouldin_score(points, clustering.labels_)
                        # if true_devies_metric > 1:  # originally minimisation function with minimum in 0
                        davies = 1 / true_devies_metric
                        # else:
                        #     davies = true_devies_metric
                        silhouette = metrics.silhouette_score(points, clustering.labels_)
                        fitness = calinski + 2 * davies + silhouette
                    scores.append(fitness)
                elif self.fitness_function_num == filter:

                    ## silhouette ->  davies -> calinski
                    # self.target_fitness_type = "max"
                    # fitness = -1
                    # if num_of_clusters > 250 or num_of_clusters < 15:
                    #     fitness = -1
                    # else:
                    #     silhouette = metrics.silhouette_score(points, clustering.labels_)
                    #     # calinski = metrics.calinski_harabasz_score(points, clustering.labels_)
                    #     if silhouette > 0.1:
                    #         davies = metrics.davies_bouldin_score(points, clustering.labels_)
                    #         if davies < 1:
                    #             fitness = metrics.calinski_harabasz_score(points, clustering.labels_)

                    ## silhouette -> calinski -> davies
                    self.target_fitness_type = "min"
                    fitness = 150
                    if num_of_clusters > 250 or num_of_clusters < 15:
                        fitness = 150
                    else:
                        silhouette = metrics.silhouette_score(points, clustering.labels_)
                        # calinski = metrics.calinski_harabasz_score(points, clustering.labels_)
                        # if silhouette > 0.1:
                        calinski = metrics.calinski_harabasz_score(points, clustering.labels_)
                        if calinski > 700:
                            fitness = metrics.davies_bouldin_score(points, clustering.labels_)
                    scores.append(fitness)
            fitness_value = np.mean(scores)
            self.map_params_fitnes[chrom_str] = fitness_value
            return fitness_value

    def initialize_population(self):
        a_eps = 0.1  # minimum valid distance
        b_eps = 1  # maximum valid distance

        a_min_samples = 1
        b_min_samples = self.population_size / 2

        h_eps = (b_eps - a_eps) / self.population_size
        h_min_samples = round((b_min_samples - a_min_samples) / self.population_size)
        if h_min_samples == 0:
            h_min_samples += 1
        self.population = self.make_population(
            [random.uniform(a_eps, b_eps), random.randint(1, 5)] for i in range(self.population_size)
        )

    # def my_chromosome_function(self):
    #     chromosome_data = [
    #         # Gene instructions set here
    #         random.randint(1, 100),
    #         random.randint(300, 400),
    #         random.randint(8, 15)
    #     ]
    #     return chromosome_data

    def ga_run(self, verbose):
        while self.active():
            self.evolve(1)
            if verbose:
                print("------------------------------")
                self.print_population()
                self.print_best_chromosome()
                self.print_worst_chromosome()
                print("------------------------------")
        return self.population[0].gene_value_list
