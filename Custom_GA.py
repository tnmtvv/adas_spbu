import random
import typing
from typing import List

# import fast_hdbscan
import numpy as np
import open3d
from EasyGA import GA
from EasyGA.crossover import Crossover
from cdbw import CDbw
from sklearn import metrics
from multiprocessing import Pool
from Utils.MyUtils import *


# from ClustersFeatures import *


class MyGA(GA):
    def __init__(
        self,
        num_of_shots: int,
        fitness_function_num: int,
        num_generation: int,
        num_population: int,
        chromosome_length: int,
        mutation_population_impl,
        gene_mutation_rate: float,
        params_types: List[str],
        params_names: List[str],
        clusterisation_method: typing.Callable,
        pcds_cropped_outliers: List[open3d.geometry.PointCloud],
        pcds_inliers: List[open3d.geometry.PointCloud],
        possible_string_params=None

    ):
        GA.__init__(self)
        self.pcds_inliers = list(pcds_inliers),
        self.pcds_cropped_outliers = list(pcds_cropped_outliers),
        self.generation_goal = num_generation
        self.population_size = num_population
        self.num_of_shots = num_of_shots
        self.fitness_function_num = fitness_function_num
        self.fitness_function_impl = self.fitness_function
        self.map_params_fitnes = {}
        self.necessary_labels = set()
        self.clusterisation_method = clusterisation_method
        self.chromosome_length = chromosome_length # number of parameters to optimize
        self.params_names = params_names
        self.params_types = params_types
        self.possible_str_params = possible_string_params
        self.initialize_population()
        self.crossover_population_impl = Crossover.Population.random
        self.crossover_individual_impl = Crossover.Individual.single_point
        self.mutation_population_impl = Mutation.Population.best_replace_worst
        self.mutation_individual_impl = Mutation.Individual.individual_genes
        self.gene_mutation_rate = gene_mutation_rate
        self.number_to_function = {1: (metrics.silhouette_score, 'max'), 2: (metrics.davies_bouldin_score, 'min'),
                              3: (metrics.calinski_harabasz_score, 'max'), 4: (CDbw, 'max'),
                              5: (self.crowd_wisdom, 'max'),
                              6: (self.filter, 'max')}

    @staticmethod
    def crowd_wisdom(points=None, raw_labels=None):
        calinski = -1 / metrics.calinski_harabasz_score(points, raw_labels)  # originally maximisation function
        true_devies_metric = metrics.davies_bouldin_score(points, raw_labels)
        davies = 1 / true_devies_metric
        silhouette = metrics.silhouette_score(points, raw_labels)
        fitness = calinski + 2 * davies + silhouette
        return fitness

    @staticmethod
    def filter(cloud, chromosome, points=None, raw_labels=None):
        fitness = 5000
        silhouette = metrics.silhouette_score(points, raw_labels)
        if silhouette > 0.1:
           calinski = metrics.calinski_harabasz_score(points, raw_labels)
           if calinski > 700:
               fitness = metrics.davies_bouldin_score(points, raw_labels)
        return fitness

    def fitness(self, cloud, chromosome):
        points = np.asarray(cloud.points)

        clusterer = self.clusterisation_method()

        params = list(map(lambda x: x.value, chromosome.gene_list))
        params_dict = dict(zip(self.params_names, params))

        clusterer.set_params(**params_dict)
        raw_labels = clusterer.fit_predict(cloud.points)
        num_of_clusters = len(set(raw_labels))

        target_fitness_type = self.number_to_function[self.fitness_function_num][1]
        if num_of_clusters < 20 or num_of_clusters > 500:
            if target_fitness_type == 'max':
                fitness = -1
            else:
                fitness = 5000
        else:
            fitness = float(
                self.number_to_function[self.fitness_function_num][0](points, raw_labels)
            )
        return fitness

    def fitness_function(self, chromosome):
        chrom_str = str(chromosome)
        if chrom_str in self.map_params_fitnes:
            return self.map_params_fitnes[chrom_str]
        else:
            clouds = list(self.pcds_cropped_outliers)[0]
            n = len(clouds)
            # with Pool() as pool:
                # scores = list(pool.starmap(self.fitness, list(zip(clouds, [chromosome]*n))))
            scores = list(map(self.fitness, clouds, [chromosome]*n))
            # pool.join()
            fitness_value = np.mean(scores)
            self.map_params_fitnes[chrom_str] = fitness_value
            return fitness_value


    def initialize_population(self):
        list_of_genes = []
        for i, param in enumerate(self.params_types):
            cur_list = []
            if param == 'int':
                cur_list = [random.randint(2, 100) for _ in range(self.population_size)]
            elif param == 'float':
                cur_list = [random.uniform(0.1, 1) for _ in range(self.population_size)]
            elif param == 'str':
                param_name = self.params_names[i]
                possible_values = self.possible_str_params[param_name]
                cur_list = [possible_values[random.randint(0, len(possible_values) - 1)] for _ in range(self.population_size)]
            list_of_genes.append(cur_list)
        if len(list_of_genes) > 1:
            cur_res = list_of_genes[0]
            for cur_param_list in list_of_genes[1:]:
                cur_res = list(zip(cur_res, cur_param_list))
                cur_res = list(map(lambda x: list(x), cur_res))

            cur_res = list(map(lambda x: to_list(x)[0], cur_res))
        else:
            cur_res = list_of_genes[0]
        self.population = self.make_population(cur_res)

    def my_chromosome_impl(self):
        chromosome_data = []
        for i, param in enumerate(self.params_types):
            if param == 'int':
                cur_param = random.randint(2, 100)
            elif param == 'float':
                cur_param = random.uniform(0.1, 1)
            else:
                param_name = self.params_names[i]
                possible_values = self.possible_str_params[param_name]
                cur_param = random.choice(possible_values)
            chromosome_data.append(cur_param)
        return chromosome_data


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
