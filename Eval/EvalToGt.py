import multiprocessing
import random
from typing import List, Set, Callable, Dict

# import fast_hdbscan
import numpy as np
import open3d
from EasyGA import GA
from EasyGA.crossover import Crossover
from cdbw import CDbw
from sklearn import metrics
from multiprocessing import Pool

from Eval import IoU
from Eval.LabeledPcd import LabeledPcd
from Utils.MyUtils import *

# from ClustersFeatures import *


class GAToCheck(GA):
    def __init__(
        self,
        num_of_shots: int,
        num_generation: int,
        num_population: int,
        chromosome_length: int,
        gene_mutation_rate: float,
        params_types: List[str],
        params_names: List[str],
        params_domains: Dict,
        clusterisation_method: Callable,
        pcds_cropped_outliers: List[LabeledPcd],
        necessary_labels: Set[int],
        possible_string_params=None

    ):
        GA.__init__(self)
        self.pcds_cropped_outliers = list(pcds_cropped_outliers),
        self.pcds_cropped_outliers = list(self.pcds_cropped_outliers)[0]
        self.generation_goal = num_generation
        self.population_size = num_population
        self.num_of_shots = num_of_shots
        self.fitness_function_impl = self.fitness_function
        self.map_params_fitness = {}
        self.necessary_labels = necessary_labels,
        self.necessary_labels= list(self.necessary_labels)[0]
        self.clusterisation_method = clusterisation_method
        self.chromosome_length = chromosome_length # number of parameters to optimize
        self.params_names = params_names
        self.params_types = params_types
        self.possible_str_params = possible_string_params
        self.chromosome_impl = self.my_chromosome_impl
        self.initialize_population()
        self.crossover_population_impl = Crossover.Population.random
        self.crossover_individual_impl = Crossover.Individual.single_point
        self.mutation_population_impl = Mutation.Population.best_replace_worst
        self.mutation_individual_impl = Mutation.Individual.individual_genes
        self.gene_mutation_rate = gene_mutation_rate

    def fitness_function(self, chromosome):  # custom fitness function
        chrom_str = str(chromosome)
        clusterer = self.clusterisation_method()
        if chrom_str in self.map_params_fitness:
            return self.map_params_fitness[chrom_str]
        else:
            scores = []
            for cloud in self.pcds_cropped_outliers:
                points = np.asarray(cloud.pcd.points)
                params = list(map(lambda x: x.value, chromosome.gene_list))
                params_dict = dict(zip(self.params_names, params))

                clusterer.set_params(**params_dict)
                raw_labels = clusterer.fit_predict(points)
                num_of_clusters = len(set(raw_labels))
                self.target_fitness_type = "max"
                cur_map = {}
                cloud.raw_labels = raw_labels
                if not self.necessary_labels:
                    flatten_indices_of_interest = IoU.extract_necessary_indices(cloud, set(cloud.gt_labels))
                else:
                    flatten_indices_of_interest = IoU.extract_necessary_indices(cloud, self.necessary_labels)
                fitness, _ = IoU.evaluate_IoU(cloud, cur_map, flatten_indices_of_interest)
                scores.append(fitness)
            fitness_value = np.mean(scores)
            self.map_params_fitness[chrom_str] = fitness_value
            return fitness_value

    def user_chromosome_function(self):
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
                cur_param = random.randint(1, 100)
            elif param == 'float':
                cur_param = random.uniform(0.1, 1)
            else:
                param_name = self.params_names[i]
                possible_values = self.possible_str_params[param_name]
                cur_param = random.choice(possible_values)
            chromosome_data.append(cur_param)
        return chromosome_data

    def ga_run(self, verbose):
        self.target_fitness_type = 'max'
        while self.active():
            self.evolve(1)
            if verbose:
                print("------------------------------")
                self.print_population()
                self.print_best_chromosome()
                self.print_worst_chromosome()
                print("------------------------------")
        return self.population[0].gene_value_list
