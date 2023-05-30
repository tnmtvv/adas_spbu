import argparse
import json
from typing import Dict

import numpy as np

import Custom_GA
from Utils import loader, pcdUtils
import numpy.typing as npt
from EasyGA.mutation import Mutation
from sklearn import cluster as skc

from Utils.DatasetFunctions.SemanticKitti_methods import get_AoI_indices

import fast_hdbscan


def main(
    path_to_lidar: str,
    path_to_images: str,
    start_indx: int,
    num_shots_to_optimise: int,
    generation_goal: int,
    population_size: int,
    fitness_function: int,
    dataset: int,
    clusterisation_method: str,
    verbose=True,
):
    algos_params_types, string_params, map_params_domains = loader.read_algos_params()
    algos_functions = {
        "dbscan": skc.DBSCAN,
        "kmeans": skc.KMeans,
        "optics": skc.OPTICS,
        "AgglomerativeClustering": skc.AgglomerativeClustering,
        "Bisecting_K-Means": skc.BisectingKMeans,
        "SpectralClustering": skc.SpectralClustering,
        "hdbscan": fast_hdbscan.HDBSCAN,
    }

    algo = algos_functions[clusterisation_method]
    params_types, params_names = pcdUtils.parse_parameters_info(
        algos_params_types[clusterisation_method]
    )

    possible_str_params = None
    if clusterisation_method in string_params:
        possible_str_params = string_params[clusterisation_method]
    verbose = True

    list_main, list_sub = loader.create_data_lists(
        path_to_lidar,
        path_to_images,
        start_indx,
        start_indx + num_shots_to_optimise,
        separator=r"\.|_",
    )

    if num_shots_to_optimise == -1:
        num_shots_to_optimise = len(list_main)
    map_label_color: Dict[int, npt.NDArray] = {}
    if dataset == 1:
        pcds, lidars = loader.build_point_clouds_and_lidars(
            list_main, num_shots_to_optimise
        )
    else:
        pcds = loader.extract_sem_kitti_pcds(
            list_main, list_sub, get_AoF=get_AoI_indices
        )
    (
        pcds_cropped_outliers,
        pcds_inliers,
        inliers_indx,
    ) = pcdUtils.plane_segmentation(pcds, 5)
    # getting pcds without road, road, above road area and road points indices

    clouds_points = []
    cur_params_domains = {}
    if clusterisation_method in map_params_domains:
        cur_params_domains = map_params_domains[clusterisation_method]
    for cloud in pcds_cropped_outliers:
        clouds_points.append(np.asarray(cloud.points))
    ga = Custom_GA.MyGA(
        num_shots_to_optimise,
        fitness_function,
        generation_goal,
        population_size,
        len(params_names),
        Mutation.Population.random_avoid_best,
        0.3,
        params_types,
        params_names,
        cur_params_domains,
        algo,
        pcds_cropped_outliers,
        pcds_inliers,
        clouds_points,
        possible_string_params=possible_str_params,
    )

    best_params = ga.ga_run(verbose)
    best_params_dict = dict(zip(params_names, best_params))

    with open("results.txt", "a") as file:
        file.write(json.dumps(best_params_dict))
        file.write("\n")
    print(best_params)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Finds parameters for clusterization algorithms"
    )
    parser.add_argument(
        "main_path", type=str, help="Directory where lidar files are stored"
    )
    parser.add_argument(
        "extra_path", type=str, help="Directory where images are stored"
    )
    parser.add_argument("start_indx", type=int, help="Start index")
    parser.add_argument("num_of_shots", type=int, help="Number of shots to optimise")
    parser.add_argument("generation_goal", type=int, help="Number of generations in GA")
    parser.add_argument(
        "population_size", type=int, help="Number of samples in population"
    )
    parser.add_argument(
        "fitness_function",
        type=int,
        choices=[1, 2, 3, 4, 5, 6],
        help="Which fitness functon to use for optimisation: "
        "1 -- silhouette score, "
        "2 -- davies-bouldin score,"
        "3 -- calinski-harabasz,"
        "4 -- CDbw,"
        "5 -- CrowdWisdom"
        "6 -- filter",
    )
    parser.add_argument(
        "dataset",
        type=int,
        choices=[1, 2],
        help="Choose dataset: 1 -- AUDI, 2 -- SemanticKitti",
    )
    parser.add_argument(
        "algo",
        type=str,
        help="Your algorithm: dbscan, kmeans, optics, AgglomerativeClustering, "
        "Bisecting_K-Means, SpectralClustering, hdbscan",
    )
    parser.add_argument("--no-verbose", dest="verbose", action="store_false")

    args = parser.parse_args()
    main(
        args.main_path,
        args.extra_path,
        args.start_indx,
        args.num_of_shots,
        args.generation_goal,
        args.population_size,
        args.fitness_function,
        args.dataset,
        args.algo,
        args.verbose,
    )
