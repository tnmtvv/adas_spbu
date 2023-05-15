import argparse
from typing import Dict

import numpy as np

import Custom_GA
from Eval import IoU
from Utils import loader, MyUtils
import numpy.typing as npt
import open3d as o3d
from Utils.DatasetFunctions import SemanticKitti_methods
from EasyGA.mutation import Mutation
from sklearn import cluster as skc

from Utils.DatasetFunctions.SemanticKitti_methods import get_AoI_indices


# import fast_hdbscan

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
    algos_params_types, string_params = loader.read_algos_params()
    algos_functions = {'dbscan': skc.DBSCAN, 'kmeans': skc.KMeans, 'optics': skc.OPTICS,
                       'AgglomerativeClustering': skc.AgglomerativeClustering, 'Bisecting_K-Means': skc.BisectingKMeans,
                       'SpectralClustering': skc.SpectralClustering}

    algo = algos_functions[clusterisation_method]
    params_types, params_names = MyUtils.parse_parameters_info(algos_params_types[clusterisation_method])

    possible_str_params = None
    if clusterisation_method in string_params:
        possible_str_params = string_params[clusterisation_method]

    verbose = True

    list_main, list_sub = loader.create_data_lists(
        path_to_lidar,
        path_to_images,
        start_indx,
        start_indx + num_shots_to_optimise,
        separator=r'\.|_',
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
            list_main, list_sub, get_AoI_indices
        )
    (
        pcds_cropped_outliers,
        pcds_inliers,
        inliers_indx,
    ) = MyUtils.plane_segmentation(pcds, 5)
    # getting pcds without road, road, above road area and road points indices

    clouds_points = []
    for cloud in pcds_cropped_outliers:
        clouds_points.append(np.asarray(cloud.points))
    ga = Custom_GA.MyGA(num_shots_to_optimise, fitness_function, generation_goal, population_size, len(params_names),
                        Mutation.Population.random_avoid_best, 0.3, params_types, params_names,
                        algo, pcds_cropped_outliers, pcds_inliers, clouds_points, possible_string_params=possible_str_params)

    best_params = ga.ga_run(verbose)
    best_params_dict = dict(zip(params_names, best_params))

    with open("results.txt", "a") as file:
        file.write("{} {} {}\n".format(tuple(best_params_dict), num_shots_to_optimise, fitness_function))
    file.close()

    print(best_params)
    # for pcd in pcds_cropped_outliers:  # clustering with chosen parameters
    #
    #     pcd_outlier = pcd
    #
    #     clusterer = algo()
    #     clusterer.set_params(**best_params_dict)
    #     raw_labels = clusterer.fit_predict(pcd_outlier.points)
    #
    #     pcd_outlier = MyUtils.paint_cloud(pcd_outlier, raw_labels)
    #     o3d.visualization.draw_geometries([pcd_outlier])


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
        choices=[1, 2, 3, 4, 5, 6, 7, 8],
        help="Which fitness functon to use for optimisation: "
             "1 -- silhouette score, "
             "2 -- davies-bouldin score,"
             "3 -- calinski-harabasz",

    )
    parser.add_argument(
        "dataset",
        type=int,
        choices=[1, 2],
        help="Choose dataset: 1 -- AUDI," " 2 -- SemanticKitti",
    )
    parser.add_argument("algo", type=str, help="Your algorithm")
    parser.add_argument("--no-verbose", dest="verbose", action="store_false")

    args = parser.parse_args()
    needed_functions = [1, 2, 3, 4, 5]
    num_clouds = [10, 25, 100]
    algos = ['dbscan', 'kmeans', 'optics']
    for algo in algos:
        for num_cloud in num_clouds:
            for func in needed_functions:
                main(
                    args.main_path,
                    args.extra_path,
                    args.start_indx,
                    num_cloud,
                    args.generation_goal,
                    args.population_size,
                    func,
                    args.dataset,
                    algo,
                    args.verbose,
                )