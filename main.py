import argparse
import copy
import random
import statistics
from typing import Dict, List

import MyUtils
import loader
import numpy as np
import numpy.typing as npt
import open3d as o3d
import SemanticKitti_methods
from AUDI_methods import AUDIMethods
from Custom_GA import MyGA
from sklearn import metrics
from EasyGA.crossover import Crossover
from EasyGA.mutation import Mutation
from LabeledPcd import LabeledPcd
from matplotlib import pyplot as plt
from sklearn import cluster as skc

from Plane import Plane


def two_stage_plane_segmentation(cur_pcd, down_sample_coeff=1):
    cur_pcd = cur_pcd.uniform_down_sample(down_sample_coeff)
    points = np.array(cur_pcd.points)

    # extract plane on same level with the lowest point
    min_z_point = MyUtils.find_min_z(points)
    plane_point_indices = np.where(np.abs(points[:, -1] - min_z_point) < 0.15)
    plane_points = points[plane_point_indices]

    # pcd = cur_pcd.select_by_index(plane_point_indices[0])
    # o3d.visualization.draw_geometries([cur_pcd])
    # o3d.visualization.draw_geometries([pcd])

    # build plane equation of the extracted points
    plane_equation = Plane.get_equation(plane_points)

    normal_vector = plane_equation[:-1]
    c = np.mean(plane_points, axis=0)
    d = np.dot(normal_vector, c)

    # get all the points that satisfy the equation
    second_plane_point_indices = np.where(np.abs(np.asarray(points).dot(normal_vector) - d) <= 0.1)[0]
    inlier_cloud = cur_pcd.select_by_index(second_plane_point_indices)
    cur_pcd = cur_pcd.select_by_index(second_plane_point_indices, invert=True)

    left_bound, right_bound = MyUtils.find_left_right(list(inlier_cloud.points))

    cur_outlier_points = list(np.asarray(cur_pcd.points))
    cropped_outlier_points = list(filter(lambda x: left_bound <= x[1] <= right_bound, cur_outlier_points))
    cropped_outlier_indices = range(0, len(cur_outlier_points))
    set_outlier_points = set(tuple(x) for x in cropped_outlier_points)

    cropped_outlier_indices = list(
        filter(lambda i: tuple(cur_outlier_points[i]) in set_outlier_points, cropped_outlier_indices))

    cur_pcd = cur_pcd.select_by_index(cropped_outlier_indices)
    # o3d.visualization.draw_geometries([inlier_cloud])
    return cur_pcd, inlier_cloud, second_plane_point_indices


def plane_segmentation(gt_pcds, down_smple_coeff=1):
    all_indices = []
    inlier_clouds = []
    for i, _ in enumerate(gt_pcds):
        gt_pcds[i], inlier_cloud, indices = two_stage_plane_segmentation(gt_pcds[i], down_smple_coeff)
        all_indices.append(indices)
        inlier_clouds.append(inlier_cloud)

    return gt_pcds, inlier_clouds, all_indices


def main(
        path_to_lidar: str,
        path_to_images: str,
        start_indx: int,
        num_shots_to_optimise: int,
        default: bool,
        generation_goal: int,
        population_size: int,
        fitness_function: int,
        dataset: int,
        verbose=True,
):

    verbose = True
    best_params_dbscan = (2.5335932584187955, 5)
    best_params_kmeans = (20, 300, 10)
    best_params_ransac = 0.14
    images = []

    list_main, list_sub = loader.create_data_lists(
        path_to_lidar,
        path_to_images,
        start_indx,
        start_indx + num_shots_to_optimise,
        separator=".",
    )

    ga = MyGA(num_shots_to_optimise, fitness_function, generation_goal, population_size)
    ga.chromosome_length = 2  # number of parameters to optimize
    necessary_labels = []

    if num_shots_to_optimise == -1:
        num_shots_to_optimise = len(list_main)
    if dataset == 1:
        map_label_color: Dict[int, npt.NDArray] = {}
        pcds, lidars = loader.build_point_clouds_and_lidars(
            list_sub, num_shots_to_optimise
        )
        (
            ga.pcds_labeled_outliers,
            ga.pcds_inliers,
            ga.inliers_indx,
        ) = plane_segmentation(pcds, 5)
    else:

        map_label_color: Dict[int, npt.NDArray] = {}
        gt_pcds = loader.extract_sem_kitti_pcds(
            list_main, list_sub, map_label_color, SemanticKitti_methods.get_AoI_indices
        )
        (
            ga.pcds_cropped_outliers,
            ga.pcds_inliers,
            ga.inliers_indx,
        ) = plane_segmentation(gt_pcds, 5)

        # getting pcds without road, road, above road area and road points indices
    ga.chromosome_impl = ga.my_chromosome_function
    ga.initialize_population()
    ga.crossover_population_impl = (
        Crossover.Population.random
    )  # setting ga configuration
    ga.mutation_population_impl = Mutation.Population.random_avoid_best
    ga.fitness_function_impl = ga.fitting_function
    ga.gene_mutation_rate = 0.3
    ga.necessary_labels = necessary_labels

    if default:  # choose parameters with ga or use default ones
        # best_params_dbscan = ga.ga_run(verbose)
        best_params_kmeans = ga.ga_run(verbose)

    with open("results.txt", "a") as file:
        file.write("{} {} {}\n".format(tuple(best_params_kmeans), num_shots_to_optimise, fitness_function))
    file.close()
    # for i, pcd in enumerate(
    #     zip(ga.pcds_cropped_outliers, ga.pcds_inliers)
    # ):  # clustering with chosen parameters
    #
    #     pcd_outlier, pcd_inlier = pcd
    #     merged_pcd = pcd_outlier + pcd_inlier
    #
    #     clustering = skc.DBSCAN(
    #         eps=best_params_dbscan[0], min_samples=best_params_dbscan[1]
    #     ).fit(np.asarray(pcd_outlier.points))
    #
    #     # clustering = skc.KMeans(
    #     #     n_clusters=best_params_kmeans[0], max_iter=best_params_kmeans[1], n_init=best_params_kmeans[1]
    #     # ).fit(np.asarray(pcd_outlier.pcd.points))
    #
    #     # pcd_outlier.raw_labels = clustering.labels_
    #     # pcd_outlier.unique_raw_labels = set(pcd_outlier.raw_labels)
    #
    #     # pcd_outlier.gt_colors = np.asarray(pcd_outlier.colors)
    #
    #     colors_cloud_raw = np.zeros(np.shape(pcd_outlier.points))
    #
    #     one_more_set = set()
    #     for label in set(clustering.labels_):
    #         cur_color = MyUtils.align_color(label, one_more_set)
    #         cur_indices = np.where(clustering.labels_ == label)[0].tolist()
    #         # label_indices_of_interest = list(set(cur_indices).intersection(set(flatten_indices_of_interest)))
    #         colors_cloud_raw[cur_indices] = cur_color
    #
    #     # pcd_outlier.raw_colors = colors_cloud_raw
    #
    #     # visualizing with raw
    #     # pcd_outlier.colors = o3d.utility.Vector3dVector(colors_cloud_raw)
    #     # o3d.visualization.draw_geometries([pcd_outlier])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Finds parameters for clusterization algorithms"
    )
    parser.add_argument(
        "lidar_path", type=str, help="Directory where lidar files are stored"
    )
    parser.add_argument(
        "images_path", type=str, help="Directory where images are stored"
    )
    parser.add_argument("start_indx", type=int, help="Start index")
    parser.add_argument("num_of_shots", type=int, help="Number of shots to optimise")
    parser.add_argument("--no-default", dest="default", action="store_false")
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
    parser.add_argument("--no-verbose", dest="verbose", action="store_false")

    args = parser.parse_args()
    # needed_functions = [1, 2, 3, 7]
    needed_functions = [7]
    num_clouds = [10]
    for metric in needed_functions:
        for nc in num_clouds:
            main(
                args.lidar_path,
                args.images_path,
                args.start_indx,
                nc,
                args.default,
                args.generation_goal,
                args.population_size,
                metric,
                args.dataset,
                args.verbose,
            )