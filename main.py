import argparse
import copy
import random
from typing import Set

import loader
import MyUtils
import numpy as np
import open3d as o3d
import SemanticKitti_methods
from AUDI_methods import AUDIMethods
from Custom_GA import MyGA
from EasyGA.crossover import Crossover
from EasyGA.mutation import Mutation
from LabeledPcd import LabeledPcd
from matplotlib import pyplot as plt
from sklearn import cluster as skc


def ransac_segmentation(list_pcds, gt_labeled_pcds, distance_threshold=0.4):
    list_outliers = []
    list_inliers = []
    list_indices = []

    for i, pcd in enumerate(list_pcds):
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=distance_threshold, ransac_n=3, num_iterations=1000
        )

        inlier_cloud = pcd.select_by_index(inliers)
        outlier_cloud = pcd.select_by_index(inliers, invert=True)
        gt_labeled_pcds[i].pcd = gt_labeled_pcds[i].pcd.select_by_index(
            inliers, invert=True
        )

        list_outliers.append(outlier_cloud)
        list_inliers.append(inlier_cloud)
        list_indices.append(inliers)

    return list_outliers, list_inliers, gt_labeled_pcds, list_indices


def separate_all_clusters(pcd, labels, hm_lables_colors, indices):
    clusters_pcds = []
    clusters_dists = []
    pcd.paint_uniform_color([0, 0, 0])

    labels_filtered = list(filter(lambda x: x >= 0, labels))  # filter from x < 0

    for i, label_1 in enumerate(labels_filtered):
        for j, label_2 in enumerate(labels_filtered[i + 1 :]):
            new_pcd = copy.deepcopy(pcd)
            cur_color_1 = hm_lables_colors[label_1]
            cur_color_2 = hm_lables_colors[label_2]

            cluster_1 = pcd.select_by_index(indices[i])
            cluster_2 = pcd.select_by_index(indices[i + j + 1])

            colors = np.zeros(np.shape(pcd.points))
            colors[indices[i]] = cur_color_1
            colors[indices[i + j + 1]] = cur_color_2

            new_pcd.colors = o3d.utility.Vector3dVector(colors)

            dists = cluster_1.compute_point_cloud_distance(cluster_2)
            dists = np.asarray(dists)
            min_dist = np.min(dists)

            clusters_dists.append(min_dist)
            clusters_pcds.append(new_pcd)
    return clusters_dists, clusters_pcds


def main(
    path_to_lidar: str,
    path_to_images: str,
    start_indx: int,
    num_shots_to_optimise: int,
    default: bool,
    generation_goal: int,
    population_size: int,
    fitness_function: int,
    mode: int,
    dataset: int,
    verbose=True,
):
    verbose = True
    best_params_dbscan = (0.6, 7)
    best_params_ransac = 0.125
    images = []

    list_main, list_sub = loader.create_data_lists(
        path_to_lidar,
        path_to_images,
        start_indx,
        start_indx + num_shots_to_optimise,
        separator=".",
    )

    if mode == 3:
        images = loader.extract_images_audi(list_sub)
    if num_shots_to_optimise == -1:
        num_shots_to_optimise = len(list_main)
    if dataset == 1:
        pcds, lidars = loader.build_point_clouds_and_lidars(
            list_sub, num_shots_to_optimise
        )
    else:
        map_color_label = {}
        gt_labeled_pcds, extracted_pcds = loader.extract_sem_kitti_pcds(
            list_main, list_sub, map_color_label
        )
    ga = MyGA(num_shots_to_optimise, fitness_function, generation_goal, population_size)
    ga.chromosome_length = 2  # number of parameters to optimize

    (
        ga.pcds_cropped_outliers,
        ga.pcds_inliers,
        gt_outliers,
        inliers_indx,
    ) = ransac_segmentation(extracted_pcds, gt_labeled_pcds, best_params_ransac)
    # getting pcds without road, road, above road area and road points indices

    clusters_indices = []

    ga.chromosome_impl = lambda: [random.uniform(0.1, 3), random.randrange(5, 15)]

    ga.crossover_population_impl = (
        Crossover.Population.random
    )  # setting ga configuration
    ga.mutation_population_impl = Mutation.Population.random_avoid_best
    ga.fitness_function_impl = ga.fitting_function
    ga.gene_mutation_rate = 0.2

    if not default:  # choose parameters with ga or use default ones
        best_params_dbscan = ga.ga_run(verbose)
    raw_clustered_pcds = []
    IoU = []

    for i, pcd in enumerate(
        zip(ga.pcds_cropped_outliers, ga.pcds_inliers)
    ):  # clustering with chosen parameters
        cur_pcd_clusters = []
        cur_label_color = {}

        pcd_outlier, pcd_inlier = pcd
        clustering = skc.DBSCAN(
            eps=best_params_dbscan[0], min_samples=best_params_dbscan[1]
        ).fit(np.asarray(pcd_outlier.points))

        labels = clustering.labels_

        cur_raw_labeled_pcd = LabeledPcd(pcd_outlier, labels)

        map_raw_true = {}
        evaluated_IoU = SemanticKitti_methods.evaluate_IoU(
            gt_outliers[i], cur_raw_labeled_pcd, map_raw_true
        )
        IoU.append(evaluated_IoU)

        colors_cloud = np.zeros(np.shape(pcd_outlier.points))

        for label in set(labels):
            true_label = map_raw_true[label]
            cur_color = map_color_label[true_label]
            cur_indices = np.where(labels == label)[0].tolist()
            colors_cloud[cur_indices] = cur_color
            cur_pcd_clusters.append(cur_indices)
        clusters_indices.append(cur_pcd_clusters)

        cur_raw_labeled_pcd.pcd.colors = o3d.utility.Vector3dVector(colors_cloud)
        merged_pcd = pcd_outlier + pcd_inlier
        unique_labels = set(labels)

        if mode == 1:  # only point clouds
            o3d.visualization.draw_geometries([gt_labeled_pcds[i].pcd])
            o3d.visualization.draw_geometries([cur_raw_labeled_pcd.pcd])
            print(evaluated_IoU)
        if mode == 2:  # point clouds with distances
            o3d.visualization.draw_geometries([merged_pcd])
            if_dists = int(input("show distances"))

            if if_dists:
                dists, pair_clusters = separate_all_clusters(
                    merged_pcd, unique_labels, cur_label_color, cur_pcd_clusters
                )
                for j, pair in enumerate(pair_clusters):
                    print(dists[j])
                    o3d.visualization.draw_geometries([pair])
        elif mode == 3:  # mapped images
            colors = np.zeros(np.shape(merged_pcd.colors))
            mask = np.ones(len(merged_pcd.colors), dtype=bool)
            mask[inliers_indx[i]] = 0
            colors[mask] = colors_cloud

            mapped_images = []
            image = images[i]

            cur_image = AUDIMethods.map_lidar_points_onto_image(
                image, lidars[i], colors
            )
            mapped_images.append(cur_image)

            plt.fig = plt.figure(figsize=(20, 20))
            plt.imshow(cur_image)
            plt.axis("off")
            plt.close()
        elif mode == 4:  # with editing
            vis = o3d.visualization.VisualizerWithEditing()
            vis.create_window()
            vis.add_geometry(merged_pcd)
            vis.run()  # user picks points
            vis.destroy_window()
            picked_points = vis.get_picked_points()

            mismatched_points = merged_pcd.select_by_index(picked_points)

            dists = pcd_inlier.compute_point_cloud_distance(mismatched_points)
            dists = np.asarray(dists)
            min_dist = np.min(dists)
            print(min_dist)
    print(np.mean(np.asarray(IoU)))


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
        choices=[1, 2, 3],
        help="Which fitness functon to use for optimisation: "
        "1 -- silhouette score, "
        "2 -- davies-bouldin score,"
        "3 -- calinski-harabasz",
    )
    parser.add_argument(
        "mode",
        type=int,
        choices=[1, 2, 3, 4],
        help="Choose the mode: 1 -- point clouds,"
        " 2 -- point clouds with distances, "
        "3 -- mapped images, 4 -- points selection mode",
    )
    parser.add_argument(
        "dataset",
        type=int,
        choices=[1, 2],
        help="Choose dataset: 1 -- AUDI," " 2 -- SemanticKitti",
    )
    parser.add_argument("--no-verbose", dest="verbose", action="store_false")

    args = parser.parse_args()

    main(
        args.lidar_path,
        args.images_path,
        args.start_indx,
        args.num_of_shots,
        args.default,
        args.generation_goal,
        args.population_size,
        args.fitness_function,
        args.mode,
        args.dataset,
        args.verbose,
    )
