import argparse
import os.path
import random
import cv2
import glob

import numpy as np
import open3d as o3d

from AUDI_methods import AUDIMethods
from EasyGA.mutation import Mutation
from EasyGA.crossover import Crossover
from Custom_GA import My_GA
from Plane import Plane
from sklearn import cluster as skc
from copy import copy
from matplotlib import pyplot as plt


def run_algo(pcd):
    segment_models = {}
    segments = {}
    max_plane_idx = 5
    rest = pcd

    colors = plt.get_cmap("tab20")(0)
    segment_models[0], inliers = rest.segment_plane(
        distance_threshold=0.15, ransac_n=3, num_iterations=1000)
    segments[0] = rest.select_by_index(inliers)
    segments[0].paint_uniform_color(list(colors[:3]))
    rest = rest.select_by_index(inliers, invert=True)

    for i in range(1, max_plane_idx):
        colors = plt.get_cmap("tab20")(i)
        segment_models[i], inliers = rest.segment_plane(
            distance_threshold=0.12, ransac_n=3, num_iterations=1000)
        segments[i] = rest.select_by_index(inliers)
        segments[i].paint_uniform_color(list(colors[:3]))
        rest = rest.select_by_index(inliers, invert=True)
        print("pass", i, "/", max_plane_idx, "done.")
    o3d.visualization.draw_geometries([segments[i] for i in range(max_plane_idx)]+[rest])


def build_point_clouds_and_lidars(lidar_list, num_shots):

    list_pcds = []
    list_lidars = []

    for file in lidar_list:
        data = np.load(file)

        lidar_front_center = {'points': data['pcloud_points'], 'reflectance': data['pcloud_attr.reflectance'],
                              'timestamp': data['pcloud_attr.timestamp.npy'], 'row': data['pcloud_attr.row.npy'],
                              'col': data['pcloud_attr.col.npy'], 'distance': data['pcloud_attr.distance.npy'],
                              'depth': data['pcloud_attr.depth.npy'], 'lidar_ids': data['pcloud_attr.lidar_id.npy']}
        pcd_front_center = AUDIMethods.create_open3d_pc(lidar_front_center)
        list_pcds.append(pcd_front_center)
        list_lidars.append(lidar_front_center)

    return list_pcds[:num_shots], list_lidars[:num_shots]


def extract_images(images_path_list):
    images_list = []

    for file_name in images_path_list:
        images_list.append(cv2.imread(file_name))
    return images_list


def create_lidar_image_lists(path_to_lidar: str, path_to_images: str, indx_from=0, indx_to=0):
    lidar_files = os.listdir(path_to_lidar)
    images_files = glob.glob(path_to_images + '/*.png')

    lidar_files = sorted(lidar_files, key=lambda x: int(x[-13:-4]))
    images_files = sorted(images_files, key=lambda x: int(x[-13:-4]))

    list_lidar = list(map(lambda x: os.path.join(path_to_lidar, x), lidar_files))
    list_images = list(map(lambda x: os.path.join(path_to_images, x), images_files))

    if indx_to == indx_from:
        indx_from = 0
        indx_to = len(list_lidar)

    return list_lidar[indx_from: indx_to], list_images[indx_from: indx_to]


def find_left_right(inlier_points):
    sorted_points = sorted(inlier_points, key=lambda x: x[1])
    return sorted_points[0][1], sorted_points[-1][1]


def ransac_segmentation(list_pcds, distance_threshold=0.4):
    list_outliers = []
    list_inliers = []
    list_cropped_pcds = []
    list_indices = []

    for pcd in list_pcds:
        plane_model, inliers = pcd.segment_plane(distance_threshold=distance_threshold, ransac_n=3, num_iterations=1000)

        inlier_cloud = pcd.select_by_index(inliers)
        outlier_cloud = pcd.select_by_index(inliers, invert=True)

        list_outliers.append(outlier_cloud)
        list_inliers.append(inlier_cloud)
        list_indices.append(inliers)

        left_bound, right_bound = find_left_right(list(inlier_cloud.points))

        cur_outlier_points = np.array(outlier_cloud.points).tolist()
        outlier_points = list(filter(lambda x: (left_bound <= x[1] <= right_bound).all(axis=0), cur_outlier_points))

        cropped_outlier = o3d.geometry.PointCloud()
        cropped_outlier.points = o3d.utility.Vector3dVector(outlier_points)
        list_cropped_pcds.append(cropped_outlier)

    return list_outliers, list_inliers, list_cropped_pcds, list_indices


def separate_all_clusters(pcd, labels, hm_lables_colors, indices):
    clusters_pcds = []
    clusters_dists = []
    pcd.paint_uniform_color([0, 0, 0])

    labels_filtered = list(filter(lambda x: x >= 0, labels))  # filter from x < 0

    for i, label_1 in enumerate(labels_filtered):
        for j, label_2 in enumerate(labels_filtered[i + 1:]):
            new_pcd = copy(pcd)
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


def main(path_to_lidar: str, path_to_images: str, start_indx: int,
         num_shots_to_optimise: int, default: int, generation_goal:
        int, population_size: int, fitness_function: int, mode: int, verbose=False):

    list_lidar, list_images = create_lidar_image_lists(path_to_lidar, path_to_images, start_indx,
                                                       start_indx + num_shots_to_optimise)

    if num_shots_to_optimise == 0:
        num_shots_to_optimise = len(list_lidar)

    pcds, lidars = build_point_clouds_and_lidars(list_lidar, num_shots_to_optimise)

    ga = My_GA(num_shots_to_optimise, fitness_function, generation_goal, population_size)
    ga.chromosome_length = 2
    best_params = (2, 10)

    outliers, ga.pcds_inliers, cropped_outliers, inliers_indx = ransac_segmentation(pcds, 0.125)

    clusters_indices = []
    dicts_label_color = []

    for i, inlier in enumerate(ga.pcds_inliers):
        plane_equation = Plane.get_equation(inlier.points)
        points_projected = []

        for point in cropped_outliers[i].points:
            projection = point - (np.dot(plane_equation[:-1], point) + plane_equation[-1]) * plane_equation[:-1]
            points_projected.append(projection)
        pcd_projected = o3d.geometry.PointCloud()
        pcd_projected.points = o3d.utility.Vector3dVector(points_projected)

        ga.pcds_projected_outliers.append(pcd_projected)

    ga.chromosome_impl = lambda: [
        random.uniform(0.1, 3),
        random.randrange(5, 15)
    ]

    ga.crossover_population_impl = Crossover.Population.random
    ga.mutation_population_impl = Mutation.Population.random_avoid_best
    ga.fitness_function_impl = ga.fitting_function
    ga.gene_mutation_rate = 0.1

    if default == 0:
        while ga.active():
            ga.evolve(1)
            if verbose:
                print('------------------------------')
                ga.print_population()
                ga.print_best_chromosome()
                ga.print_worst_chromosome()
                print('------------------------------')

        best_params = ga.population[0].gene_value_list

    for i, pcd in enumerate(zip(outliers, ga.pcds_inliers)):
        cur_pcd_clusters = []
        cur_label_color = {}

        pcd_outlier, pcd_inlier = pcd
        clustering = skc.DBSCAN(eps=best_params[0], min_samples=best_params[1]).fit(np.asarray(pcd_outlier.points))
        labels = clustering.labels_
        unique_labels = np.unique(labels)
        set_colors = set()

        colors_outlier = np.zeros(np.shape(pcd_outlier.points))

        for label in unique_labels:
            if label < 0:
                colors_outlier[np.where(labels == label)] = [0, 0, 0]
            else:
                cur_indices = np.where(labels == label)[0].tolist()
                cur_color = plt.get_cmap("prism")(random.randint(1, 1000))[:3]
                while cur_color in set_colors:
                    cur_color = plt.get_cmap("prism")(random.randint(1, 1000))[:3]
                set_colors.add(cur_color)
                cur_label_color[label] = cur_color
                colors_outlier[np.where(labels == label)] = cur_color
                set_colors.add(cur_color)
                cur_pcd_clusters.append(cur_indices)

        clusters_indices.append(cur_pcd_clusters)
        dicts_label_color.append(cur_label_color)

        pcd_outlier.colors = o3d.utility.Vector3dVector(colors_outlier)
        pcd_inlier.paint_uniform_color([0, 0, 0])

        merged_pcd = pcd_inlier + pcd_outlier

        if mode == 1:  # only point clouds
            o3d.visualization.draw_geometries([merged_pcd])
        if mode == 2:  # point clouds with distances
            o3d.visualization.draw_geometries([merged_pcd])
            if_dists = int(input("show distances"))

            if if_dists:
                dists, pair_clusters = separate_all_clusters(merged_pcd, unique_labels, cur_label_color,
                                                             cur_pcd_clusters)
                for j, pair in enumerate(pair_clusters):
                    print(dists[j])
                    o3d.visualization.draw_geometries([pair])
        elif mode == 3:  # mapped images
            colors = np.zeros(np.shape(pcds[i].colors))
            mask = np.ones(len(pcds[i].colors), dtype=bool)
            mask[inliers_indx[i]] = 0
            colors[inliers_indx[i]] = [0, 0, 0]
            colors[mask] = colors_outlier

            mapped_images = []
            images = extract_images(list_images)

            cur_image = AUDIMethods.map_lidar_points_onto_image(images[i], lidars[i], colors)
            mapped_images.append(cur_image)

            plt.fig = plt.figure(figsize=(20, 20))
            plt.imshow(cur_image)
            plt.axis('off')
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


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Finds parameters for clusterization algorithms"
    )
    parser.add_argument(
        "lidar_path", type=str, help="Directory where main information files are stored"
    )
    parser.add_argument(
        "images_path", type=str, help="Directory where main information files are stored"
    )
    parser.add_argument(
        "start_indx", type=int, help="Start index"
    )
    parser.add_argument(
        "num_of_shots", type=int, help="Number of shots to optimise"
    )
    parser.add_argument(
        "default", type=int, help="Use default parameters or not"
    )
    parser.add_argument(
        "generation_goal", type=int, help="Number of generations in GA"
    )
    parser.add_argument(
        "population_size", type=int, help="Number of samples in population"
    )
    parser.add_argument(
        "fitness_function", type=int, help="Which fitness functon to use for optimisation: "
                                           "1 -- silhouette score, 2 -- davies-bouldin score, 3 -- calinski-harabasz"
    )
    parser.add_argument(
        "mode", type=int, help="Choose the mode: 1 -- point clouds, 2 -- point clouds with distances, "
                               "3 -- mapped images, 4 -- with editing"
    )
    parser.add_argument(
        "verbose", type=bool, help="Print information for each generation"
    )

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
        args.verbose
    )
