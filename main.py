import argparse
import ast
import os.path
import random

from EasyGA.mutation import Mutation
from AUDI_methods import AUDIMethods
from os.path import join
import glob
import numpy as np
import open3d as o3d
from EasyGA.crossover import Crossover
from sklearn import cluster as skc
from sklearn import metrics
from matplotlib import pyplot as plt
import EasyGA
from Custom_GA import My_GA
import cv2

from Plane import Plane


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


def build_point_clouds_and_lidars(lidar_list):

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

    return list_pcds, list_lidars


def extract_images(images_path_list):
    images_list = []

    for file_name in images_path_list:
        images_list.append(cv2.imread(file_name))
    return images_list


def create_lidar_image_lists(path_to_lidar: str, path_to_images: str):
    lidar_files = os.listdir(path_to_lidar)
    images_files = glob.glob(path_to_images + '/*.png')

    lidar_files = sorted(lidar_files, key=lambda x: int(x[-13:-4]))
    images_files = sorted(images_files, key=lambda x: int(x[-13:-4]))

    list_lidar = list(map(lambda x: os.path.join(path_to_lidar, x), lidar_files))
    list_images = list(map(lambda x: os.path.join(path_to_images, x), images_files))

    return list_lidar, list_images


def ransac_segmentation(list_pcds, distance_threshold=0.4):
    list_outliers = []
    list_inliers = []
    list_indx = []

    for pcd in list_pcds:
        plane_model, inliers = pcd.segment_plane(distance_threshold=distance_threshold, ransac_n=3, num_iterations=1000)

        inlier_cloud = pcd.select_by_index(inliers)
        outlier_cloud = pcd.select_by_index(inliers, invert=True)

        list_outliers.append(outlier_cloud)
        list_inliers.append(inlier_cloud)
        list_indx.append(inliers)
    return list_outliers, list_inliers, list_indx


def get_progection(equation, coordinates):
    # (x - a)/equaion[0] = (y - b)/equation[1] = (z - c)/equation[2]
    pass


def main(path_to_lidar: str, path_to_images: str,
         num_shots_to_optimise: int, generation_goal: int, population_size: int, fitness_function: int, mode: int, verbose=False):

    list_lidar, list_images = create_lidar_image_lists(path_to_lidar, path_to_images)

    pcds, lidars = build_point_clouds_and_lidars(list_lidar)
    images = extract_images(list_images)

    if num_shots_to_optimise == 0:
        num_shots_to_optimise = len(list_lidar) - 1

    ga = My_GA(num_shots_to_optimise, fitness_function, generation_goal, population_size)
    ga.chromosome_length = 2
    ga.pcds_outliers, ga.pcds_inliers, list_indx = ransac_segmentation(pcds, 0.125)

    for i, inlier in enumerate(ga.pcds_inliers):
        plane_equation = Plane.get_equation(inlier.points)
        print(plane_equation)
        points_projected = []

        for point in ga.pcds_outliers[i].points:
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

    while ga.active():
        ga.evolve(1)
        if verbose:
            print('------------------------------')
            ga.print_population()
            ga.print_best_chromosome()
            ga.print_worst_chromosome()
            print('------------------------------')

    best_params = ga.population[0]

    for i, pcd in enumerate(zip(ga.pcds_outliers, ga.pcds_inliers)):
        pcd_outlier, pcd_inlier = pcd

        clustering = skc.DBSCAN(eps=best_params[0].value, min_samples=best_params[1].value).fit(np.asarray(pcd_outlier.points))
        labels = clustering.labels_
        unique_labels = np.unique(labels)
        set_colors = set()

        merged_pcd = pcd_inlier + pcd_outlier
        colors = np.zeros(np.shape(merged_pcd.points))
        colors_outlier = np.zeros(np.shape(pcd_outlier.points))

        for label in unique_labels:
            if label < 0:
                colors_outlier[np.where(labels == label)] = [0, 0, 0]
            else:
                cur_color = plt.get_cmap("prism")(random.randint(1, len(unique_labels)))[:3]
                colors_outlier[np.where(labels == label)] = cur_color
                set_colors.add(cur_color)

        pcd_outlier.colors = o3d.utility.Vector3dVector(colors_outlier)
        merged_pcd_1 = pcd_inlier + pcd_outlier

        if mode == 1:  # only point clouds
            o3d.visualization.draw_geometries([merged_pcd_1])
        elif mode == 2: # mapped images
            mapped_images = []

            cur_image = AUDIMethods.map_lidar_points_onto_image(images[i], lidars[i], colors)
            mapped_images.append(cur_image)

            plt.fig = plt.figure(figsize=(20, 20))
            plt.imshow(cur_image)
            plt.axis('off')
            plt.close()
        elif mode == 3:  # with editing
            cur_inlier_indices = list_indx[i]

            mask = np.ones(colors.shape[0], bool)
            mask[cur_inlier_indices] = 0
            colors[mask] = colors_outlier

            vis = o3d.visualization.VisualizerWithEditing()
            vis.create_window()
            vis.add_geometry(merged_pcd_1)
            vis.run()  # user picks points
            vis.destroy_window()
            picked_points = vis.get_picked_points()

            mismatched_points = merged_pcd_1.select_by_index(picked_points)

            dists = pcd_inlier.compute_point_cloud_distance(mismatched_points)
            dists = np.asarray(dists)
            min_dist = np.min(dists)
            print (min_dist)


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
        "num_of_shots", type=int, help="Number of shots to optimise"
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
        "mode", type=int, help="Choose the mode: 1 -- point clouds, 2 -- mapped images, 3 -- with editing"
    )
    parser.add_argument(
        "verbose", type=bool, help="Print information for each generation"
    )

    args = parser.parse_args()

    main(
        args.lidar_path,
        args.images_path,
        args.num_of_shots,
        args.generation_goal,
        args.population_size,
        args.fitness_function,
        args.mode,
        args.verbose
    )
