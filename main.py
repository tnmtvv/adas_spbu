import argparse
import ast
import os.path
import random
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


def colours_from_reflectances(reflectances):
    return np.stack([reflectances, reflectances, reflectances], axis=1)


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


def create_open3d_pc(lidar, cam_image=None):
    # create open3d point cloud
    pcd = o3d.geometry.PointCloud()

    # assign point coordinates
    pcd.points = o3d.utility.Vector3dVector(lidar['points'])

    # assign colours
    if cam_image is None:
        median_reflectance = np.median(lidar['reflectance'])
        colours = colours_from_reflectances(lidar['reflectance']) / (median_reflectance * 5)

    # clip colours for visualisation on a white background
        colours = np.clip(colours, 0, 0.75)
    else:
        rows = (lidar['row'] + 0.5).astype(np.int)
        cols = (lidar['col'] + 0.5).astype(np.int)
        colours = cam_image[rows, cols, :] / 255.0

    pcd.colors = o3d.utility.Vector3dVector(colours)

    return pcd


def build_point_clouds(root_path):
    file_names = os.listdir(root_path)
    list_pcds = []

    for file in file_names:
        data = np.load(os.path.join(root_path, file))

        lidar_front_center = {'points': data['pcloud_points'], 'reflectance': data['pcloud_attr.reflectance'],
                              'timestamp': data['pcloud_attr.timestamp.npy'], 'rows': data['pcloud_attr.row.npy'],
                              'cols': data['pcloud_attr.col.npy'], 'distance': data['pcloud_attr.distance.npy'],
                              'depth': data['pcloud_attr.depth.npy'], 'lidar_ids': data['pcloud_attr.lidar_id.npy']}
        pcd_front_center = create_open3d_pc(lidar_front_center)
        list_pcds.append(pcd_front_center)

    return list_pcds


def ransac_segmentation(list_pcds, distance_threshold=0.3):
    list_outliers = []
    list_inliers = []

    for pcd in list_pcds:
        plane_model, inliers = pcd.segment_plane(distance_threshold=distance_threshold, ransac_n=3, num_iterations=1000)

        inlier_cloud = pcd.select_by_index(inliers)
        outlier_cloud = pcd.select_by_index(inliers, invert=True)

        list_outliers.append(outlier_cloud)
        list_inliers.append(inlier_cloud)
    return list_outliers, list_inliers


def main(path_to_data: str, num_shots_to_optimise: int):

    ga = My_GA(num_shots_to_optimise, 1)

    pcds = build_point_clouds(path_to_data)
    ga.pcds_outliers, ga.pcds_inliers = ransac_segmentation(pcds, 0.3)

    ga.chromosome_length = 2
    ga.population_size = 20
    ga.generation_goal = 5

    ga.chromosome_impl = lambda: [
        random.uniform(0.1, 3),
        random.randrange(5, 15)
    ]

    ga.crossover_population_impl = Crossover.Population.random

    ga.fitness_function_impl = ga.fitting_function
    ga.target_fitness_type = 'max'

    while ga.active():
        ga.evolve(1)
        print('------------------------------')
        ga.print_population()
        ga.print_best_chromosome()
        ga.print_worst_chromosome()
        print('------------------------------')

    clustering = \
        skc.DBSCAN(eps=2, min_samples=10).fit(np.asarray(ga.pcds_outliers[0].points))
    fitness = metrics.silhouette_score(np.asarray(ga.pcds_outliers[0].points), clustering.labels_)

    print("manually: " + str(fitness))

    best_params = ga.population[0]
    for pcd_outlier, pcd_inlier in zip(ga.pcds_outliers, ga.pcds_inliers):
        clustering = skc.DBSCAN(eps=best_params[0].value, min_samples=best_params[1].value).fit(np.asarray(pcd_outlier.points))
        labels = clustering.labels_
        unique_labels = np.unique(labels)
        set_colors = set()

        colors = np.zeros(np.shape(pcd_outlier.points))
        for label in unique_labels:
            if label < 0:
                colors[np.where(labels == label)] = [0, 0, 0]
            else:
                cur_color = plt.get_cmap("prism")(random.randint(1, len(unique_labels)))[:3]
                while cur_color in set_colors or cur_color == [0, 0, 0]:
                    cur_color = plt.get_cmap("prism")(random.randint(0, label))[:3]
                colors[np.where(labels == label)] = cur_color
                set_colors.add(cur_color)

        pcd_outlier.colors = o3d.utility.Vector3dVector(colors)
        o3d.visualization.draw_geometries([pcd_outlier, pcd_inlier])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Finds parameters for clusterization algorithms"
    )
    parser.add_argument(
        "data_path", type=str, help="Directory where main information files are stored"
    )
    parser.add_argument(
        "num_of_shots", type=int, help="Number of shots to optimise"
    )

    args = parser.parse_args()

    main(
        args.data_path,
        args.num_of_shots,
    )
