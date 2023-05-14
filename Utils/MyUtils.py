import random

import numpy as np
import open3d as o3d
import sklearn
from EasyGA.crossover import Crossover
from EasyGA.mutation import Mutation
from matplotlib import pyplot as plt
from numpy import double

from Utils.Plane import Plane


def align_color(label, set_colors):
    if label < 0:
        return [0, 0, 0]
    else:
        cur_color = tuple(plt.get_cmap('Spectral')(random.randint(1, 1000000))[:3])
        while cur_color in set_colors:
            cur_indx = random.randint(1, 1000000)
            cur_color = tuple(plt.get_cmap('Spectral')(random.randint(1, 1000000))[:3])
        set_colors.add(cur_color)
        return np.asarray(cur_color)


def find_left_right(inlier_points):
    sorted_points = sorted(inlier_points, key=lambda x: x[1])
    return sorted_points[0][1], sorted_points[-1][1]


# def find_min_z(points):
#     min_z = list(sorted(points, key=lambda x: x[-1]))[0]
#     return min_z

def find_min_z(points):
    sorted_points = list(sorted(points, key=lambda x: x[-1]))
    cur_min_z = sorted_points[0]
    next_min_z = sorted_points[1]
    i = 1
    while np.abs(cur_min_z[-1] - next_min_z[-1]) > 0.7:
        i += 1
        cur_min_z = next_min_z
        next_min_z = sorted_points[i]
    layer = list(filter(lambda x: np.abs(x[2] - cur_min_z[-1]) < 0.7, points))
    eps = 0.25
    while len(layer) < 2000:
        layer = list(filter(lambda x: np.abs(x[2] - cur_min_z[-1]) < 0.7 + eps, points))
        eps += 0.25
    # pcd = o3d.geometry.PointCloud()
    # pcd.points = o3d.utility.Vector3dVector(layer)
    # o3d.visualization.draw_geometries([pcd])
    layer = np.asarray(layer)[:, 2]
    z_to_return = np.mean(layer)
    return z_to_return


def get_projection(inlier, cropped_outlier):
    plane_equation = Plane.get_equation(inlier.points)
    points_projected = []

    for point in cropped_outlier.points:
        projection = point - (np.dot(plane_equation[:-1], point) + plane_equation[-1]) * plane_equation[:-1]
        points_projected.append(projection)
    pcd_projected = o3d.geometry.PointCloud()
    pcd_projected.points = o3d.utility.Vector3dVector(points_projected)
    return pcd_projected


def two_stage_plane_segmentation(cur_pcd, down_sample_coeff=1, if_eval=False):
    if if_eval:
        cur_pcd = cur_pcd.pcd.uniform_down_sample(down_sample_coeff)
    else:
        cur_pcd = cur_pcd.uniform_down_sample(down_sample_coeff)
    points = np.array(cur_pcd.points)

    # extract plane on same level with the lowest point
    min_z_point = find_min_z(points)
    plane_point_indices = np.where(np.abs(points[:, -1] - min_z_point) < 0.15)
    plane_points = points[plane_point_indices]
    #
    # pcd = cur_pcd.select_by_index(plane_point_indices[0])
    # o3d.visualization.draw_geometries([cur_pcd])
    # o3d.visualization.draw_geometries([pcd])

    # build plane equation of the extracted points
    plane_equation = Plane.get_equation(plane_points)

    normal_vector = plane_equation[:-1]
    c = np.mean(plane_points, axis=0)
    d = np.dot(normal_vector, c)

    # get all the points that satisfy the equation
    # second_plane_point_indices = np.where(np.abs(np.asarray(points).dot(normal_vector) - d) <= 0.3)[0]
    _, second_plane_point_indices = cur_pcd.segment_plane(distance_threshold=0.1, ransac_n=3, num_iterations=1000)
    inlier_cloud = cur_pcd.select_by_index(second_plane_point_indices)
    cur_pcd = cur_pcd.select_by_index(second_plane_point_indices, invert=True)

    left_bound, right_bound = find_left_right(list(inlier_cloud.points))

    cur_outlier_points = list(np.asarray(cur_pcd.points))
    cropped_outlier_points = list(filter(lambda x: left_bound <= x[1] <= right_bound, cur_outlier_points))
    cropped_outlier_indices = range(0, len(cur_outlier_points))
    set_outlier_points = set(tuple(x) for x in cropped_outlier_points)

    cropped_outlier_indices = list(
        filter(lambda i: tuple(cur_outlier_points[i]) in set_outlier_points, cropped_outlier_indices))

    cur_pcd = cur_pcd.select_by_index(cropped_outlier_indices)
    # o3d.visualization.draw_geometries([inlier_cloud])
    # o3d.visualization.draw_geometries([cur_pcd])
    return cur_pcd, inlier_cloud, second_plane_point_indices


def plane_segmentation(gt_pcds, down_smple_coeff=1):
    all_indices = []
    inlier_clouds = []
    for i, _ in enumerate(gt_pcds):
        gt_pcds[i], inlier_cloud, indices = two_stage_plane_segmentation(gt_pcds[i], down_smple_coeff)
        all_indices.append(indices)
        inlier_clouds.append(inlier_cloud)

    return gt_pcds, inlier_clouds, all_indices


def paint_cloud(cloud, raw_labels):
    one_more_set = set()
    colors_cloud_raw = np.zeros(np.shape(cloud.points))
    for label in set(raw_labels):
        cur_color = align_color(label, one_more_set)
        cur_indices = np.where(raw_labels == label)[0].tolist()
        # label_indices_of_interest = list(set(cur_indices).intersection(set(flatten_indices_of_interest)))
        colors_cloud_raw[cur_indices] = cur_color
    cloud.colors = o3d.utility.Vector3dVector(colors_cloud_raw)
    return cloud


def parse_parameters(params: str):
    best_params_str = params[1:-1]
    best_params = list(map(lambda x: double(x), best_params_str.split(',')))
    return best_params


def parse_parameters_info(parameters_types):
    list_types = []
    list_params = []
    for str_param in parameters_types:
        cur_list = str_param.split(':')
        list_types.append(cur_list[1])
        list_params.append(cur_list[0])
    return list_types, list_params


def to_list(lst):
    while len(lst) != 1:
        lst[0].append(lst[-1])
        lst = lst[:-1]
    return lst
