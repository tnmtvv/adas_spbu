import random

import numpy as np
import open3d as o3d
from matplotlib import pyplot as plt
from Plane import Plane


def align_color(label, set_colors):
    if label < 0:
        return [0, 0, 0]
    else:
        cur_color = tuple(plt.get_cmap('Spectral')(random.randint(1, 1000000))[:3])
        while cur_color in set_colors:
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
