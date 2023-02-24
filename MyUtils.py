import random
from typing import List

import numpy as np
import open3d as o3d
from matplotlib import pyplot as plt
from Plane import Plane



def align_color(label, set_colors):
    if label < 0:
        return [0, 0, 0]
    else:
        cur_color = plt.get_cmap('Spectral')(random.randint(1, 1000000))[:3]
        while cur_color in set_colors:
            cur_color = plt.get_cmap('Spectral')(random.randint(1, 1000000))[:3]
        set_colors.add(cur_color)
        return cur_color


def get_projection(inlier, cropped_outlier):
    plane_equation = Plane.get_equation(inlier.points)
    points_projected = []

    for point in cropped_outlier.points:
        projection = point - (np.dot(plane_equation[:-1], point) + plane_equation[-1]) * plane_equation[:-1]
        points_projected.append(projection)
    pcd_projected = o3d.geometry.PointCloud()
    pcd_projected.points = o3d.utility.Vector3dVector(points_projected)
    return pcd_projected
