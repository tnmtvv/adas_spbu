from AUDI_methods import AUDIMethods

import os.path
import cv2

import numpy as np


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


def create_lidar_image_lists(path_to_lidar: str, path_to_images: str, indx_from=0, indx_to=-1):
    lidar_files = os.listdir(path_to_lidar)
    lidar_images = os.listdir(path_to_images)
    images_files = list(filter(lambda x: x.endswith(".png"), lidar_images))

    lidar_files = sorted(lidar_files, key=lambda x: int(x.split('_')[0]))
    images_files = sorted(images_files, key=lambda x: int(x.split('_')[0]))

    list_lidar = list(map(lambda x: os.path.join(path_to_lidar, x), lidar_files))
    list_images = list(map(lambda x: os.path.join(path_to_images, x), images_files))

    if indx_to < 0:
        indx_to = len(list_lidar)

    return list_lidar[indx_from: indx_to], list_images[indx_from: indx_to]
