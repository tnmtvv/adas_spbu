import os.path
from typing import Callable

import MyUtils
import numpy as np
import open3d as o3d
import SemanticKitti_methods
from AUDI_methods import AUDIMethods
from LabeledPcd import LabeledPcd
import yaml


#  adopted from https://www.a2d2.audi/a2d2/en/tutorial.html
def build_point_clouds_and_lidars(lidar_list, num_shots):
    list_pcds = []
    list_lidars = []

    for file in lidar_list:
        data = np.load(file)

        lidar_front_center = {
            "points": data["pcloud_points"],
            "reflectance": data["pcloud_attr.reflectance"],
            "timestamp": data["pcloud_attr.timestamp.npy"],
            "row": data["pcloud_attr.row.npy"],
            "col": data["pcloud_attr.col.npy"],
            "distance": data["pcloud_attr.distance.npy"],
            "depth": data["pcloud_attr.depth.npy"],
            "lidar_ids": data["pcloud_attr.lidar_id.npy"],
        }
        pcd_front_center = AUDIMethods.create_open3d_pc(lidar_front_center)
        list_pcds.append(pcd_front_center)
        list_lidars.append(lidar_front_center)
    return list_pcds[:num_shots], list_lidars[:num_shots]


# adopted from https://github.com/PRBonn/semantic-kitti-api/blob/master/auxiliary/laserscan.py
def extract_sem_kitti_pcds(bin_list, label_list, map_label_color, get_AoF: Callable = None):
    gt_pcds = []

    if len(bin_list) == len(label_list):
        for i, bin_file in enumerate(bin_list):
            print(bin_file)
            cur_pcd = SemanticKitti_methods.read_pcd_from_bin(bin_file)
            if get_AoF:
                indices = get_AoF(cur_pcd)
                cur_pcd = cur_pcd.select_by_index(indices=indices)
            gt_pcds.append(cur_pcd)
    return gt_pcds
    # gt_labeled_pcds = []
    #
    # with open("config.yaml", "r") as stream:
    #     try:
    #         data = yaml.safe_load(stream)
    #     except yaml.YAMLError as exc:
    #         print(exc)
    # map_sem_color = data['color_map']
    # necessary_labels = data['necessary_labels']
    #
    # if len(bin_list) == len(label_list):
    #     for i, bin_file in enumerate(bin_list):
    #         cur_pcd = SemanticKitti_methods.read_pcd_from_bin(bin_file)
    #         cur_true_labels, cur_sem_labels = SemanticKitti_methods.read_labels(label_list[i])
    #         cur_colors = np.zeros(np.shape(cur_pcd.points))
    #         cur_sem_colors = np.zeros(np.shape(cur_pcd.points))
    #
    #         indices = []
    #         set_colors = set()
    #
    #         if get_AoF:
    #             indices = get_AoF(cur_pcd)
    #             cur_pcd = cur_pcd.select_by_index(indices=indices)
    #             cur_sem_labels = cur_sem_labels[indices]
    #
    #         for sem_label in set(cur_sem_labels):
    #             cur_indices = np.where(cur_sem_labels == sem_label)[0].tolist()
    #             cropped_indices = np.ma.intersect1d(indices, cur_indices)
    #             cur_sem_colors[cropped_indices] = map_sem_color[sem_label]
    #
    #         if not map_label_color:
    #             map_label_color[-1] = (0, 0, 0)
    #             inst_color_lut = np.random.uniform(
    #                 low=0.0, high=1.0, size=(len(set(cur_true_labels)), 3)
    #             )
    #             for i, label in enumerate(set(cur_true_labels)):
    #                 map_label_color[label] = inst_color_lut[i, :]
    #                 set_colors.add(tuple(map_label_color[label]))
    #                 cur_indices = np.where(cur_true_labels == label)[0].tolist()
    #                 cropped_indices = np.ma.intersect1d(indices, cur_indices)
    #                 cur_colors[cropped_indices] = map_label_color[label]
    #         else:
    #             for i, label in enumerate(set(cur_true_labels)):
    #                 if label not in map_label_color.keys():
    #                     map_label_color[label] = MyUtils.align_color(
    #                         label, set_colors
    #                     )
    #                 cur_indices = np.where(cur_true_labels == label)[0].tolist()
    #                 if cur_indices:
    #                     cropped_indices = np.ma.intersect1d(indices, cur_indices)
    #                     cur_colors[cropped_indices] = map_label_color[label]
    #
    #         cur_colors = cur_colors[indices]
    #         cur_pcd.colors = o3d.utility.Vector3dVector(cur_colors)
    #         gt_labeled_pcds.append(LabeledPcd(cur_pcd, gt_labels=cur_true_labels[indices], sem_labels=cur_sem_labels,
    #                                           sem_colors=cur_sem_colors, true_colors=cur_colors))
    # return gt_labeled_pcds, necessary_labels


def create_data_lists(
    path_to_main_data: str,
    path_to_sub_data: str,
    indx_from=0,
    indx_to=-1,
    separator="_",
    audi_mode=False,
):
    main_files = os.listdir(path_to_main_data)
    sub_files = os.listdir(path_to_sub_data)

    if audi_mode:
        sub_data_files = list(filter(lambda x: x.endswith(".png"), sub_files))
        sub_files = sub_data_files
    main_files = sorted(main_files, key=lambda x: int(x.split(separator)[0]))
    sub_files = sorted(sub_files, key=lambda x: int(x.split(separator)[0]))

    list_main_data = list(map(lambda x: os.path.join(path_to_main_data, x), main_files))
    list_sub_data = list(map(lambda x: os.path.join(path_to_sub_data, x), sub_files))

    if indx_to < 0:
        indx_to = len(list_main_data)
    return list_main_data[indx_from:indx_to], list_sub_data[indx_from:indx_to]
