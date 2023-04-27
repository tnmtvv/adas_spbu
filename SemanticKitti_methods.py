from itertools import chain
from typing import Dict, Set

import numpy as np
import numpy.typing as npt
import open3d as o3d
import yaml

from LabeledPcd import LabeledPcd
from Plane import Plane

EXTENSIONS_SCAN = [".bin"]
EXTENSIONS_LABEL = [".label"]


def read_pcd_from_bin(bin_file) -> o3d.geometry.PointCloud:
    np_points = np.fromfile(bin_file, dtype=np.float32)
    pcd_points = np_points.reshape(-1, 4)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pcd_points[:, :3])
    return pcd


def read_labels(label_file):
    # open label
    print(label_file)
    label = np.fromfile(label_file, dtype=np.uint32)
    label = label.reshape((-1))
    upper_half = label >> 16  # get upper half for instances
    lower_half = label & 0xFFFF  # get lower half for semantics
    label = (upper_half << 16) + lower_half  # reconstruct full label
    return label.astype(np.uint32), lower_half


def match_clusters_most_frequent(gt_label: int, pcd: LabeledPcd, indices: npt.NDArray, map_association: Dict, map_cluster_num_points: Dict):
    raw_labels = pcd.raw_labels[indices]
    for label in set(raw_labels):
        num_of_labeled_points = len(np.where(raw_labels == label)[0].tolist())
        if (label in map_association and map_cluster_num_points[label] < num_of_labeled_points) or label not in map_cluster_num_points:
            map_association[label] = gt_label
            map_cluster_num_points[label] = num_of_labeled_points
    return max(set(raw_labels), key=raw_labels.tolist().count)


def match_clusters_best_IoU(gt_label: int, pcd: LabeledPcd, true_indices: npt.NDArray, map_association: Dict, external_iou_map: Dict):
    local_raw_labels = pcd.raw_labels[true_indices]
    # o3d.visualization.draw_geometries([pcd.pcd.select_by_index(true_indices)])

    unique_labels = set(local_raw_labels)
    IoUs = {}
    for label in unique_labels:
        all_raw_label_indices = list(np.where(pcd.raw_labels == label)[0])
        local_raw_label_indices = list(np.where(local_raw_labels == label)[0])
        # o3d.visualization.draw_geometries([pcd.pcd.select_by_index(all_raw_label_indices)])

        union = set(all_raw_label_indices).union(set(true_indices))
        intersection = local_raw_label_indices
        cur_iou = float(len(intersection) / len(union))

        IoUs[cur_iou] = label
        if (label in external_iou_map and external_iou_map[label] < cur_iou) or label not in external_iou_map:
            external_iou_map[label] = cur_iou
            map_association[label] = gt_label
    return IoUs[max(IoUs.keys())], max(IoUs.keys())


def extract_necessary_indices(pcd: LabeledPcd, necessary_labels: Set):
    sem_labels_of_interest = set(pcd.sem_labels).intersection(set(necessary_labels))
    indices_of_interest = []
    for sem_label in sem_labels_of_interest:
        cur_indices_of_interest = (np.where(pcd.sem_labels == sem_label)[0]).tolist()
        # o3d.visualization.draw_geometries([pcd.pcd.select_by_index(list(cur_indices_of_interest))])
        indices_of_interest.append(cur_indices_of_interest)
    flatten_indices_of_interest = list(chain.from_iterable(indices_of_interest))
    return flatten_indices_of_interest


def evaluate_IoU(pcd: LabeledPcd, map_raw_true: Dict, flatten_indices_of_interest):
    overall_IoU = 0.0
    map_true_raw = {}
    map_label_iou = {}
    # o3d.visualization.draw_geometries([pcd.pcd.select_by_index(flatten_indices_of_interest)])

    gt_labels_of_interest = pcd.gt_labels[flatten_indices_of_interest]
    sem_labels_of_interest = pcd.sem_labels[flatten_indices_of_interest]
    # getting labels of instances, choosing indices on intersection gt and sem
    unique_gt_labels_of_interest = set(gt_labels_of_interest)
    small_clusters = 0
    for label in unique_gt_labels_of_interest:
        gt_label_indices = np.where(pcd.gt_labels == label)[0]
        if len(gt_label_indices) < 10:
            small_clusters += 1
        else:
        # o3d.visualization.draw_geometries([pcd.pcd.select_by_index(gt_label_indices)])
            alg_label, best_cur_iou = match_clusters_best_IoU(
                label, pcd, gt_label_indices, map_raw_true, map_label_iou,
            )

            map_true_raw[label] = alg_label
            overall_IoU += best_cur_iou

    overall_IoU = overall_IoU / (len(unique_gt_labels_of_interest) - small_clusters)
    return overall_IoU, map_true_raw
    # if pcds have different amount of unique labels, coefficient is calculated over the max


def get_AoI_indices(cur_pcd: o3d.data.PCDPointCloud):
    points_to_filter = np.asarray(cur_pcd.points)[:, 0]
    indices = np.where(points_to_filter > 0)[0].tolist()
    return indices